/*
  ESP32_CAN.cpp - Library for ESP32 built-in CAN module
    Now converted to use the built-in TWAI driver in ESP-IDF. This should allow for support for the
    full range of ESP32 hardware and probably be more stable than the old approach.
  
  Author: Collin Kidder
  
  Created: 31/1/18, significant rework 1/5/23
*/

#include "Arduino.h"
#include "esp32_can_builtin.h"
//because of the way the TWAI library works, it's just easier to store the valid timings here and anything not found here
//is just plain not supported. If you need a different speed then add it here. Be sure to leave the zero record at the end
//as it serves as a terminator
const VALID_TIMING valid_timings[] = 
{
    {TWAI_TIMING_CONFIG_1MBITS(), 1000000},
    {TWAI_TIMING_CONFIG_500KBITS(), 500000},
    {TWAI_TIMING_CONFIG_250KBITS(), 250000},
    {TWAI_TIMING_CONFIG_125KBITS(), 125000},
    {TWAI_TIMING_CONFIG_800KBITS(), 800000},
    {TWAI_TIMING_CONFIG_100KBITS(), 100000},
    {TWAI_TIMING_CONFIG_50KBITS(), 50000},
    {TWAI_TIMING_CONFIG_25KBITS(), 25000},
    //caution, these next entries are custom and haven't really been fully tested yet.
    //Note that brp can take values in multiples of 2 up to 128 and multiples of 4 up to 256
    //TSEG1 can be 1 to 16 and TSEG2 can be 1 to 8. There is a silent +1 added to the sum of these two.
    //The default clock is 80MHz so plan accordingly
    {{.brp = 100, .tseg_1 = 7, .tseg_2 = 2, .sjw = 3, .triple_sampling = false}, 80000}, 
    {{.brp = 120, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}, 33333},
    //this one is only possible on ECO2 ESP32 or ESP32-S3 not on the older ESP32 chips
    {{.brp = 200, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}, 20000},
    {TWAI_TIMING_CONFIG_25KBITS(), 0} //this is a terminator record. When the code sees an entry with 0 speed it stops searching
};

ESP32CAN::ESP32CAN(gpio_num_t rxPin, gpio_num_t txPin, uint8_t busNumber) : CAN_COMMON(32)
{
    twai_general_cfg.rx_io = rxPin;
    twai_general_cfg.tx_io = txPin;
    cyclesSinceTraffic = 0;
    readyForTraffic = false;
    twai_general_cfg.tx_queue_len = BI_TX_BUFFER_SIZE;
    twai_general_cfg.rx_queue_len = 6;
    rxBufferSize = BI_RX_BUFFER_SIZE;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    bus_handle = nullptr;
    twai_general_cfg.controller_id = busNumber;
#endif
}

ESP32CAN::ESP32CAN() : CAN_COMMON(BI_NUM_FILTERS) 
{
    twai_general_cfg.tx_queue_len = BI_TX_BUFFER_SIZE;
    twai_general_cfg.rx_queue_len = 6;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    bus_handle = nullptr;
#endif
    rxBufferSize = BI_RX_BUFFER_SIZE;

    for (int i = 0; i < BI_NUM_FILTERS; i++)
    {
        filters[i].id = 0;
        filters[i].mask = 0;
        filters[i].extended = false;
        filters[i].configured = false;
    }

    readyForTraffic = false;
    cyclesSinceTraffic = 0;
}

void ESP32CAN::setCANPins(gpio_num_t rxPin, gpio_num_t txPin)
{
    twai_general_cfg.rx_io = rxPin;
    twai_general_cfg.tx_io = txPin;
}

void ESP32CAN::CAN_WatchDog_Builtin( void *pvParameters )
{
    ESP32CAN* espCan = (ESP32CAN*)pvParameters;
    const TickType_t xDelay = 200 / portTICK_PERIOD_MS;
    twai_status_info_t status_info;

    for(;;)
    {
        vTaskDelay( xDelay );
        espCan->cyclesSinceTraffic++;

        esp_err_t result;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
        result = twai_get_status_info_v2(espCan->bus_handle, &status_info);
#else
        result = twai_get_status_info(&status_info);
#endif
        if (result == ESP_OK)
        {
            if (status_info.state == TWAI_STATE_BUS_OFF)
            {
                espCan->cyclesSinceTraffic = 0;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
                result = twai_initiate_recovery_v2(espCan->bus_handle);
#else
                result = twai_initiate_recovery();
#endif
                if (result != ESP_OK)
                {
                    printf("Could not initiate bus recovery!\n");
                }
            }
        }
    }
}

//infinitely loops accepting frames from the TWAI driver. Calls
//our processing routine which then applies the custom 32 filters and
//decides whether to trigger callbacks or queue the frame (or throw it away)
void ESP32CAN::task_LowLevelRX(void *pvParameters)
{
    ESP32CAN* espCan = (ESP32CAN*)pvParameters;
    
    while (1)
    {
        twai_message_t message;
        if (espCan->readyForTraffic)
        {
            esp_err_t result;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
            result = twai_receive_v2(espCan->bus_handle, &message, pdMS_TO_TICKS(100));
#else
            result = twai_receive(&message, pdMS_TO_TICKS(100));
#endif
            if (result == ESP_OK)
            {
                espCan->processFrame(message);
            }
        }
        else vTaskDelay(pdMS_TO_TICKS(100));
    }
    
}

/*
Issue callbacks to registered functions and objects
Used to keep this kind of thing out of the interrupt handler
The callback type and mailbox are passed in the fid member of the
CAN_FRAME struct. It isn't really used by anything.
Layout of the storage:
bit   31 -    If set indicates an object callback
bits  24-30 - Idx into listener table
bits  0-7   - Mailbox number that triggered callback
*/
void ESP32CAN::task_CAN( void *pvParameters )
{
    ESP32CAN* espCan = (ESP32CAN*)pvParameters;
    CAN_FRAME rxFrame;

    //delay a bit upon initial start up
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1)
    {
        if (uxQueueMessagesWaiting(espCan->callbackQueue)) {
            //receive next CAN frame from queue and fire off the callback
            if(xQueueReceive(espCan->callbackQueue, &rxFrame, portMAX_DELAY) == pdTRUE)
            {
                espCan->sendCallback(&rxFrame);
            }
        }
        else vTaskDelay(pdMS_TO_TICKS(4)); //if you don't delay here it will slow down the whole system. Need some delay.

//probably don't need this extra delay. Test and find out.
#if defined(CONFIG_FREERTOS_UNICORE)
    vTaskDelay(pdMS_TO_TICKS(6)); 
#endif
    }

    vTaskDelete(NULL);
}

void ESP32CAN::sendCallback(CAN_FRAME *frame)
{
    //frame buffer
    CANListener *thisListener;
    int mb;
    int idx;

    mb = (frame->fid & 0xFF);
    if (mb == 0xFF) mb = -1;

    if (frame->fid & 0x80000000ul) //object callback
    {
        idx = (frame->fid >> 24) & 0x7F;
        thisListener = listener[idx];
        thisListener->gotFrame(frame, mb);
    }
    else //C function callback
    {
        if (mb > -1) (*cbCANFrame[mb])(frame);
        else (*cbGeneral)(frame);
    }
}

void ESP32CAN::setRXBufferSize(int newSize)
{
    rxBufferSize = newSize;
}

void ESP32CAN::setTXBufferSize(int newSize)
{
    twai_general_cfg.tx_queue_len = newSize;
}

int ESP32CAN::_setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended)
{
    if (mailbox < BI_NUM_FILTERS)
    {
        filters[mailbox].id = id & mask;
        filters[mailbox].mask = mask;
        filters[mailbox].extended = extended;
        filters[mailbox].configured = true;
        return mailbox;
    }
    return -1;
}

int ESP32CAN::_setFilter(uint32_t id, uint32_t mask, bool extended)
{
    for (int i = 0; i < BI_NUM_FILTERS; i++)
    {
        if (!filters[i].configured) 
        {
            _setFilterSpecific(i, id, mask, extended);
            return i;
        }
    }
    if (debuggingMode) Serial.println("Could not set filter!");
    return -1;
}

void ESP32CAN::_init()
{
    if (debuggingMode) Serial.println("Built in CAN Init");
    for (int i = 0; i < BI_NUM_FILTERS; i++)
    {
        filters[i].id = 0;
        filters[i].mask = 0;
        filters[i].extended = false;
        filters[i].configured = false;
    }

    if (!CAN_WatchDog_Builtin_handler) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
        std::ostringstream canWatchDogTaskNameStream;
        canWatchDogTaskNameStream << "CAN_WD_BI_CAN" << twai_general_cfg.controller_id;
        const char *canWatchDogTaskName = canWatchDogTaskNameStream.str().c_str();
#else
        const char *canWatchDogTaskName = "CAN_WD_BI";
#endif

#if defined(CONFIG_FREERTOS_UNICORE)
        xTaskCreate(&CAN_WatchDog_Builtin, canWatchDogTaskName, 2048, this, 10, &CAN_WatchDog_Builtin_handler);
#else
        xTaskCreatePinnedToCore(&CAN_WatchDog_Builtin, canWatchDogTaskName, 2048, this, 10, &CAN_WatchDog_Builtin_handler, 1);
#endif
    }
    if (debuggingMode) Serial.println("_init done");
}

uint32_t ESP32CAN::init(uint32_t ul_baudrate)
{
    ESP_LOGD("CAN", "Init called");
    _init();
    ESP_LOGD("CAN", "Init done");
    set_baudrate(ul_baudrate);
    ESP_LOGD("CAN", "Baudrate set");
    if (debuggingMode)
    {
        //Reconfigure alerts to detect Error Passive and Bus-Off error states
        uint32_t alerts_to_enable = TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_AND_LOG | TWAI_ALERT_ERR_ACTIVE 
                                  | TWAI_ALERT_ARB_LOST | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL;

        esp_err_t result;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
        result = twai_reconfigure_alerts_v2(bus_handle, alerts_to_enable, NULL);
#else
        result = twai_reconfigure_alerts(alerts_to_enable, NULL);
#endif
        if (result == ESP_OK)
        {
            printf("Alerts reconfigured\n");
        }
        else
        {
            printf("Failed to reconfigure alerts");
        }
    }
    //this task implements our better filtering on top of the TWAI library. Accept all frames then filter in here VVVVV
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    std::ostringstream canLowLevelTaskNameStream;
    canLowLevelTaskNameStream << "CAN_LORX_CAN" << twai_general_cfg.controller_id;
    const char* canLowLevelTaskName = canLowLevelTaskNameStream.str().c_str();
#else
    const char* canLowLevelTaskName = "CAN_LORX_CAN0";
#endif

#if defined(CONFIG_FREERTOS_UNICORE)
    xTaskCreate(ESP32CAN::task_LowLevelRX, canLowLevelTaskName, 4096, this, 19, NULL);
#else
    xTaskCreatePinnedToCore(ESP32CAN::task_LowLevelRX, canLowLevelTaskName, 4096, this, 19, NULL, 1);
#endif
    readyForTraffic = true;
    return ul_baudrate;
}

uint32_t ESP32CAN::beginAutoSpeed()
{
    twai_general_config_t oldMode = twai_general_cfg;

    _init();

    readyForTraffic = false;
    twai_stop();
    twai_general_cfg.mode = TWAI_MODE_LISTEN_ONLY;
    int idx = 0;
    while (valid_timings[idx].speed != 0)
    {
        twai_speed_cfg = valid_timings[idx].cfg;
        disable();
        Serial.print("Trying Speed ");
        Serial.print(valid_timings[idx].speed);
        enable();
        delay(600); //wait a while
        if (cyclesSinceTraffic < 2) //only would happen if there had been traffic
        {
            disable();
            twai_general_cfg.mode = oldMode.mode;
            enable();
            Serial.println(" SUCCESS!");
            return valid_timings[idx].speed;
        }
        else
        {
            Serial.println(" FAILED.");
        }
        idx++;
    }
    Serial.println("None of the tested CAN speeds worked!");
    twai_stop();
    return 0;
}

uint32_t ESP32CAN::set_baudrate(uint32_t ul_baudrate)
{
    disable();
    //now try to find a valid timing to use
    int idx = 0;
    while (valid_timings[idx].speed != 0)
    {
        if (valid_timings[idx].speed == ul_baudrate)
        {
            twai_speed_cfg = valid_timings[idx].cfg;
            enable();
            return ul_baudrate;
        }
        idx++;
    }
    printf("Could not find a valid bit timing! You will need to add your desired speed to the library!\n");
    return 0;
}

void ESP32CAN::setListenOnlyMode(bool state)
{
    disable();
    twai_general_cfg.mode = state?TWAI_MODE_LISTEN_ONLY:TWAI_MODE_NORMAL;
    enable();
}

void ESP32CAN::setNoACKMode(bool state)
{
    disable();
    twai_general_cfg.mode = state?TWAI_MODE_NO_ACK:TWAI_MODE_NORMAL;
    enable();
}

void ESP32CAN::enable()
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    if (twai_driver_install_v2(&twai_general_cfg, &twai_speed_cfg, &twai_filters_cfg, &bus_handle) == ESP_OK) {
        printf("Driver installed - bus %d\n", twai_general_cfg.controller_id);
    } else {
        printf("Failed to install driver - bus %d\n", twai_general_cfg.controller_id);
        return;
    }
#else
    if (twai_driver_install(&twai_general_cfg, &twai_speed_cfg, &twai_filters_cfg) == ESP_OK)
    {
        printf("TWAI Driver installed\n");
    }
    else
    {
        printf("Failed to install TWAI driver\n");
        return;
    }
#endif

    printf("Creating queues\n");

    callbackQueue = xQueueCreate(16, sizeof(CAN_FRAME));
    rx_queue = xQueueCreate(rxBufferSize, sizeof(CAN_FRAME));

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    std::ostringstream canHandlerTaskNameStream;
    std::ostringstream canLowLevelTaskNameStream;
    canHandlerTaskNameStream << "CAN_RX_CAN" << twai_general_cfg.controller_id;
    canLowLevelTaskNameStream << "CAN_LORX_CAN" << twai_general_cfg.controller_id;
    const char* canHandlerTaskName = canHandlerTaskNameStream.str().c_str();
    const char* canLowLevelTaskName = canLowLevelTaskNameStream.str().c_str();
#else
    const char* canHandlerTaskName = "CAN_RX_CAN";
    const char* canLowLevelTaskName = "CAN_LORX_CAN";
#endif

    printf("Starting can handler task\n");
    xTaskCreate(ESP32CAN::task_CAN, canHandlerTaskName, 8192, this, 15, &task_CAN_handler);

#if defined(CONFIG_FREERTOS_UNICORE)
    printf("Starting low level RX task\n");
    xTaskCreate(ESP32CAN::task_LowLevelRX, canLowLevelTaskName, 4096, this, 19, &task_LowLevelRX_handler);
#else
    //this next task implements our better filtering on top of the TWAI library. Accept all frames then filter in here VVVVV
    xTaskCreatePinnedToCore(&task_LowLevelRX, canLowLevelTaskName, 4096, this, 19, &task_LowLevelRX_handler, 1);
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    //Start TWAI driver
    if (twai_start_v2(bus_handle) == ESP_OK) {
        printf("Driver started - bus %d\n", twai_general_cfg.controller_id);
    } else {
        printf("Failed to start driver\n");
        return;
    }
#else
    // Start TWAI driver
    if (twai_start() == ESP_OK)
    {
        printf("TWAI Driver started\n");
    }
    else
    {
        printf("Failed to start TWAI driver\n");
        return;
    }
#endif

    readyForTraffic = true;
}

void ESP32CAN::disable()
{
    twai_status_info_t info;
    if (twai_get_status_info(&info) == ESP_OK) {
        if (info.state == TWAI_STATE_RUNNING) {
            twai_stop();
        }

        for (auto task : {task_CAN_handler, task_LowLevelRX_handler}) {
            if (task != NULL)
            {
                vTaskDelete(task);
                task = NULL;
            }
        }

        for (auto queue : {rx_queue, callbackQueue}) {
            if (queue) {
                vQueueDelete(queue);
            }
        }

        twai_driver_uninstall();
    } else {
        return;
    }
    readyForTraffic = false;
}

//This function is too big to be running in interrupt context. Refactored so it doesn't.
bool ESP32CAN::processFrame(twai_message_t &frame)
{
    CANListener *thisListener;
    CAN_FRAME msg;

    cyclesSinceTraffic = 0; //reset counter to show that we are receiving traffic

    msg.id = frame.identifier;
    msg.length = frame.data_length_code;
    msg.rtr = frame.rtr;
    msg.extended = frame.extd;
    for (int i = 0; i < 8; i++) msg.data.byte[i] = frame.data[i];
    
    for (int i = 0; i < BI_NUM_FILTERS; i++)
    {
        if (!filters[i].configured) continue;
        if ((msg.id & filters[i].mask) == filters[i].id && (filters[i].extended == msg.extended))
        {
            //frame is accepted, lets see if it matches a mailbox callback
            if (cbCANFrame[i])
            {
                msg.fid = i;
                xQueueSend(callbackQueue, &msg, 0);
                return true;
            }
            else if (cbGeneral)
            {
                msg.fid = 0xFF;
                xQueueSend(callbackQueue, &msg, 0);
                return true;
            }
            else
            {
                for (int listenerPos = 0; listenerPos < SIZE_LISTENERS; listenerPos++)
                {
                    thisListener = listener[listenerPos];
                    if (thisListener != NULL)
                    {
                        if (thisListener->isCallbackActive(i)) 
				        {
					        msg.fid = 0x80000000ul + (listenerPos << 24ul) + i;
                            xQueueSend(callbackQueue, &msg, 0);
                            return true;
				        }
				        else if (thisListener->isCallbackActive(numFilters)) //global catch-all 
				        {
                            msg.fid = 0x80000000ul + (listenerPos << 24ul) + 0xFF;
					        xQueueSend(callbackQueue, &msg, 0);
                            return true;
				        }
                    }
                }
            }
            
            //otherwise, send frame to input queue
            xQueueSend(rx_queue, &msg, 0);
            if (debuggingMode) Serial.write('_');
            return true;
        }
    }
    return false;
}

bool ESP32CAN::sendFrame(CAN_FRAME& txFrame)
{
    twai_message_t __TX_frame;

    __TX_frame.identifier = txFrame.id;
    __TX_frame.data_length_code = txFrame.length;
    __TX_frame.rtr = txFrame.rtr;
    __TX_frame.extd = txFrame.extended;
    for (int i = 0; i < 8; i++) __TX_frame.data[i] = txFrame.data.byte[i];

    //don't wait long if the queue was full. The end user code shouldn't be sending faster
    //than the buffer can empty. Set a bigger TX buffer or delay sending if this is a problem.
    esp_err_t result;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    result = twai_transmit_v2(bus_handle, &__TX_frame, pdMS_TO_TICKS(4));
#else
    result = twai_transmit(&__TX_frame, pdMS_TO_TICKS(4));
#endif
    switch (result)
    {
    case ESP_OK:
        if (debuggingMode) Serial.write('<');
        break;
    case ESP_ERR_TIMEOUT:
        if (debuggingMode) Serial.write('T');
        break;
    case ESP_ERR_INVALID_ARG:
    case ESP_FAIL:
    case ESP_ERR_INVALID_STATE:
    case ESP_ERR_NOT_SUPPORTED:
        if (debuggingMode) Serial.write('!');
        break;
    }
    
    return true;
}

bool ESP32CAN::rx_avail()
{
    if (!rx_queue) return false;
    return uxQueueMessagesWaiting(rx_queue) > 0?true:false;
}

uint16_t ESP32CAN::available()
{
    if (!rx_queue) return 0;
    return uxQueueMessagesWaiting(rx_queue);
}

uint32_t ESP32CAN::get_rx_buff(CAN_FRAME &msg)
{
    CAN_FRAME frame;
    //receive next CAN frame from queue
    if (uxQueueMessagesWaiting(rx_queue)) {
        if(xQueueReceive(rx_queue, &frame, 0) == pdTRUE)
        {
            msg = frame; //do a copy in the case that the receive worked
            return true;
        }
        else
            return false;
    }
    return false; //otherwise we leave the msg variable alone and just return false
}

