/*
  ESP32_CAN.cpp - Library for ESP32 built-in CAN module
  
  Author: Collin Kidder
  
  Created: 31/1/18
*/

#include "Arduino.h"
#include "esp32_can_builtin.h"

CAN_device_t CAN_cfg = {
    CAN_SPEED_500KBPS,
    GPIO_NUM_17,
    GPIO_NUM_16,
    NULL,
    NULL
};

QueueHandle_t callbackQueue;
extern QueueHandle_t lowLevelRXQueue;

extern volatile uint32_t needReset;

ESP32CAN::ESP32CAN(gpio_num_t rxPin, gpio_num_t txPin) : CAN_COMMON(32)
{
    CAN_cfg.rx_pin_id = rxPin;
    CAN_cfg.tx_pin_id = txPin;
    cyclesSinceTraffic = 0;
    rxBufferSize = BI_RX_BUFFER_SIZE; //set defaults
    txBufferSize = BI_TX_BUFFER_SIZE;
}

void ESP32CAN::setCANPins(gpio_num_t rxPin, gpio_num_t txPin)
{
    CAN_cfg.rx_pin_id = rxPin;
    CAN_cfg.tx_pin_id = txPin;
}

void CAN_WatchDog_Builtin( void *pvParameters )
{
    ESP32CAN* espCan = (ESP32CAN*)pvParameters;
    const TickType_t xDelay = 200 / portTICK_PERIOD_MS;

     for(;;)
     {
        vTaskDelay( xDelay );
        espCan->cyclesSinceTraffic++;
        if (needReset)
        {
            espCan->cyclesSinceTraffic = 0;
            needReset = 0;
            if (CAN_cfg.speed > 0 && CAN_cfg.speed <= 1000000ul && espCan->initializedResources == true) 
            {
                if (espCan->debuggingMode)   Serial.println("Builtin CAN Forced Reset!");
                CAN_stop();
                CAN_init();
            }
        }
     }
}

void task_LowLevelRX(void *pvParameters)
{
    ESP32CAN* espCan = (ESP32CAN*)pvParameters;
    CAN_frame_t rxFrame;
    
    while (1)
    {
        //receive next CAN frame from queue and fire off the callback
        if(xQueueReceive(lowLevelRXQueue, &rxFrame, portMAX_DELAY)==pdTRUE)
        {
            espCan->processFrame(rxFrame);
        }
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
void task_CAN( void *pvParameters )
{
    ESP32CAN* espCan = (ESP32CAN*)pvParameters;
    CAN_FRAME rxFrame;

    while (1)
    {
        //receive next CAN frame from queue and fire off the callback
        if(xQueueReceive(callbackQueue, &rxFrame, portMAX_DELAY)==pdTRUE)
        {
            espCan->sendCallback(&rxFrame);
        }
    }
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

ESP32CAN::ESP32CAN() : CAN_COMMON(BI_NUM_FILTERS) 
{
    for (int i = 0; i < BI_NUM_FILTERS; i++)
    {
        filters[i].id = 0;
        filters[i].mask = 0;
        filters[i].extended = false;
        filters[i].configured = false;
    }
    initializedResources = false;
    cyclesSinceTraffic = 0;
}

void ESP32CAN::setRXBufferSize(int newSize)
{
    rxBufferSize = newSize;
}

void ESP32CAN::setTXBufferSize(int newSize)
{
    txBufferSize = newSize;
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

    if (!initializedResources) {
                                 //Queue size, item size
        CAN_cfg.rx_queue = xQueueCreate(rxBufferSize,sizeof(CAN_frame_t));
        CAN_cfg.tx_queue = xQueueCreate(txBufferSize,sizeof(CAN_frame_t));
        callbackQueue = xQueueCreate(16, sizeof(CAN_FRAME));
        CAN_initRXQueue();
                  //func        desc    stack, params, priority, handle to task
        xTaskCreate(&task_CAN, "CAN_RX", 8192, this, 15, NULL);
        xTaskCreatePinnedToCore(&task_LowLevelRX, "CAN_LORX", 4096, this, 19, NULL, 1);
        xTaskCreatePinnedToCore(&CAN_WatchDog_Builtin, "CAN_WD_BI", 2048, this, 10, NULL, 1);
        initializedResources = true;
    }
}

uint32_t ESP32CAN::init(uint32_t ul_baudrate)
{
    _init();
    CAN_cfg.speed = (CAN_speed_t)(ul_baudrate / 1000);
    needReset = 0;
    return CAN_init();
}

uint32_t ESP32CAN::beginAutoSpeed()
{
    const uint32_t bauds[7] = {1000,500,250,125,800,80,33}; //list of speeds to try, scaled down by 1000x
    bool oldLOM = CAN_GetListenOnlyMode();

    _init();

    CAN_stop();
    CAN_SetListenOnly(true);
    for (int i = 0; i < 7; i++)
    {
        CAN_cfg.speed = (CAN_speed_t)bauds[i];
        CAN_stop(); //stop hardware so we can reconfigure it
        needReset = 0;
        Serial.print("Trying Speed ");
        Serial.print(bauds[i] * 1000);
        CAN_init(); //set it up
        delay(600); //wait a while
        if (cyclesSinceTraffic < 2) //only would happen if there had been traffic
        {
            CAN_stop();
            CAN_SetListenOnly(oldLOM);
            CAN_init();
            Serial.println(" SUCCESS!");
            return bauds[i] * 1000;
        }
        else
        {
            Serial.println(" FAILED.");
        }
        
    }
    Serial.println("None of the tested CAN speeds worked!");
    CAN_stop();
    return 0;
}

uint32_t ESP32CAN::set_baudrate(uint32_t ul_baudrate)
{
    CAN_stop();
    CAN_cfg.speed = (CAN_speed_t)(ul_baudrate / 1000);
    needReset = 0;
    return CAN_init();
}

void ESP32CAN::setListenOnlyMode(bool state)
{
    CAN_SetListenOnly(state);
}

void ESP32CAN::enable()
{
    CAN_stop();
    CAN_init();
    needReset = 0;
}

void ESP32CAN::disable()
{
    CAN_stop();
    needReset = 0;
}

//This function is too big to be running in interrupt context. Refactored so it doesn't.
bool ESP32CAN::processFrame(CAN_frame_t &frame)
{
    CANListener *thisListener;
    CAN_FRAME msg;

    cyclesSinceTraffic = 0; //reset counter to show that we are receiving traffic

    msg.id = frame.MsgID;
    msg.length = frame.FIR.B.DLC;
    msg.rtr = frame.FIR.B.RTR;
    msg.extended = frame.FIR.B.FF;
    for (int i = 0; i < 8; i++) msg.data.byte[i] = frame.data.u8[i];
    
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
            xQueueSend(CAN_cfg.rx_queue, &frame, 0);
            if (debuggingMode) Serial.write('_');
            return true;
        }
    }
    return false;
}

bool ESP32CAN::sendFrame(CAN_FRAME& txFrame)
{
    CAN_frame_t __TX_frame;

    __TX_frame.MsgID = txFrame.id;
    __TX_frame.FIR.B.DLC = txFrame.length;
    __TX_frame.FIR.B.RTR = (CAN_RTR_t)txFrame.rtr;
    __TX_frame.FIR.B.FF = (CAN_frame_format_t)txFrame.extended;
    for (int i = 0; i < 8; i++) __TX_frame.data.u8[i] = txFrame.data.byte[i];

    if (CAN_TX_IsBusy()) //hardware already sending, queue for sending when possible
    {
        xQueueSend(CAN_cfg.tx_queue,&__TX_frame,0);
        if (debuggingMode) Serial.write('<');
    }
    else //hardware is free, send immediately
    {
        CAN_write_frame(&__TX_frame);
        if (debuggingMode) Serial.write('>');
    }
    return true;
}

bool ESP32CAN::rx_avail()
{
    if (!CAN_cfg.rx_queue) return false;
    return uxQueueMessagesWaiting(CAN_cfg.rx_queue) > 0?true:false;
}

uint16_t ESP32CAN::available()
{
    if (!CAN_cfg.rx_queue) return 0;
    return uxQueueMessagesWaiting(CAN_cfg.rx_queue);
}

uint32_t ESP32CAN::get_rx_buff(CAN_FRAME &msg)
{
    CAN_frame_t __RX_frame;
    //receive next CAN frame from queue
    if(xQueueReceive(CAN_cfg.rx_queue,&__RX_frame, 0)==pdTRUE)
    {
        msg.id = __RX_frame.MsgID;
        msg.length = __RX_frame.FIR.B.DLC;
        msg.rtr = __RX_frame.FIR.B.RTR;
        msg.extended = __RX_frame.FIR.B.FF;
        for (int i = 0; i < 8; i++) msg.data.byte[i] = __RX_frame.data.u8[i];
        return true;
    }
    return false;
}

