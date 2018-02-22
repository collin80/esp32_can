/*
  ESP32_CAN.cpp - Library for ESP32 built-in CAN module
  
  Author: Collin Kidder
  
  Created: 31/1/18
*/

#include "Arduino.h"
#include "ESP32_CAN.h"

CAN_device_t CAN_cfg = {
    CAN_SPEED_500KBPS,
    GPIO_NUM_17,
    GPIO_NUM_16,
    NULL,
    NULL
};

ESP32CAN::ESP32CAN() : CAN_COMMON(NUM_FILTERS) 
{
    for (int i = 0; i < NUM_FILTERS; i++)
    {
        filters[i].id = 0;
        filters[i].mask = 0;
        filters[i].extended = false;
        filters[i].configured = false;
    }
}

int ESP32CAN::_setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended)
{
    if (mailbox < NUM_FILTERS)
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
    for (int i = 0; i < NUM_FILTERS; i++)
    {
        if (!filters[i].configured) 
        {
            _setFilterSpecific(i, id, mask, extended);
            return i;
        }
    }
    return -1;
}

uint32_t ESP32CAN::init(uint32_t ul_baudrate)
{
    for (int i = 0; i < NUM_FILTERS; i++)
    {
        filters[i].id = 0;
        filters[i].mask = 0;
        filters[i].extended = false;
        filters[i].configured = false;
    }
                                 //Queue size, item size
    CAN_cfg.rx_queue = xQueueCreate(32,sizeof(CAN_frame_t));
    CAN_cfg.tx_queue = xQueueCreate(16,sizeof(CAN_frame_t));
    CAN_cfg.speed = (CAN_speed_t)(ul_baudrate / 1000);
    CAN_init();
}

uint32_t ESP32CAN::beginAutoSpeed()
{

}

uint32_t ESP32CAN::set_baudrate(uint32_t ul_baudrate)
{
    CAN_stop();
    CAN_cfg.speed = (CAN_speed_t)(ul_baudrate / 1000);
    CAN_init();
}

void ESP32CAN::setListenOnlyMode(bool state)
{
    CAN_SetListenOnly(state);
}

void ESP32CAN::enable()
{
    CAN_stop();
    CAN_init();
}

void ESP32CAN::disable()
{
    CAN_stop();
}

bool ESP32CAN::processFrame(CAN_frame_t &frame)
{
    CANListener *thisListener;
    CAN_FRAME msg;

    msg.id = frame.MsgID;
    msg.length = frame.FIR.B.DLC;
    msg.rtr = frame.FIR.B.RTR;
    msg.extended = frame.FIR.B.FF;
    for (int i = 0; i < 8; i++) msg.data.byte[i] = frame.data.u8[i];

    for (int i = 0; i < NUM_FILTERS; i++)
    {
        if (!filters[i].configured) continue;
        if ((frame.MsgID & filters[i].mask) == filters[i].id && (filters[i].extended == frame.FIR.B.FF))
        {
            //frame is accepted, lets see if it matches a mailbox callback
            if (cbCANFrame[i])
            {
                (*cbCANFrame[i])(&msg);
                return true;
            }
            else if (cbGeneral)
            {
        		(*cbGeneral)(&msg);
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
					        thisListener->gotFrame(&msg, i);
                            return true;
				        }
				        else if (thisListener->isCallbackActive(numFilters)) //global catch-all 
				        {
					        thisListener->gotFrame(&msg, -1);
                            return true;
				        }
                    }
                }
            }
            
            //otherwise, send frame to input queue
            xQueueSendFromISR(CAN_cfg.rx_queue, &frame, 0);
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
    }
    else //hardware is free, send immediately
    {
        CAN_write_frame(&__TX_frame);
    }
}

bool ESP32CAN::rx_avail()
{
    return uxQueueMessagesWaiting(CAN_cfg.rx_queue) > 0?true:false;
}

uint16_t ESP32CAN::available()
{
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

ESP32CAN CAN;
