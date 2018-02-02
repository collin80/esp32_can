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

void task_CAN( void *pvParameters ){
    //frame buffer
    CAN_frame_t __RX_frame;

    while (1){
        //receive next CAN frame from queue
        //if(xQueueReceive(CAN_cfg.rx_queue,&__RX_frame, 0)==pdTRUE){

        //}
    }
}

ESP32CAN::ESP32CAN() : CAN_COMMON(6) 
{

}

int ESP32CAN::_setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended)
{

}

int ESP32CAN::_setFilter(uint32_t id, uint32_t mask, bool extended)
{

}

uint32_t ESP32CAN::init(uint32_t ul_baudrate)
{
                                 //Queue size, item size
    CAN_cfg.rx_queue = xQueueCreate(32,sizeof(CAN_frame_t));
    CAN_cfg.tx_queue = xQueueCreate(16,sizeof(CAN_frame_t));
              //func          desc     stack, params, priority, handle to task
    xTaskCreate(&task_CAN, "CAN_RX", 2048, this, 5, NULL);
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

/*
void MCP2515::intHandler(void) {
    CAN_FRAME message;
    uint32_t ctrlVal;
    // determine which interrupt flags have been set
    uint8_t interruptFlags = Read(CANINTF);
    //Now, acknowledge the interrupts by clearing the intf bits
    Write(CANINTF, 0); 	
    
    if(interruptFlags & RX0IF) {
      // read from RX buffer 0
		message = ReadBuffer(RXB0);
        ctrlVal = Read(RXB0CTRL);
        handleFrameDispatch(&message, ctrlVal & 1);
    }
    if(interruptFlags & RX1IF) {
        // read from RX buffer 1
        message = ReadBuffer(RXB1);
        ctrlVal = Read(RXB1CTRL);
        handleFrameDispatch(&message, ctrlVal & 7);
    }
    if(interruptFlags & TX0IF) {
		// TX buffer 0 sent
       if (tx_frame_read_pos != tx_frame_write_pos) {
			LoadBuffer(TXB0, (CAN_FRAME *)&tx_frames[tx_frame_read_pos]);
		   	SendBuffer(TXB0);
			tx_frame_read_pos = (tx_frame_read_pos + 1) % 8;
	   }
    }
    if(interruptFlags & TX1IF) {
		// TX buffer 1 sent
	  if (tx_frame_read_pos != tx_frame_write_pos) {
		  LoadBuffer(TXB1, (CAN_FRAME *)&tx_frames[tx_frame_read_pos]);
		  SendBuffer(TXB1);
		  tx_frame_read_pos = (tx_frame_read_pos + 1) % 8;
	  }
    }
    if(interruptFlags & TX2IF) {
		// TX buffer 2 sent
		if (tx_frame_read_pos != tx_frame_write_pos) {
			LoadBuffer(TXB2, (CAN_FRAME *)&tx_frames[tx_frame_read_pos]);
			SendBuffer(TXB2);
			tx_frame_read_pos = (tx_frame_read_pos + 1) % 8;
		}
    }
    if(interruptFlags & ERRIF) {
      if (running == 1) { //if there was an error and we had been initialized then try to fix it by reinitializing
		  running = 0;
		  //InitBuffers();
		  //Init(savedBaud, savedFreq);
	  }
    }
    if(interruptFlags & MERRF) {
      // error handling code
      // if TXBnCTRL.TXERR set then transmission error
      // if message is lost TXBnCTRL.MLOA will be set
      if (running == 1) { //if there was an error and we had been initialized then try to fix it by reinitializing
		running = 0;
		//InitBuffers();
		//Init(savedBaud, savedFreq);
	  }	  
    }
}

void MCP2515::handleFrameDispatch(CAN_FRAME *frame, int filterHit)
{
    CANListener *thisListener;

    //First, try to send a callback. If no callback registered then buffer the frame.
    if (cbCANFrame[filterHit]) 
	{
		(*cbCANFrame[filterHit])(frame);
        return;
	}
	else if (cbGeneral) 
	{
		(*cbGeneral)(frame);
        return;
	}
	else
	{
		for (int listenerPos = 0; listenerPos < SIZE_LISTENERS; listenerPos++)
		{
			thisListener = listener[listenerPos];
			if (thisListener != NULL)
			{
				if (thisListener->isCallbackActive(filterHit)) 
				{
					thisListener->gotFrame(frame, filterHit);
                    return;
				}
				else if (thisListener->isCallbackActive(numFilters)) //global catch-all 
				{
					thisListener->gotFrame(frame, -1);
                    return;
				}
			}
		}
	}
	//if none of the callback types caught this frame then queue it in the buffer
    EnqueueRX(*frame);
}
*/