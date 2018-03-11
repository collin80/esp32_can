/*
  MCP2515.h - Library for Microchip MCP2515 CAN Controller
  
  Author: David Harding
  Maintainer: RechargeCar Inc (http://rechargecar.com)
  Further Modification: Collin Kidder
  
  Created: 11/08/2010
  
  For further information see:
  
  http://ww1.microchip.com/downloads/en/DeviceDoc/21801e.pdf
  http://en.wikipedia.org/wiki/CAN_bus


  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef MCP2515_h
#define MCP2515_h

#include "Arduino.h"
#include "mcp2515_defs.h"
#include "can_common.h"

//#define DEBUG_SETUP

class MCP2515 : public CAN_COMMON
{
  public:
	// Constructor defining which pins to use for CS and INT
    MCP2515(uint8_t CS_Pin, uint8_t INT_Pin);
	
	// Overloaded initialization function
	int Init(uint32_t baud, uint8_t freq);
	int Init(uint32_t baud, uint8_t freq, uint8_t sjw);

    //block of functions which must be overriden from CAN_COMMON to implement functionality for this hardware
	int _setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended);
    int _setFilter(uint32_t id, uint32_t mask, bool extended);
	uint32_t init(uint32_t ul_baudrate);
    uint32_t beginAutoSpeed();
    uint32_t set_baudrate(uint32_t ul_baudrate);
    void setListenOnlyMode(bool state);
	void enable();
	void disable();
	bool sendFrame(CAN_FRAME& txFrame);
	bool rx_avail();
	uint16_t available(); //like rx_avail but returns the number of waiting frames
	uint32_t get_rx_buff(CAN_FRAME &msg);
	
	// Basic MCP2515 SPI Command Set
    void Reset();
    byte Read(uint8_t address);
    void Read(uint8_t address, uint8_t data[], uint8_t bytes);
	CAN_FRAME ReadBuffer(uint8_t buffer);
	void Write(uint8_t address, uint8_t data);
	void Write(uint8_t address, uint8_t data[], uint8_t bytes);
	void LoadBuffer(uint8_t buffer, CAN_FRAME *message);
	void SendBuffer(uint8_t buffers);
	uint8_t Status();
	uint8_t RXStatus();
	void BitModify(uint8_t address, uint8_t mask, uint8_t data);

	// Extra functions
	bool Interrupt(); // Expose state of INT pin
	bool Mode(uint8_t mode); // Returns TRUE if mode change successful
	void setINTPin(uint8_t pin);
	void setCSPin(uint8_t pin);
	void EnqueueRX(CAN_FRAME& newFrame);
	void EnqueueTX(CAN_FRAME& newFrame);
	bool GetRXFrame(CAN_FRAME &frame);
	void SetRXFilter(uint8_t filter, uint32_t FilterValue, bool ext);
	void SetRXMask(uint8_t mask, uint32_t MaskValue);
    void GetRXFilter(uint8_t filter, uint32_t &filterVal, boolean &isExtended);
    void GetRXMask(uint8_t mask, uint32_t &filterVal);

	void InitFilters(bool permissive);
	void intHandler();
	void InitBuffers();
  private:
	bool _init(uint32_t baud, uint8_t freq, uint8_t sjw, bool autoBaud);
    void handleFrameDispatch(CAN_FRAME *frame, int filterHit);
    // Pin variables
	uint8_t _CS;
	uint8_t _INT;
	volatile uint16_t savedBaud;
	volatile uint8_t savedFreq;
	volatile uint8_t running; //1 if out of init code, 0 if still trying to initialize (auto baud detecting)
    // Definitions for software buffers
	volatile CAN_FRAME rx_frames[8];
	volatile CAN_FRAME tx_frames[8];
	volatile uint8_t rx_frame_read_pos, rx_frame_write_pos;
	volatile uint8_t tx_frame_read_pos, tx_frame_write_pos;
	//void (*cbCANFrame[7])(CAN_FRAME *); //6 filters plus an optional catch all
};

#endif
