//Demo of the second can interface Can1 which uses the MCP2517FD module - Now in CANFD mode

#include "esp32_can.h"

void setup() {
	Serial.begin(115200);
	
	Serial.println("Initializing ...");
	
	// Initialize MCP2517FD CAN controller at the specified speed
	if(CAN1.beginFD(500000, 4000000))
	{
		Serial.println("MCP2517FD Init OK ...");
	} else {
		Serial.println("MCP2517FD Init Failed ...");
	}

  CAN1.watchFor(); //allow anything through
  
  //CAN1.setRXFilter(0, 0x100, 0x700, false);
  //CAN1.setRXFilter(1, 0x200, 0x700, false);
  /*
  CAN1.setRXFilter(2, 0x300, 0x700, false);
  CAN1.setRXFilter(3, 0x400, 0x700, false);
  CAN1.setRXFilter(4, 0x500, 0x700, false);
  CAN1.setRXFilter(5, 0x600, 0x700, false);
  */

	Serial.println("Ready ...!");
  CAN_FRAME_FD txFrame;
  txFrame.rrs = 0;
  txFrame.id = 0x123;
  txFrame.extended = false;
  txFrame.priority = 15;
  txFrame.fdMode = 1; //Use full FD mode for this frame
  txFrame.length = 12;
  txFrame.data.uint32[0] = 0x34AA76BB;
  txFrame.data.uint32[1] = 0xFD7A41C9;
  txFrame.data.uint32[2] = 0x01F1AAD7;
  CAN1.sendFrameFD(txFrame);
}

byte i=0;

// CAN message frame
CAN_FRAME_FD message;

void loop() {
	if (CAN1.readFD(message)) {
		// Print message
		Serial.print("ID: ");
		Serial.println(message.id,HEX);
		Serial.print("Extended: ");
		if(message.extended) {
			Serial.println("Yes");
		} else {
			Serial.println("No");
		}
		Serial.print("Length: ");
		Serial.println(message.length,DEC);
		for(i = 0;i < message.length; i++) {
			Serial.print(message.data.uint8[i],HEX);
			Serial.print(" ");
		}
		Serial.println();
		Serial.println();

		// Send out a return message for each one received
		// Simply increment message id and data bytes to show proper transmission
		// Note: this will double the traffic on the network (provided it passes the filter above)
		message.id++;
		for(i = 0;i < message.length; i++) {
			message.data.uint8[i]++;
		}
		CAN1.sendFrameFD(message);
	}
}

