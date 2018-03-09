//Demo of the second can interface Can1 which uses the MCP2517FD module

#include "esp32_can.h"

void setup() {
	Serial.begin(115200);
	
	Serial.println("Initializing ...");
	
	// Initialize MCP2517FD CAN controller at the specified speed
	if(CAN1.begin(500000))
	{
		Serial.println("MCP2517FD Init OK ...");
	} else {
		Serial.println("MCP2517FD Init Failed ...");
	}

	CAN1.watchFor(); //allow anything through

	Serial.println("Ready ...!");
  CAN_FRAME txFrame;
  txFrame.rtr = 0;
  txFrame.id = 0x123;
  txFrame.extended = false;
  txFrame.length = 4;
  txFrame.data.byte[0] = 0x10;
  txFrame.data.byte[1] = 0x1A;
  txFrame.data.byte[2] = 0xFF;
  txFrame.data.byte[3] = 0x5D;
  CAN1.sendFrame(txFrame);
}

byte i=0;

// CAN message frame
CAN_FRAME message;

void loop() {
	if (CAN1.read(message)) {
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
			Serial.print(message.data.byte[i],HEX);
			Serial.print(" ");
		}
		Serial.println();
		Serial.println();

		// Send out a return message for each one received
		// Simply increment message id and data bytes to show proper transmission
		// Note: this will double the traffic on the network (provided it passes the filter above)
		message.id++;
		for(i = 0;i < message.length; i++) {
			message.data.byte[i]++;
		}
		CAN1.sendFrame(message);
	}
}

