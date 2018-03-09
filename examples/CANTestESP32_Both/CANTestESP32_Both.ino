/*
  Demo of using both CAN ports on the EVTV ESP32 board. Shows an example of many techniques in the library:
	1. Initializing both
	2. Set Can0 to forward any traffic where the ID starts with 0x1?? where ?? could be any number 0 - FF
	3. Set Can1 to forward any traffic where the ID starts with 0x23? where ? could be any number 0 - F
	4. Do both of the above via a C style callback
	5. Display any other frames over the serial port
*/

#include "esp32_can.h"

void handleCAN0CB(CAN_FRAME *frame)
{
	CAN1.sendFrame(*frame);
}

void handleCAN1CB(CAN_FRAME *frame)
{
	CAN0.sendFrame(*frame);
}

void printFrame(CAN_FRAME &frame)
{
	// Print message
	Serial.print("ID: ");
	Serial.println(frame.id,HEX);
	Serial.print("Ext: ");
	if(frame.extended) {
		Serial.println("Y");
	} else {
		Serial.println("N");
	}
	Serial.print("Len: ");
	Serial.println(frame.length,DEC);
	for(int i = 0;i < frame.length; i++) {
		Serial.print(frame.data.uint8[i],HEX);
		Serial.print(" ");
	}
	Serial.println();
}

void setup() {
	Serial.begin(115200);
	
	Serial.println("Initializing ...");

	// Initialize builtin CAN controller at the specified speed
	if(CAN0.begin(500000))
	{
		Serial.println("Builtin CAN Init OK ...");
	} else {
		Serial.println("BuiltIn CAN Init Failed ...");
	}
	
	// Initialize MCP2517FD CAN controller at the specified speed
	if(CAN1.begin(500000))
	{
		Serial.println("MCP2517FD Init OK ...");
	} else {
		Serial.println("MCP2517FD Init Failed ...");
	}

	CAN0.setRXFilter(0, 0x100, 0x700, false);
	CAN0.watchFor(); //allow everything else through
	CAN0.setCallback(0, handleCAN0CB);

	CAN1.setRXFilter(0, 0x230, 0x7F0, false);
	CAN1.watchFor(); //allow everything else through
	CAN1.setCallback(0, handleCAN1CB);

	Serial.println("Ready ...!");
}


void loop() {
	CAN_FRAME message;

	if (CAN0.read(message)) {
		printFrame(message);
	}
	if (CAN1.read(message)) {
		printFrame(message);
	}
}

