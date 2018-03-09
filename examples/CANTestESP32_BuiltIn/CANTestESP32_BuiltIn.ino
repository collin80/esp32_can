#include <esp32_can.h>

void printFrame(CAN_FRAME *message)
{
  Serial.print(message->id, HEX);
  if (message->extended) Serial.print(" X ");
  else Serial.print(" S ");   
  Serial.print(message->length, DEC);
  for (int i = 0; i < message->length; i++) {
    Serial.print(message->data.byte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void gotHundred(CAN_FRAME *frame)
{
  Serial.print("Got special frame!  ");
  printFrame(frame);
}

void setup() {
  Serial.begin(115200);

  Serial.println("Initializing ...");

  CAN0.begin(500000);

  Serial.println("Ready ...!");
  CAN_FRAME txFrame;
  txFrame.rtr = 0;
  txFrame.id = 0x123;
  txFrame.extended = false;
  txFrame.length = 4;
  txFrame.data.uint8[0] = 0x10;
  txFrame.data.uint8[1] = 0x1A;
  txFrame.data.uint8[2] = 0xFF;
  txFrame.data.uint8[3] = 0x5D;
  CAN0.sendFrame(txFrame);

  CAN0.watchFor(0x100, 0xF00); //setup a special filter
  CAN0.watchFor(); //then let everything else through anyway
  CAN0.setCallback(0, gotHundred); //callback on that first special filter
}

void loop() {
  byte i = 0;
  CAN_FRAME message;
  if (CAN0.read(message)) {

    printFrame(&message);

    // Send out a return message for each one received
    // Simply increment message id and data bytes to show proper transmission
    // Note: this will double the traffic on the network (provided it passes the filter above)
    message.id++;
    for (i = 0; i < message.length; i++) {
      message.data.uint8[i]++;
    }
    //CAN.sendFrame(message);
  }
  //or, just plain send traffic periodically
  /*
    delayMicroseconds(200);
    message.id++;
    message.length = 8;
    for(i=0;i<message.length;i++) {
     message.data.uint8[i]++;
    }
    CAN.sendFrame(message);
  */
}

