#include "esp32_can.h"

//Set these to the proper pin numbers for you board. Set by default to correct for EVTV ESP32-Due
             //rxpin       txpin
ESP32CAN CAN0(GPIO_NUM_16, GPIO_NUM_17);

//Select and uncomment the proper module you've got connected via SPI
            //CS, INT
//MCP2517FD CAN1(5, 27);
MCP2515 CAN1(5, 27);