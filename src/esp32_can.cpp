#include "esp32_can.h"

//The objects in here are defined weak now. If you're using an EVTV board you need to do nothing.
//If you're using something else that uses different pins then just go ahead and redefine 
//the objects with whichever pins you need. 

//Set these to the proper pin numbers for you board. Set by default to correct for EVTV ESP32-Due
             //rxpin       txpin
ESP32CAN CAN0(GPIO_NUM_16, GPIO_NUM_17) __attribute__((weak));

//Select and uncomment the proper module you've got connected via SPI
            //CS, INT
MCP2517FD CAN1(5, 27) __attribute__((weak));
//MCP2515 CAN1(5, 27) __attribute__((weak));