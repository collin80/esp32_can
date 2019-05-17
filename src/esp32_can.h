#include "esp32_can_builtin.h"
#include "mcp2517fd.h"  //uncomment if you've got a mcp2517fd attached to spi
//#include "mcp2515.h" //uncomment if you've got a MCP2515 attached to SPI

extern ESP32CAN CAN0;
//Select which external chip you've got connected
extern MCP2517FD CAN1;
//extern MCP2515 CAN1;

extern volatile uint32_t biIntsCounter;
extern volatile uint32_t biReadFrames;

#define Can0 CAN0
#define Can1 CAN1