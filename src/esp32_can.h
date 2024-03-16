#include "esp32_can_builtin.h"

// add the following define into your code if you have an MCP2517FD or MCP2515 connected to the SPI bus
// #define HAS_EXTERNAL_CAN_CONTROLLER

#if defined(HAS_EXTERNAL_CAN_CONTROLLER)
#include "mcp2517fd.h"  //uncomment if you've got a mcp2517fd attached to spi
//#include "mcp2515.h" //uncomment if you've got a MCP2515 attached to SPI
#endif

extern ESP32CAN CAN0;
//Select which external chip you've got connected

#if SOC_TWAI_CONTROLLER_NUM == 2 and ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
extern ESP32CAN CAN1;

#if defined(HAS_EXTERNAL_CAN_CONTROLLER)
extern MCP2517FD CAN2;
// extern MCP2515 CAN2;
#endif

#elif defined (HAS_EXTERNAL_CAN_CONTROLLER)
extern MCP2517FD CAN1;
//extern MCP2515 CAN1;
#endif

extern volatile uint32_t biIntsCounter;
extern volatile uint32_t biReadFrames;

#define Can0 CAN0

#if (SOC_TWAI_CONTROLLER_NUM == 2 and ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)) or defined (HAS_EXTERNAL_CAN_CONTROLLER)
#define Can1 CAN1
#endif

#if (SOC_TWAI_CONTROLLER_NUM == 2 and ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)) and defined (HAS_EXTERNAL_CAN_CONTROLLER)
#define Can2 CAN2
#endif