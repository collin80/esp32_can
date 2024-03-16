#include "esp32_can_builtin.h"

// #define HAS_EXTERNAL_TWAI_CONTROLLER

#if defined(USES_EXTERNAL_TWAI_CONTROLLER)
#include "mcp2517fd.h"  //uncomment if you've got a mcp2517fd attached to spi
//#include "mcp2515.h" //uncomment if you've got a MCP2515 attached to SPI
#endif

extern ESP32CAN CAN0;
//Select which external chip you've got connected

#if SOC_TWAI_CONTROLLER_NUM == 2
extern ESP32CAN CAN1;

#if defined(USES_EXTERNAL_TWAI_CONTROLLER)
extern MCP2517FD CAN2;
// extern MCP2515 CAN2;
#endif

#elif defined (USES_EXTERNAL_TWAI_CONTROLLER)
extern MCP2517FD CAN1;
//extern MCP2515 CAN1;
#endif

extern volatile uint32_t biIntsCounter;
extern volatile uint32_t biReadFrames;

#define Can0 CAN0

#if (SOC_TWAI_CONTROLLER_NUM == 2) or defined (USES_EXTERNAL_TWAI_CONTROLLER)
#define Can1 CAN1
#endif

#if (SOC_TWAI_CONTROLLER_NUM == 2) and defined (USES_EXTERNAL_TWAI_CONTROLLER)
#define Can2 CAN2
#endif