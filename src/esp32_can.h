#include "esp32_can_builtin.h"

extern ESP32CAN CAN0;
//Select which external chip you've got connected

#if SOC_TWAI_CONTROLLER_NUM == 2 and ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    extern ESP32CAN CAN1;
#endif

extern volatile uint32_t biIntsCounter;
extern volatile uint32_t biReadFrames;

#define Can0 CAN0

#if (SOC_TWAI_CONTROLLER_NUM == 2 and ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)) or defined (USES_MCP2517FD) or defined (USES_MCP2515)
#define Can1 CAN1
#endif
