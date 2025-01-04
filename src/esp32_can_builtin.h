/*
  MCP2515.h - Library for Microchip MCP2515 CAN Controller
  
  Author: David Harding
  Maintainer: RechargeCar Inc (http://rechargecar.com)
  Further Modification: Collin Kidder
  
  Created: 11/08/2010
  
  For further information see:
  
  http://ww1.microchip.com/downloads/en/DeviceDoc/21801e.pdf
  http://en.wikipedia.org/wiki/CAN_bus


  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __ESP32_CAN__
#define __ESP32_CAN__

#include "Arduino.h"
#include <can_common.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_system.h"
#include "esp_adc_cal.h"
#include "driver/twai.h"
#include <string.h>
#include <sstream>

//#define DEBUG_SETUP
#define BI_NUM_FILTERS 32

#define BI_RX_BUFFER_SIZE	64
#define BI_TX_BUFFER_SIZE  16

typedef struct
{
  uint32_t mask;
  uint32_t id;
  bool extended;
  bool configured;
} ESP32_FILTER;

typedef struct
{
    twai_timing_config_t cfg;
    uint32_t speed;
} VALID_TIMING;

class ESP32CAN : public CAN_COMMON
{
public:
  ESP32CAN(gpio_num_t rxPin, gpio_num_t txPin, uint8_t busNumber = 0);
  ESP32CAN();

  //block of functions which must be overriden from CAN_COMMON to implement functionality for this hardware
  int _setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended);
  int _setFilter(uint32_t id, uint32_t mask, bool extended);
  void _init();
  uint32_t init(uint32_t ul_baudrate);
  uint32_t beginAutoSpeed();
  uint32_t set_baudrate(uint32_t ul_baudrate);
  void setListenOnlyMode(bool state);
  void setNoACKMode(bool state);
  void enable();
  void disable();
  bool sendFrame(CAN_FRAME& txFrame);
  bool rx_avail();
  void setTXBufferSize(int newSize);
  void setRXBufferSize(int newSize);
  uint16_t available(); //like rx_avail but returns the number of waiting frames
  uint32_t get_rx_buff(CAN_FRAME &msg);
  bool processFrame(twai_message_t &frame);
  void sendCallback(CAN_FRAME *frame);

  void setCANPins(gpio_num_t rxPin, gpio_num_t txPin);

  static void CAN_WatchDog_Builtin( void *pvParameters );

  #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    twai_handle_t bus_handle;
  #endif

protected:
  bool readyForTraffic;
  int cyclesSinceTraffic;
                                                                      //tx,         rx,           mode
  twai_general_config_t twai_general_cfg = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_17, GPIO_NUM_16, TWAI_MODE_NORMAL);
  twai_timing_config_t twai_speed_cfg = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t twai_filters_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  QueueHandle_t callbackQueue;
  QueueHandle_t rx_queue;

  TaskHandle_t CAN_WatchDog_Builtin_handler = NULL;
  TaskHandle_t task_CAN_handler = NULL;
  TaskHandle_t task_LowLevelRX_handler = NULL;

private:
  // Pin variables
  ESP32_FILTER filters[BI_NUM_FILTERS];
  int rxBufferSize;

  static void task_CAN(void *pvParameters);
  static void task_LowLevelRX(void *pvParameters);
};

extern QueueHandle_t callbackQueue;

#endif
