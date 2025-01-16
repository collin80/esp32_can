esp32_can
==========

Implements a CAN driver for the built-in CAN hardware on an ESP32. 
The builtin CAN is called CAN0, and also CAN1 if there is a second CAN port on the ESP32 chip in use.
This library is specifically meant to be used with the EVTV ESP32-Due board. 
However, with small modifications either driver found within this library could be used on other boards.
Primarily, one should check TX and RX pins if using a different board.

This library requires the can_common library. That library is a common base that 
other libraries can be built off of to allow a more universal API for CAN.

The needed can_common library is found here: https://github.com/collin80/can_common

As of version 0.3.1 of this library, MCP2517FD support is no longer included. It
has been moved to its own library. Get the library here:

https://github.com/collin80/esp32_mcp2517fd

then add this header include to your source code:

#include <esp32_mcp2517fd.h>
