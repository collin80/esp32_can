esp32_can
==========

A new, unified library all inclusive of code needed to make it operate. Implements
a CAN driver for the built-in CAN hardware on an ESP32. Also implements a driver
for the MCP2517FD SPI connected CAN module. The builtin CAN is called CAN0,
the MCP2517FD is called CAN1. This library is specifically meant to be used with 
the EVTV ESP32-Due board. However, with small modifications either driver found
within this library could be used on other boards.

This library requires the can_common library. That library is a common base that 
other libraries can be built off of to allow a more universal API for CAN.

The needed can_common library is found here: https://github.com/collin80/can_common
