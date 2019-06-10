esp32_can
==========

A new, unified library all inclusive of code needed to make it operate. Implements a CAN driver for the built-in CAN hardware on an ESP32. Also implements a driver for the MCP2517FD SPI connected CAN module. 

This library is designed to be used with the EVTV ESP32-Due board.

## Example usage:
```cpp
/*
ESP32's native CAN controller (there's only one)
*/
ESP32CAN CAN0(GPIO_NUM_16, GPIO_NUM_17);  // (RX, TX)

/*
SPI-controlled MCP2517FD CAN controller
*/
MCP2517FD CAN1(5, 27);                    // (SPI_CS, INTERRUPT)
MCP2517FD CAN2(32, 36);                   // (SPI_CS, INTERRUPT)
```
Thanks to [can_common](https://github.com/collin80/can_common), multiple types of CAN peripheral are controllable with the same API.
```cpp
void send_frames()
{
  CAN_FRAME txFrame;
  txFrame.rtr = 0;
  txFrame.id = 0x123;
  txFrame.extended = false;
  txFrame.length = 4;
  txFrame.data.int8[0] = 0x10;
  txFrame.data.int8[1] = 0x1A;
  txFrame.data.int8[2] = 0xFF;
  txFrame.data.int8[3] = 0x5D;
  txFrame.data.int8[7] = 0xFF;

  CAN0.sendFrame(txFrame);
  CAN1.sendFrame(txFrame);
  CAN2.sendFrame(txFrame);
}
```

## Try the [examples](examples)
