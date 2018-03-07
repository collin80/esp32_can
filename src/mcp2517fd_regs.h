/*******************************************************************************
  CAN FD SPI Driver: Register Header File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_canfdspi_register.h

  Summary:
    This header file contains SPI instruction defines, register address defines,
    register structures, and reset values of registers.

  Description:
    This file is used by the API.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 Microchip Technology Inc. and its subsidiaries.  
You may use this software and any derivatives exclusively with Microchip products. 
  
THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  
NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, 
INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, 
AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, 
COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 

IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER 
RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED 
OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWED BY LAW, 
MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE 
WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE TERMS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_CANFDSPI_REGISTER_H
#define _DRV_CANFDSPI_REGISTER_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif
// DOM-IGNORE-END  


// *****************************************************************************
// *****************************************************************************
/* SPI Instruction Set */
#define CMD_RESET			0x00
#define CMD_READ			0x03
#define CMD_READ_CRC        0x0B
#define CMD_WRITE			0x02
#define CMD_WRITE_CRC       0x0A
#define CMD_WRITE_SAFE      0x0C

// *****************************************************************************
// *****************************************************************************
/* Register Addresses */

/* can_fd_ubp */
#define ADDR_CiCON       0x000
#define ADDR_CiNBTCFG    0x004
#define ADDR_CiDBTCFG    0x008
#define ADDR_CiTDC       0x00C

#define ADDR_CiTBC           0x010
#define ADDR_CiTSCON         0x014
#define ADDR_CiVEC           0x018
#define ADDR_CiINT           0x01C
#define ADDR_CiINTFLAG       ADDR_CiINT
#define ADDR_CiINTENABLE     (ADDR_CiINT+2)

#define ADDR_CiRXIF     0x020
#define ADDR_CiTXIF     0x024
#define ADDR_CiRXOVIF   0x028
#define ADDR_CiTXATIF   0x02C

#define ADDR_CiTXREQ    0x030
#define ADDR_CiTREC     0x034
#define ADDR_CiBDIAG0   0x038
#define ADDR_CiBDIAG1   0x03C

#define ADDR_CiTEFCON   0x040
#define ADDR_CiTEFSTA   0x044
#define ADDR_CiTEFUA    0x048
#define ADDR_CiFIFOBA   0x04C

#define ADDR_CiFIFOCON  0x050
#define ADDR_CiFIFOSTA  0x054
#define ADDR_CiFIFOUA   0x058
#define CiFIFO_OFFSET  (3*4)

#define ADDR_CiTXQCON  0x050
#define ADDR_CiTXQSTA  0x054
#define ADDR_CiTXQUA   0x058

#ifdef FIXED_FILTER_ADDRESS
// Up to A1, the filter start address was fixed
#define ADDR_CiFLTCON   0x1D0
#define ADDR_CiFLTOBJ   0x1F0
#define ADDR_CiMASK     0x1F4
#else
// Starting with B0, the filters start right after the FIFO control/status registers
#define ADDR_CiFLTCON   (ADDR_CiFIFOCON+(CiFIFO_OFFSET*CAN_FIFO_TOTAL_CHANNELS))
#define ADDR_CiFLTOBJ   (ADDR_CiFLTCON + CAN_FIFO_TOTAL_CHANNELS)
#define ADDR_CiMASK     (ADDR_CiFLTOBJ+4)
#endif

#define CiFILTER_OFFSET     (2*4)

/* MCP2517 Specific */
#define ADDR_OSC         0xE00
#define ADDR_IOCON       0xE04
#define ADDR_CRC         0xE08
#define ADDR_ECCCON      0xE0C
#define ADDR_ECCSTA      0xE10

/* RAM addresses */
#define cRAM_SIZE       2048
#define cRAMADDR_START  0x400
#define cRAMADDR_END    (cRAMADDR_START+cRAM_SIZE)

// *****************************************************************************
// *****************************************************************************
/* Register Structures */

// *****************************************************************************
//! General 32-bit Register

typedef union _REG_t {
    uint8_t byte[4];
    uint32_t word;
} REG_t;


// *****************************************************************************
// *****************************************************************************
/* can_fd_ubp */

// *****************************************************************************
//! CAN Control Register

typedef union _REG_CiCON {

    struct {
        uint32_t DNetFilterCount : 5;
        uint32_t IsoCrcEnable : 1;
        uint32_t ProtocolExceptionEventDisable : 1;
        uint32_t unimplemented1 : 1;
        uint32_t WakeUpFilterEnable : 1;
        uint32_t WakeUpFilterTime : 2;
        uint32_t unimplemented2 : 1;
        uint32_t BitRateSwitchDisable : 1;
        uint32_t unimplemented3 : 3;
        uint32_t RestrictReTxAttempts : 1;
        uint32_t EsiInGatewayMode : 1;
        uint32_t SystemErrorToListenOnly : 1;
        uint32_t StoreInTEF : 1;
        uint32_t TXQEnable : 1;
        uint32_t OpMode : 3;
        uint32_t RequestOpMode : 3;
        uint32_t AbortAllTx : 1;
        uint32_t TxBandWidthSharing : 4;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiCON;

// *****************************************************************************
//! Nominal Bit Time Configuration Register

typedef union _REG_CiNBTCFG {

    struct {
        uint32_t SJW : 7;
        uint32_t unimplemented1 : 1;
        uint32_t TSEG2 : 7;
        uint32_t unimplemented2 : 1;
        uint32_t TSEG1 : 8;
        uint32_t BRP : 8;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiNBTCFG;

// *****************************************************************************
//! Data Bit Time Configuration Register

typedef union _REG_CiDBTCFG {

    struct {
        uint32_t SJW : 4;
        uint32_t unimplemented1 : 4;
        uint32_t TSEG2 : 4;
        uint32_t unimplemented2 : 4;
        uint32_t TSEG1 : 5;
        uint32_t unimplemented3 : 3;
        uint32_t BRP : 8;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiDBTCFG;

// *****************************************************************************
//! Transmitter Delay Compensation Register

typedef union _REG_CiTDC {

    struct {
        uint32_t TDCValue : 6;
        uint32_t unimplemented1 : 2;
        uint32_t TDCOffset : 7;
        uint32_t unimplemented2 : 1;
        uint32_t TDCMode : 2;
        uint32_t unimplemented3 : 6;
        uint32_t SID11Enable : 1;
        uint32_t EdgeFilterEnable : 1;
        uint32_t unimplemented4 : 6;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiTDC;

// *****************************************************************************
//! Time Stamp Configuration Register

typedef union _REG_CiTSCON {

    struct {
        uint32_t TBCPrescaler : 10;
        uint32_t unimplemented1 : 6;
        uint32_t TBCEnable : 1;
        uint32_t TimeStampEOF : 1;
        uint32_t unimplemented2 : 14;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiTSCON;

// *****************************************************************************
//! Interrupt Vector Register

typedef union _REG_CiVEC {

    struct {
        uint32_t ICODE : 7;
        uint32_t unimplemented1 : 1;
        uint32_t FilterHit : 5;
        uint32_t unimplemented2 : 3;
        uint32_t TXCODE : 7;
        uint32_t unimplemented3 : 1;
        uint32_t RXCODE : 7;
        uint32_t unimplemented4 : 1;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiVEC;

// *****************************************************************************
//! Interrupt Flags

typedef struct _CAN_INT_FLAGS {
    uint32_t TXIF : 1;
    uint32_t RXIF : 1;
    uint32_t TBCIF : 1;
    uint32_t MODIF : 1;
    uint32_t TEFIF : 1;
    uint32_t unimplemented1 : 3;

    uint32_t ECCIF : 1;
    uint32_t SPICRCIF : 1;
    uint32_t TXATIF : 1;
    uint32_t RXOVIF : 1;
    uint32_t SERRIF : 1;
    uint32_t CERRIF : 1;
    uint32_t WAKIF : 1;
    uint32_t IVMIF : 1;
} CAN_INT_FLAGS;

// *****************************************************************************
//! Interrupt Enables

typedef struct _CAN_INT_ENABLES {
    uint32_t TXIE : 1;
    uint32_t RXIE : 1;
    uint32_t TBCIE : 1;
    uint32_t MODIE : 1;
    uint32_t TEFIE : 1;
    uint32_t unimplemented2 : 3;

    uint32_t ECCIE : 1;
    uint32_t SPICRCIE : 1;
    uint32_t TXATIE : 1;
    uint32_t RXOVIE : 1;
    uint32_t SERRIE : 1;
    uint32_t CERRIE : 1;
    uint32_t WAKIE : 1;
    uint32_t IVMIE : 1;
} CAN_INT_ENABLES;

// *****************************************************************************
//! Interrupt Register

typedef union _REG_CiINT {

    struct {
        CAN_INT_FLAGS IF;
        CAN_INT_ENABLES IE;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiINT;

// *****************************************************************************
//! Interrupt Flag Register

typedef union _REG_CiINTFLAG {
    CAN_INT_FLAGS IF;
    uint16_t word;
    uint8_t byte[2];
} REG_CiINTFLAG;

// *****************************************************************************
//! Interrupt Enable Register

typedef union _REG_CiINTENABLE {
    CAN_INT_ENABLES IE;
    uint16_t word;
    uint8_t byte[2];
} REG_CiINTENABLE;

// *****************************************************************************
//! Transmit/Receive Error Count Register

typedef union _REG_CiTREC {

    struct {
        uint32_t RxErrorCount : 8;
        uint32_t TxErrorCount : 8;
        uint32_t ErrorStateWarning : 1;
        uint32_t RxErrorStateWarning : 1;
        uint32_t TxErrorStateWarning : 1;
        uint32_t RxErrorStatePassive : 1;
        uint32_t TxErrorStatePassive : 1;
        uint32_t TxErrorStateBusOff : 1;
        uint32_t unimplemented1 : 10;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiTREC;

// *****************************************************************************
//! Diagnostic Register 0

typedef union _REG_CiBDIAG0 {

    struct {
        uint32_t NRxErrorCount : 8;
        uint32_t NTxErrorCount : 8;
        uint32_t DRxErrorCount : 8;
        uint32_t DTxErrorCount : 8;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiBDIAG0;

// *****************************************************************************
//! Diagnostic Register 1

typedef union _REG_CiBDIAG1 {

    struct {
        uint32_t ErrorFreeMsgCount : 16;

        uint32_t NBit0Error : 1;
        uint32_t NBit1Error : 1;
        uint32_t NAckError : 1;
        uint32_t NFormError : 1;
        uint32_t NStuffError : 1;
        uint32_t NCRCError : 1;
        uint32_t unimplemented1 : 1;
        uint32_t TXBOError : 1;
        uint32_t DBit0Error : 1;
        uint32_t DBit1Error : 1;
        uint32_t DAckError : 1;
        uint32_t DFormError : 1;
        uint32_t DStuffError : 1;
        uint32_t DCRCError : 1;
        uint32_t ESI : 1;
        uint32_t unimplemented2 : 1;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiBDIAG1;

// *****************************************************************************
//! Transmit Event FIFO Control Register

typedef union _REG_CiTEFCON {

    struct {
        uint32_t TEFNEIE : 1;
        uint32_t TEFHFIE : 1;
        uint32_t TEFFULIE : 1;
        uint32_t TEFOVIE : 1;
        uint32_t unimplemented1 : 1;
        uint32_t TimeStampEnable : 1;
        uint32_t unimplemented2 : 2;
        uint32_t UINC : 1;
        uint32_t unimplemented3 : 1;
        uint32_t FRESET : 1;
        uint32_t unimplemented4 : 13;
        uint32_t FifoSize : 5;
        uint32_t unimplemented5 : 3;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiTEFCON;

// *****************************************************************************
//! Transmit Event FIFO Status Register

typedef union _REG_CiTEFSTA {

    struct {
        uint32_t TEFNotEmptyIF : 1;
        uint32_t TEFHalfFullIF : 1;
        uint32_t TEFFullIF : 1;
        uint32_t TEFOVIF : 1;
        uint32_t unimplemented1 : 28;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiTEFSTA;

// *****************************************************************************
//! Transmit Queue Control Register

typedef union _REG_CiTXQCON {

    struct {
        uint32_t TxNotFullIE : 1;
        uint32_t unimplemented1 : 1;
        uint32_t TxEmptyIE : 1;
        uint32_t unimplemented2 : 1;
        uint32_t TxAttemptIE : 1;
        uint32_t unimplemented3 : 2;
        uint32_t TxEnable : 1;
        uint32_t UINC : 1;
        uint32_t TxRequest : 1;
        uint32_t FRESET : 1;
        uint32_t unimplemented4 : 5;
        uint32_t TxPriority : 5;
        uint32_t TxAttempts : 2;
        uint32_t unimplemented5 : 1;
        uint32_t FifoSize : 5;
        uint32_t PayLoadSize : 3;
    } txBF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiTXQCON;

// *****************************************************************************
//! Transmit Queue Status Register

typedef union _REG_CiTXQSTA {

    struct {
        uint32_t TxNotFullIF : 1;
        uint32_t unimplemented1 : 1;
        uint32_t TxEmptyIF : 1;
        uint32_t unimplemented2 : 1;
        uint32_t TxAttemptIF : 1;
        uint32_t TxError : 1;
        uint32_t TxLostArbitration : 1;
        uint32_t TxAborted : 1;
        uint32_t FifoIndex : 5;
        uint32_t unimplemented3 : 19;
    } txBF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiTXQSTA;

// *****************************************************************************
//! FIFO Control Register

typedef union _REG_CiFIFOCON {
    // Receive FIFO

    struct {
        uint32_t RxNotEmptyIE : 1;
        uint32_t RxHalfFullIE : 1;
        uint32_t RxFullIE : 1;
        uint32_t RxOverFlowIE : 1;
        uint32_t unimplemented1 : 1;
        uint32_t RxTimeStampEnable : 1;
        uint32_t unimplemented2 : 1;
        uint32_t TxEnable : 1;
        uint32_t UINC : 1;
        uint32_t unimplemented3 : 1;
        uint32_t FRESET : 1;
        uint32_t unimplemented4 : 13;
        uint32_t FifoSize : 5;
        uint32_t PayLoadSize : 3;
    } rxBF;

    // Transmit FIFO

    struct {
        uint32_t TxNotFullIE : 1;
        uint32_t TxHalfFullIE : 1;
        uint32_t TxEmptyIE : 1;
        uint32_t unimplemented1 : 1;
        uint32_t TxAttemptIE : 1;
        uint32_t unimplemented2 : 1;
        uint32_t RTREnable : 1;
        uint32_t TxEnable : 1;
        uint32_t UINC : 1;
        uint32_t TxRequest : 1;
        uint32_t FRESET : 1;
        uint32_t unimplemented3 : 5;
        uint32_t TxPriority : 5;
        uint32_t TxAttempts : 2;
        uint32_t unimplemented4 : 1;
        uint32_t FifoSize : 5;
        uint32_t PayLoadSize : 3;
    } txBF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiFIFOCON;

// *****************************************************************************
//! FIFO Status Register

typedef union _REG_CiFIFOSTA {
    // Receive FIFO

    struct {
        uint32_t RxNotEmptyIF : 1;
        uint32_t RxHalfFullIF : 1;
        uint32_t RxFullIF : 1;
        uint32_t RxOverFlowIF : 1;
        uint32_t unimplemented1 : 4;
        uint32_t FifoIndex : 5;
        uint32_t unimplemented2 : 19;
    } rxBF;

    // Transmit FIFO

    struct {
        uint32_t TxNotFullIF : 1;
        uint32_t TxHalfFullIF : 1;
        uint32_t TxEmptyIF : 1;
        uint32_t unimplemented1 : 1;
        uint32_t TxAttemptIF : 1;
        uint32_t TxError : 1;
        uint32_t TxLostArbitration : 1;
        uint32_t TxAborted : 1;
        uint32_t FifoIndex : 5;
        uint32_t unimplemented2 : 19;
    } txBF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiFIFOSTA;

// *****************************************************************************
//! FIFO User Address Register

typedef union _REG_CiFIFOUA {

    struct {
        uint32_t UserAddress : 12;
        uint32_t unimplemented1 : 20;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiFIFOUA;

// *****************************************************************************
//! Filter Control Register

typedef union _REG_CiFLTCON_BYTE {

    struct {
        uint32_t BufferPointer : 5;
        uint32_t unimplemented1 : 2;
        uint32_t Enable : 1;
    } bF;
    uint8_t byte;
} REG_CiFLTCON_BYTE;

// *****************************************************************************
//! Filter Object Register

typedef union _REG_CiFLTOBJ {
    CAN_FILTEROBJ_ID bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiFLTOBJ;

// *****************************************************************************
//! Mask Object Register

typedef union _REG_CiMASK {
    CAN_MASKOBJ_ID bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CiMASK;


// *****************************************************************************
// *****************************************************************************
/* MCP2517 Specific */

// *****************************************************************************
//! Oscillator Control Register

typedef union _REG_OSC {

    struct {
        uint32_t PllEnable : 1;
        uint32_t unimplemented1 : 1;
        uint32_t OscDisable : 1;
        uint32_t unimplemented2 : 1;
        uint32_t SCLKDIV : 1;
        uint32_t CLKODIV : 2;
        uint32_t unimplemented3 : 1;
        uint32_t PllReady : 1;
        uint32_t unimplemented4 : 1;
        uint32_t OscReady : 1;
        uint32_t unimplemented5 : 1;
        uint32_t SclkReady : 1;
        uint32_t unimplemented6 : 19;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_OSC;

// *****************************************************************************
//! I/O Control Register

typedef union _REG_IOCON {

    struct {
        uint32_t TRIS0 : 1;
        uint32_t TRIS1 : 1;
        uint32_t unimplemented1 : 2;
        uint32_t ClearAutoSleepOnMatch : 1;
        uint32_t AutoSleepEnable : 1;
        uint32_t XcrSTBYEnable : 1;
        uint32_t unimplemented2 : 1;
        uint32_t LAT0 : 1;
        uint32_t LAT1 : 1;
        uint32_t unimplemented3 : 5;
        uint32_t HVDETSEL : 1;
        uint32_t GPIO0 : 1;
        uint32_t GPIO1 : 1;
        uint32_t unimplemented4 : 6;
        uint32_t PinMode0 : 1;
        uint32_t PinMode1 : 1;
        uint32_t unimplemented5 : 2;
        uint32_t TXCANOpenDrain : 1;
        uint32_t SOFOutputEnable : 1;
        uint32_t INTPinOpenDrain : 1;
        uint32_t unimplemented6 : 1;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_IOCON;

// *****************************************************************************
//! CRC Regsiter

typedef union _REG_CRC {

    struct {
        uint32_t CRC : 16;
        uint32_t CRCERRIF : 1;
        uint32_t FERRIF : 1;
        uint32_t unimplemented1 : 6;
        uint32_t CRCERRIE : 1;
        uint32_t FERRIE : 1;
        uint32_t unimplemented2 : 6;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_CRC;

// *****************************************************************************
//! ECC Control Register

typedef union _REG_ECCCON {

    struct {
        uint32_t EccEn : 1;
        uint32_t SECIE : 1;
        uint32_t DEDIE : 1;
        uint32_t unimplemented1 : 5;
        uint32_t Parity : 7;
        uint32_t unimplemented2 : 17;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_ECCCON;

// *****************************************************************************
//! ECC Status Register

typedef union _REG_ECCSTA {

    struct {
        uint32_t unimplemented1 : 1;
        uint32_t SECIF : 1;
        uint32_t DEDIF : 1;
        uint32_t unimplemented2 : 13;
        uint32_t ErrorAddress : 12;
        uint32_t unimplemented3 : 4;
    } bF;
    uint32_t word;
    uint8_t byte[4];
} REG_ECCSTA;


// *****************************************************************************
// *****************************************************************************
/* Register Reset Values */

// *****************************************************************************
/* can_fd_ubp */

// Control Register Reset Values up to FIFOs
#define N_CAN_CTRL_REGS  20
static uint32_t canControlResetValues[] = {
    /* Address 0x000 to 0x00C */
#ifdef CAN_TXQUEUE_IMPLEMENTED
    0x04980760, 0x003E0F0F, 0x000E0303, 0x00021000,
#else
    0x04880760, 0x003E0F0F, 0x000E0303, 0x00021000,
#endif
    /* Address 0x010 to 0x01C */
    0x00000000, 0x00000000, 0x40400040, 0x00000000,
    /* Address 0x020 to 0x02C */
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* Address 0x030 to 0x03C */
    0x00000000, 0x00200000, 0x00000000, 0x00000000,
    /* Address 0x040 to 0x04C */
    0x00000400, 0x00000000, 0x00000000, 0x00000000
};

// FIFO Register Reset Values
#define N_CAN_FIFO_REGS (CAN_FIFO_TOTAL_CHANNELS*CiFIFO_OFFSET)
static uint32_t canFifoResetValues[] = {
    0x00600400, 0x00000000, 0x00000000
};

// Filter Control Register Reset Values
#define N_CAN_FILTER_CTRL_REGS (CAN_FILTER_TOTAL/4)
static uint32_t canFilterControlResetValue = 0x00000000;

// Filter and Mask Object Reset Values
#define N_CAN_FILTER_OBJ_REGS (CAN_FILTER_TOTAL*CiFILTER_OFFSET)
static uint32_t canFilterObjectResetValues[] = {
    0x00000000, 0x00000000
};

// *****************************************************************************
/* MCP2517 */

#ifdef MCP2517FD
#define N_MCP2517_CTRL_REGS 5
static uint32_t mcp2517ControlResetValues[] = {
    0x00000460, 0x00000003, 0x00000000, 0x00000000, 0x00000000
};
#endif

#ifdef  __cplusplus
}
#endif

#endif // _DRV_CANFDSPI_REGISTER_H
