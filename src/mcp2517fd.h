#ifndef MCP2517_h
#define MCP2517_h

#include "Arduino.h"
#include "mcp2517fd_defines.h"
#include <can_common.h>

//#define DEBUG_SETUP
#define FD_RX_BUFFER_SIZE	64
#define FD_TX_BUFFER_SIZE  32
#define FD_NUM_FILTERS 32

class MCP2517FD : public CAN_COMMON
{
  public:
	// Constructor defining which pins to use for CS and INT
    MCP2517FD(uint8_t CS_Pin, uint8_t INT_Pin);
	
	// Overloaded initialization function
	int Init(uint32_t nominalBaud, uint8_t freq);
	int Init(uint32_t CAN_Bus_Speed, uint8_t Freq, uint8_t SJW);
	int InitFD(uint32_t nominalBaud, uint32_t dataBaud, uint8_t freq);

    //block of functions which must be overriden from CAN_COMMON to implement functionality for this hardware
	int _setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended);
    int _setFilter(uint32_t id, uint32_t mask, bool extended);
	void resetHardware();
	uint32_t init(uint32_t ul_baudrate);
    uint32_t beginAutoSpeed();
    uint32_t set_baudrate(uint32_t ul_baudrate);
    void setListenOnlyMode(bool state);
	void enable();
	void disable();
	bool sendFrame(CAN_FRAME& txFrame);
	bool rx_avail();
	uint16_t available(); //like rx_avail but returns the number of waiting frames
	uint32_t get_rx_buff(CAN_FRAME &msg);
	//special FD functions required to reimplement to support FD mode
	uint32_t get_rx_buffFD(CAN_FRAME_FD &msg);
    uint32_t set_baudrateFD(uint32_t nominalSpeed, uint32_t dataSpeed);
    bool sendFrameFD(CAN_FRAME_FD& txFrame);
    uint32_t initFD(uint32_t nominalRate, uint32_t dataRate);
	
	// Basic MCP2517FD SPI Command Set
    void Reset();
    uint32_t Read(uint16_t address);
	uint8_t Read8(uint16_t address);
	uint16_t Read16(uint16_t address);
    void Read(uint16_t address, uint8_t data[], uint16_t bytes);
	void Write8(uint16_t address, uint8_t data);
	void Write16(uint16_t address, uint16_t data);
	void Write(uint16_t address, uint32_t data);
	void Write(uint16_t address, uint8_t data[], uint16_t bytes);
	void LoadFrameBuffer(uint16_t address, CAN_FRAME_FD &message);
	uint32_t ReadFrameBuffer(uint16_t address, CAN_FRAME_FD &message);

	uint8_t Status();
	uint8_t RXStatus();

	// Extra functions
	bool Interrupt();
	bool Mode(uint8_t mode); // Returns TRUE if mode change successful
	void setINTPin(uint8_t pin);
	void setCSPin(uint8_t pin);
	void EnqueueRX(CAN_FRAME_FD& newFrame);
	void EnqueueTX(CAN_FRAME_FD& newFrame);
	bool GetRXFrame(CAN_FRAME_FD& frame);
	void EnqueueRX(CAN_FRAME& newFrame);
	void EnqueueTX(CAN_FRAME& newFrame);
	bool GetRXFrame(CAN_FRAME& frame);
	void SetRXFilter(uint8_t filter, uint32_t FilterValue, bool ext);
	void SetRXMask(uint8_t mask, uint32_t MaskValue);
    void GetRXFilter(uint8_t filter, uint32_t &filterVal, boolean &isExtended);
    void GetRXMask(uint8_t mask, uint32_t &filterVal);
	void sendCallback(CAN_FRAME_FD *frame);
	void sendCallback(CAN_FRAME *frame);

	void InitFilters(bool permissive);
	void intHandler();
	void printDebug();
	void txQueueSetup();
    void setRXBufferSize(int newSize);
    void setTXBufferSize(int newSize);

    QueueHandle_t callbackQueueMCP;
    TaskHandle_t intTaskFD = NULL;
    TaskHandle_t taskHandleMCPCAN = NULL;
    TaskHandle_t taskHandleReset = NULL;
    bool needMCPReset = false;
    bool needTXFIFOReset = false;
    bool inFDMode;

  private:
	bool _init(uint32_t baud, uint8_t freq, uint8_t sjw, bool autoBaud);
	bool _initFD(uint32_t nominalSpeed, uint32_t dataSpeed, uint8_t freq, uint8_t sjw, bool autoBaud);
	void initSPI();
	void commonInit();	
    void handleFrameDispatch(CAN_FRAME_FD &frame, int filterHit);
    void handleFrameDispatch(CAN_FRAME &frame, int filterHit);
	void handleTXFifoISR(int fifo);
	void handleTXFifo(int fifo, CAN_FRAME_FD &newFrame);
	void handleTXFifo(int fifo, CAN_FRAME &newFrame);
    void initializeResources();
	uint32_t packExtValue(uint32_t input);
	uint32_t unpackExtValue(uint32_t input);
	uint32_t getErrorFlags();
	uint32_t getCIBDIAG0();
	uint32_t getCIBDIAG1();
	uint32_t getBitConfig();

    // Pin variables
	uint8_t _CS;
	uint8_t _INT;	
	volatile uint32_t savedNominalBaud;
	volatile uint32_t savedDataBaud;
	volatile uint8_t savedFreq;
	volatile uint8_t running; //1 if out of init code, 0 if still trying to initialize (auto baud detecting)
    bool initializedResources; //have we set up queues and interrupts?
    int rxBufferSize;
    int txBufferSize;
	QueueHandle_t	rxQueue;
	QueueHandle_t	txQueue;
	uint32_t errorFlags;
    uint32_t cachedDiag1;
};

extern MCP2517FD CAN1;

#endif
