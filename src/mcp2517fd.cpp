#include "Arduino.h"
#include <SPI.h>
#include "mcp2517fd.h"
#include "mcp2517fd_defines.h"
#include "mcp2517fd_regs.h"

/*
No amount of reception by itself seems to cause any mess ups in this library.
But, transmitting rapidly causes it to break badly. At that point even in debugging mode
there is no output of any kind. No debugging characters at all come through. It just dies.
It seems savvycan says that the connection itself dies but the serial console is still going
and the ESP32 itself and ESP32RET do not seem locked up.
*/


//20Mhz is the fastest we can go
#define FD_SPI_SPEED 10000000

SPISettings fdSPISettings(FD_SPI_SPEED, MSBFIRST, SPI_MODE0);

//Modified to loop, waiting for 1ms then pretending an interrupt came in
//basically switches to a polled system where we do not pay attention to actual interrupts
void task_MCPIntFD( void *pvParameters )
{
    const TickType_t iDelay = portTICK_PERIOD_MS;
    MCP2517FD* mcpCan = (MCP2517FD*)pvParameters;
    while (1)
    {
        vTaskDelay(iDelay);
        mcpCan->intHandler(); //not truly an interrupt handler anymore
    }
}

//checks ever 2 seconds to see if a CAN error happened. Will reset the CAN hardware if so.
void task_ResetWatcher(void *pvParameters)
{
    MCP2517FD* mcpCan = (MCP2517FD*)pvParameters;
    const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
    uint32_t ctrlVal;

    for(;;)
    {
        vTaskDelay( xDelay );

        //read CiCon then see if operation mode is b111/7 which is restricted mode. If so we reset
        ctrlVal = mcpCan->Read(ADDR_CiCON);
        ctrlVal = (ctrlVal >> 21) & 7;
        if (ctrlVal == 7)
        {
            mcpCan->needMCPReset = true;
            if (mcpCan->debuggingMode) Serial.println("!!!RESTRICTED MODE!!!");
        }

        if (mcpCan->needMCPReset)
        {
            mcpCan->needMCPReset = false;
            if (mcpCan->debuggingMode) Serial.println("Reset 2517FD hardware");
            mcpCan->resetHardware();
        }
        if (mcpCan->needTXFIFOReset)
        {
            mcpCan->needTXFIFOReset = false;
            if (mcpCan->debuggingMode) Serial.println("Reset TX FIFO");
            mcpCan->txQueueSetup();
        }
    }
}

//Reset the hardware to its last configured state
//Also prints out the diag registers if you have debugging text enabled.
//Saves then restores all filtering as well.
void MCP2517FD::resetHardware()
{
    uint32_t filters[32];
    uint32_t masks[32];
    uint32_t ctrl[8]; //each ctrl reg has four filters so only 8 regs not 32
    int idx;
    if (debuggingMode)
    {
        printf("Diag0: %x\n     Diag1: %x     ErrFlgs: %x", getCIBDIAG0(), cachedDiag1, getErrorFlags());
    }
    cachedDiag1 = 0;

    for (idx = 0; idx < 32; idx++)
    {
        filters[idx] = Read(ADDR_CiFLTOBJ + (CiFILTER_OFFSET * idx));
        masks[idx] = Read(ADDR_CiMASK + (CiFILTER_OFFSET * idx));
    }
    for (idx = 0; idx < 8; idx++) ctrl[idx] = Read(ADDR_CiFLTCON + 4 * idx);

    if (inFDMode) initFD(savedNominalBaud, savedDataBaud);
    else init(savedNominalBaud);

    for (idx = 0; idx < 32; idx++)
    {
        Write(ADDR_CiFLTOBJ + (CiFILTER_OFFSET * idx), filters[idx]);
        Write(ADDR_CiMASK + (CiFILTER_OFFSET * idx), masks[idx]);
    }
    for (idx = 0; idx < 8; idx++) Write(ADDR_CiFLTCON + 4 * idx, ctrl[idx]);
}

/*
Issue callbacks to registered functions and objects
Used to keep this kind of thing out of the interrupt handler
The callback type and mailbox are passed in the fid member of the
CAN_FRAME struct. It isn't really used by anything.
Layout of the storage:
bit   31 -    If set indicates an object callback
bits  24-30 - Idx into listener table
bits  0-7   - Mailbox number that triggered callback
*/
void task_MCPCAN( void *pvParameters )
{
    MCP2517FD* mcpCan = (MCP2517FD*)pvParameters;
    CAN_FRAME_FD rxFrameFD;
    CAN_FRAME rxFrame;

    while (1)
    {
        //receive next CAN frame from queue and fire off the callback
        if (mcpCan->callbackQueueMCP)
        {
            if (!mcpCan->inFDMode)
            {
                if(xQueueReceive(mcpCan->callbackQueueMCP, &rxFrame, portMAX_DELAY)==pdTRUE)
                {
                    mcpCan->canToFD(rxFrame, rxFrameFD);
                    mcpCan->sendCallback(&rxFrameFD);
                }
            }
            else
            {
                if(xQueueReceive(mcpCan->callbackQueueMCP, &rxFrameFD, portMAX_DELAY)==pdTRUE)
                {
                    mcpCan->sendCallback(&rxFrameFD);
                }
            }
        }
    }
}

void MCP2517FD::sendCallback(CAN_FRAME_FD *frame)
{
    //frame buffer
    CANListener *thisListener;
    int mb;
    int idx;
    CAN_FRAME stdFrame;
    bool isFD = false;

    if (!fdToCan(*frame, stdFrame)) isFD = true;

    mb = (frame->fid & 0xFF);
    if (mb == 0xFF) mb = -1;

    if (frame->fid & 0x80000000ul) //object callback
    {
        idx = (frame->fid >> 24) & 0x7F;
        thisListener = listener[idx];
        if (isFD) thisListener->gotFrameFD(frame, mb);
        else thisListener->gotFrame(&stdFrame, mb);
    }
    else //C function callback
    {
        if (isFD)
        {
          if (mb > -1) (*cbCANFrameFD[mb])(frame);
          else (*cbGeneralFD)(frame);
        }
        else
        {
          if (mb > -1) (*cbCANFrame[mb])(&stdFrame);
          else (*cbGeneral)(&stdFrame);
        }
    }
}

MCP2517FD::MCP2517FD(uint8_t CS_Pin, uint8_t INT_Pin) : CAN_COMMON(32) {
    pinMode(CS_Pin, OUTPUT);
    digitalWrite(CS_Pin,HIGH);
    pinMode(INT_Pin,INPUT);
    //digitalWrite(INT_Pin,HIGH);

    //attachInterrupt(INT_Pin, MCPFD_INTHandler, FALLING);

    _CS = CS_Pin;
    _INT = INT_Pin;

    savedNominalBaud = 0;
    savedDataBaud = 0;
    savedFreq = 0;
    running = 0;
    inFDMode = false;
    fdSupported = true;
    txBufferSize = FD_TX_BUFFER_SIZE;
    rxBufferSize = FD_RX_BUFFER_SIZE;
    initializedResources = false;
}

void MCP2517FD::setRXBufferSize(int newSize)
{
    rxBufferSize = newSize;
}

void MCP2517FD::setTXBufferSize(int newSize)
{
    txBufferSize = newSize;
}


void MCP2517FD::initializeResources()
{
    if (initializedResources) return;

    if (debuggingMode) Serial.println("initializeResources()");

    //queues allocate the requested space plus 96 bytes overhead
    if (inFDMode)
    {
        rxQueue = xQueueCreate(rxBufferSize, sizeof(CAN_FRAME_FD));
        txQueue = xQueueCreate(txBufferSize, sizeof(CAN_FRAME_FD));
        //as in the ESP32-Builtin CAN we create a queue and task to do callbacks outside the interrupt handler
        callbackQueueMCP = xQueueCreate(16, sizeof(CAN_FRAME_FD));
    }
    else
    {
        rxQueue = xQueueCreate(rxBufferSize, sizeof(CAN_FRAME));
        txQueue = xQueueCreate(txBufferSize, sizeof(CAN_FRAME));
        //as in the ESP32-Builtin CAN we create a queue and task to do callbacks outside the interrupt handler
        callbackQueueMCP = xQueueCreate(16, sizeof(CAN_FRAME));
    }

                           //func        desc    stack, params, priority, handle to task, which core to pin to
    //Tasks take up the stack you allocate here in bytes plus 388 bytes overhead            
    xTaskCreatePinnedToCore(&task_MCPCAN, "CAN_FD_CALLBACK", 6144, this, 8, &taskHandleMCPCAN, 0);
    xTaskCreatePinnedToCore(&task_MCPIntFD, "CAN_FD_INT", 4096, this, 19, &intTaskFD, 0);
    xTaskCreatePinnedToCore(&task_ResetWatcher, "CAN_RSTWATCH", 4096, this, 7, &taskHandleReset, 0);

    initializedResources = true;
}

void MCP2517FD::setINTPin(uint8_t pin)
{
    //detachInterrupt(_INT);
    _INT = pin;
    //pinMode(_INT,INPUT);
    //digitalWrite(_INT,HIGH);
    //attachInterrupt(_INT, MCPFD_INTHandler, FALLING);
}

void MCP2517FD::setCSPin(uint8_t pin)
{
    _CS = pin;
    pinMode(_CS, OUTPUT);
    digitalWrite(_CS,HIGH);
}

//Some sort of horrific closed head injury caused the designers of the MCP2517FD to store
//29 bit extended frames with the 18 extended bits first then the 11 standard bits added higher up
//instead of just plain making the 29 bit ID or mask contiguous in the 32 bit register. So, two routines
//here to massage the data bits around to match

//Take a 29 bit ID or MASK and turn it into a packed format suitable for the hardware register
uint32_t MCP2517FD::packExtValue(uint32_t input)
{
    return ( ((input >> 18) & 0x7FF) + ((input & 0x3FFFF) << 11) ); 
}

//Reverse that process for when we receive IDs in the stupid format and want the real ID
uint32_t MCP2517FD::unpackExtValue(uint32_t input)
{
    return ( ((input >> 11) & 0x3FFFF) + ((input & 0x7FF) << 18));
}

void MCP2517FD::initSPI()
{
    // Set up SPI Communication
    // dataMode can be SPI_MODE0 or SPI_MODE3 only for MCP2517FD
    SPI.begin(SCK, MISO, MOSI, SS);
    SPI.setClockDivider(spiFrequencyToClockDiv(FD_SPI_SPEED));
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setHwCs(false); //allow manual control of CS line
    if (debuggingMode) Serial.println("MCP2517FD SPI Inited");
}

/*
  Initialize MCP2517FD

  int CAN_Bus_Speed = transfer speed in kbps (or raw CAN speed in bits per second)
  int Freq = MCP2517FD oscillator frequency in MHz
  int SJW = Synchronization Jump Width Length bits - 1 to 4 (see data sheet)

  returns baud rate set

  Sending a bus speed of 0 kbps initiates AutoBaud and returns zero if no
  baud rate could be determined.  There must be two other active nodes on the bus!
*/
int MCP2517FD::Init(uint32_t CAN_Bus_Speed, uint8_t Freq) {
    return Init(CAN_Bus_Speed, Freq, 3);
}

int MCP2517FD::Init(uint32_t CAN_Bus_Speed, uint8_t Freq, uint8_t SJW) {
    if(SJW < 1) SJW = 1;
    if(SJW > 4) SJW = 4;
    REG_CiINT interruptFlags;

    initSPI();

    int i = 0;
    if(CAN_Bus_Speed > 0) 
    {
        if(_init(CAN_Bus_Speed, Freq, SJW, false)) 
        {
            savedNominalBaud = CAN_Bus_Speed;
            savedFreq = Freq;
            running = 1;
            errorFlags = 0;
            rxFault = false;
            txFault = false;
            faulted = false;
	        return CAN_Bus_Speed;
        }
    }
    else 
    {
	    for(i = 20; i < 1000; i = i + 5)
        {
	        if(_init(i, Freq, 1, true))
            {
                // check for bus activity
		        Write16(ADDR_CiINT,0); //write to INT flags to unset flags we can clear
		        delay(500); // need the bus to be communicating within this time frame
		        if(Interrupt())
                {
		            // determine which interrupt flags have been set
		            interruptFlags.word = Read(ADDR_CiINT);
		            if(!interruptFlags.bF.IF.CERRIF)
                    {
		                // to get here we must have received something without errors
		                Mode(CAN_NORMAL_MODE);
			            savedNominalBaud = i;
			            savedFreq = Freq;
			            running = 1;
                        errorFlags = 0;
                        rxFault = false;
                        txFault = false;
                        faulted = false;
			            return i;
		            }
		        }
	        }
	    }
    }
    return 0;
}

uint32_t MCP2517FD::initFD(uint32_t nominalRate, uint32_t dataRate)
{
    REG_CiINT interruptFlags;
    int i = 0;

    initSPI();

    if(nominalRate > 0)
    {
        if(_initFD(nominalRate, dataRate, 40, 4, false)) 
        {
            savedNominalBaud = nominalRate;
            savedDataBaud = dataRate;
            savedFreq = 40;
            running = 1;
            errorFlags = 0;
            rxFault = false;
            txFault = false;
            faulted = false;
	        return nominalRate;
        }
    }
    else
    {
	    for(i = 20; i < 1000; i = i + 5)
        {
	        if(_initFD(i, dataRate, 40, 4, true))
            {
                // check for bus activity
		        Write16(ADDR_CiINTENABLE,0); //write to INT flags to unset flags we can clear
		        delay(500); // need the bus to be communicating within this time frame
		        if(Interrupt()) 
                {
		            // determine which interrupt flags have been set
		            interruptFlags.word = Read(ADDR_CiINT);
		            if(!interruptFlags.bF.IF.CERRIF)
                    {
		                // to get here we must have received something without errors
		                Mode(CAN_NORMAL_MODE);
			            savedNominalBaud = i;
                        savedDataBaud = dataRate;
			            savedFreq = 40;
			            running = 1;
                        errorFlags = 0;
                        rxFault = false;
                        txFault = false;
                        faulted = false;
			            return i;
		            }
		        }
	        }
	    }
    }
    return 0;
}


void MCP2517FD::txQueueSetup()
{
    uint32_t debugVal;
    REG_CiTXQCON txQCon;
    const TickType_t xDelay = 1;

    Write8(ADDR_CiFIFOCON + 1, 0); //Unset TXREQ which should abort all pending TX frames in queue
    vTaskDelay(xDelay); //wait a bit for that to work

    txQCon.word = 0x400; //set FRESET to reset this FIFO
    Write(ADDR_CiTXQCON, txQCon.word);
    vTaskDelay(xDelay);

    //transmit queue set up
    txQCon.word = 0;
    txQCon.txBF.PayLoadSize = 7; //64 bytes
    txQCon.txBF.FifoSize = 8; //9 frame long FIFO
    txQCon.txBF.TxAttempts = 0; //No retries. One and done. It works or we throw it away
    txQCon.txBF.TxPriority = 15; //middle priority
    txQCon.txBF.TxEmptyIE = 0;
    Write(ADDR_CiTXQCON, txQCon.word);
    if (debuggingMode)
    {
        debugVal = Read(ADDR_CiTXQCON);
        Serial.println(debugVal, BIN);
    }
}

//chunks of the hardware init that are in common between standard and FD mode
//Setup timer hardware, FIFOs, and Transmit Queue
void MCP2517FD::commonInit()
{
    uint32_t debugVal;
    if (debuggingMode) Serial.println("commonInit()");

    REG_CiFIFOCON fifoCon;
    REG_CiTSCON tsCon;

    txQueueSetup();

    //builtin timerstamping setup
    tsCon.bF.TBCPrescaler = 39; //40x slow down means 1us resolution
    tsCon.bF.TBCEnable = 1;
    Write(ADDR_CiTSCON ,tsCon.word);
    if (debuggingMode) 
    {
        debugVal = Read(ADDR_CiTSCON);
        Serial.println(debugVal, BIN);
    }

    //Last FIFO we'll set up is receive fifo
    fifoCon.word = 0; //clear it all out to start fresh
    fifoCon.rxBF.TxEnable = 0; //Make FIFO1 a RX FIFO
    fifoCon.rxBF.FifoSize = 17; //18 frames long
    fifoCon.rxBF.PayLoadSize = 7; //64 byte payload possible
    fifoCon.rxBF.RxFullIE = 1; //if the FIFO fills up let the code know (hopefully never happens!)
    fifoCon.rxBF.RxNotEmptyIE = 1; //if the FIFO isn't empty let the code know 
    fifoCon.rxBF.RxTimeStampEnable = 1; //time stamp each frame as it comes in
    Write(ADDR_CiFIFOCON + (CiFIFO_OFFSET * 1), fifoCon.word); //Write to FIFO1
}

bool MCP2517FD::_init(uint32_t CAN_Bus_Speed, uint8_t Freq, uint8_t SJW, bool autoBaud) {
    REG_CiNBTCFG nominalCfg;
    REG_CiCON canConfig;
    int tseg1, tseg2;
    uint32_t debugVal;

    inFDMode = false;

    if (debuggingMode) Serial.println("_init()");

    Write(ADDR_CiINT,0); //disable all interrupts during init

    if (!initializedResources) initializeResources();

    // Reset MCP2517FD which puts us in config mode automatically
    Reset();
    delay(1);

    //but, just to be sure, explicitly ask for config mode
    if(!Mode(CAN_CONFIGURATION_MODE)) 
    {
        if (debuggingMode) Serial.println("Could not enter configuration mode");
        return false;
    }

    Write(ADDR_CiINT,0); //disable all interrupts during init
    Write(ADDR_CiBDIAG0, 0); //clear debug flags too.
    Write(ADDR_CiBDIAG1, 0);
    errorFlags = 0;
    cachedDiag1 = 0;

    /*the MCP2517 has a very wide range for setting the baud rate. The sys clock is 40Mhz by default.
    But, someone can pass a different value if using a different crystal.
    We'll target around "Freq" TQ (with a target 75% sample point) for each bit on the CAN
    so we want a prescaler that takes 1 million / target baud to get a prescaler.
    Obviously this makes the upper limit 1M CAN rate be a prescaler of 1.
    250K is a prescaler of 4, 33.333k is a prescale of 30
    We need TSEG1 + TSEG2 + 1 to equal Freq
    */
    // Set registers
    nominalCfg.bF.SJW = SJW;
    tseg1 = (unsigned int)(((float)Freq / 4.0) * 3.0) - 1;
    tseg2 = Freq - tseg1 - 1;
    nominalCfg.bF.TSEG1 = tseg1 - 1; //registers store value one less than actual
    nominalCfg.bF.TSEG2 = tseg2 - 1;
    if (tseg1 > 256 || tseg1 < 1)
    {
        if (debuggingMode) Serial.println("Nominal TSeg1 outside of limits. Invalid baud rate!");
        return false;
    }
    if (tseg2 > 128 || tseg2 < 1)
    {
        if (debuggingMode) Serial.println("Nominal TSeg2 outside of limits. Invalid baud rate!");
        return false;
    }
    nominalCfg.bF.BRP = (1000000ul / CAN_Bus_Speed) - 1;
    uint32_t calcRate = (Freq * 1000000ul) / (nominalCfg.bF.BRP + 1) / (tseg1 + tseg2 + 1);
    int errRate = abs(int(CAN_Bus_Speed - calcRate));
    if (errRate >= (CAN_Bus_Speed / 50)) //if more than 2% error in speed setting
    {
        if (debuggingMode) 
        {
            Serial.println("WARNING - Set baud does not match requested baud.");
            Serial.println(calcRate);
        }
    }

    canConfig.bF.IsoCrcEnable = 0; //Don't use newer ISO format. I think it might screw up normal can? Maybe this doesn't matter at all for std can
    canConfig.bF.DNetFilterCount = 0; //Don't use device net filtering
    canConfig.bF.ProtocolExceptionEventDisable = 0; //Allow softer handling of protocol exception
    canConfig.bF.WakeUpFilterEnable = 0; //Not using wakeup filter
    canConfig.bF.BitRateSwitchDisable = 1; //We won't allow FD rate switching in this standard init function
    canConfig.bF.RestrictReTxAttempts = 1; //Don't try sending frames forever if no one acks them
    canConfig.bF.EsiInGatewayMode = 0; //ESI reflects error status
    canConfig.bF.SystemErrorToListenOnly = 0; //Don't automatically switch to listen only on system error
    canConfig.bF.StoreInTEF = 0; // Don't store transmitted messages back into RAM
    canConfig.bF.TXQEnable = 1; //But, do enable the transmission queue and save space for it in RAM
    canConfig.bF.TxBandWidthSharing = 4; //wait 16 bit times before trying to send another frame. Allows other nodes a bit of room to butt in
    canConfig.bF.RequestOpMode = CAN_CONFIGURATION_MODE;
    Write(ADDR_CiCON, canConfig.word);
    Write(ADDR_CiNBTCFG, nominalCfg.word); //write the nominal bit time register
    if (debuggingMode)
    {
        debugVal = Read(ADDR_CiCON);
        printf("CiCON: %x\n", debugVal);
        debugVal = Read(ADDR_CiNBTCFG);
        printf("CiNBTCFG: %x\n", debugVal);
    }

    commonInit();

    Write(ADDR_CiINT,0); //disable all interrupts during init
    Write(ADDR_CiBDIAG0, 0); //clear debug flags too.
    Write(ADDR_CiBDIAG1, 0);
    errorFlags = 0;
    cachedDiag1 = 0;

    if(!autoBaud)
    {
        // Return to Normal mode
        if(!Mode(CAN_CLASSIC_MODE))
        {
            if (debuggingMode) Serial.println("Could not enter normal mode");
            return false;
        }
    }
    else
    {
        // Set to Listen Only mode
        if(!Mode(CAN_LISTEN_ONLY_MODE))
        {
            if (debuggingMode) Serial.println("Could not enter listen only mode");
            return false;
        }
    }

    // Enable interrupts
    Write(ADDR_CiINT, 0xB8030000); //Enable Invalid Msg, Bus err, sys err, rx overflow, rx fifo, tx fifo interrupts
    // Test that we can read back from the MCP2515 what we wrote to it
    uint32_t rtn = Read(ADDR_CiINT);
    if ((rtn & 0xFFFF0000) == 0xB8030000)
    {        
        if (debuggingMode) Serial.println("MCP2517 Init Success");
        errorFlags = 0;
        cachedDiag1 = 0;
        Write(ADDR_CiBDIAG0, 0); //clear debug flags too.
        Write(ADDR_CiBDIAG1, 0);
        rxFault = false;
        txFault = false;
        faulted = false;

        return true;
    }
    else
    {
        if (debuggingMode)
        {
            printf("MCP2517 Init Failed. CiINT = %x", rtn);
        }
        return false;
    }
    return false;
}

bool MCP2517FD::_initFD(uint32_t nominalSpeed, uint32_t dataSpeed, uint8_t freq, uint8_t sjw, bool autoBaud)
{
    REG_CiNBTCFG nominalCfg;
    REG_CiDBTCFG dataCfg;
    REG_CiCON canConfig;
    REG_CiTDC txDelayConfig;
    int tseg1, tseg2;

    uint32_t neededTQ;

    inFDMode = true;

    if (debuggingMode) Serial.println("_initFD()");

    if (!initializedResources) initializeResources();

    if (nominalSpeed < 125000) return 0; //won't work, die - Keep in mind that the FD spec doesn't want less than 500k here though
    if (dataSpeed < 1000000ul) return 0; //also won't work.

    // Reset MCP2517FD which puts us in config mode automatically
    Reset();
    delay(1);

    Write(ADDR_CiINT,0); //disable all interrupts during init

    /*Forget everything said about the baud rate generator in the _init function above. When we
    plan to be in FD mode it is best if the baud rate generator uses the same prescaler for both
    the nominal and data rates. 
    */
    // Set registers
    nominalCfg.bF.SJW = sjw;
    neededTQ = (freq * 1000000ul) / nominalSpeed;
    tseg1 = ((neededTQ * 8) / 10) - 1; //set sample point at 80%
    tseg2 = neededTQ - tseg1 - 1;
    if (tseg1 > 256 || tseg1 < 1)
    {
        if (debuggingMode) Serial.println("Nominal TSeg1 outside of limits. Invalid baud rates!");
        return false;
    }
    if (tseg2 > 128 || tseg2 < 1)
    {
        if (debuggingMode) Serial.println("Nominal TSeg2 outside of limits. Invalid baud rates!");
        return false;
    }

    nominalCfg.bF.TSEG1 = tseg1 - 1;
    nominalCfg.bF.TSEG2 = tseg2 - 1;
    nominalCfg.bF.BRP = 0; //baud rate prescaler. 0 = 1x prescale

    if (debuggingMode) Serial.printf("Nomimal Settings:  TSEG1: %u   TSEG2: %u\n", tseg1, tseg2);

    /*As above, we lock the prescaler at 1x and figure out the Seg1/Seg2 based on that.
    */
    dataCfg.bF.SJW = sjw;
    neededTQ = (freq * 1000000ul) / dataSpeed;
    tseg1 = ((neededTQ * 8) / 10) - 1; //set sample point at 80%
    tseg2 = neededTQ - tseg1 - 1;
    if (tseg1 > 32 || tseg1 < 1)
    {
        if (debuggingMode) Serial.println("Data TSeg1 outside of limits. Invalid baud rates!");
        return false;
    }
    if (tseg2 > 16 || tseg2 < 1)
    {
        if (debuggingMode) Serial.println("Data TSeg2 outside of limits. Invalid baud rates!");
        return false;
    }

    dataCfg.bF.TSEG1 = tseg1 - 1;
    dataCfg.bF.TSEG2 = tseg2 - 1;
    dataCfg.bF.BRP = 0;

    txDelayConfig.bF.TDCMode = 2; //automatic mode. The module will figure out the proper delay itself
    txDelayConfig.bF.TDCValue = 0; //this value set by hardware in auto mode
    txDelayConfig.bF.TDCOffset = (dataCfg.bF.BRP + 1) * (tseg1); //set value (dbrp * dtseg1) as a basepoint

    if (debuggingMode) Serial.printf("Data Settings:  TSEG1: %u   TSEG2: %u\n", tseg1, tseg2);

    canConfig.bF.IsoCrcEnable = 1; //It's likely we need ISO CRC mode active to get FD to work properly
    canConfig.bF.DNetFilterCount = 0; //Don't use device net filtering
    canConfig.bF.ProtocolExceptionEventDisable = 0; //Allow softer handling of protocol exception
    canConfig.bF.WakeUpFilterEnable = 0; //Not using wakeup filter
    canConfig.bF.BitRateSwitchDisable = 0; //Let each TX frame tell us whether to use FD rate switching or not
    canConfig.bF.RestrictReTxAttempts = 1; //Don't try sending frames forever if no one acks them
    canConfig.bF.EsiInGatewayMode = 0; //ESI reflects error status
    canConfig.bF.SystemErrorToListenOnly = 1; //Auto switch to listen only on system err bit
    canConfig.bF.StoreInTEF = 0; // Don't store transmitted messages back into RAM
    canConfig.bF.TXQEnable = 1; //But, do enable the transmission queue and save space for it in RAM
    canConfig.bF.TxBandWidthSharing = 4; //wait 16 bit times before trying to send another frame. Allows other nodes a bit of room to butt in
    canConfig.bF.RequestOpMode = CAN_CONFIGURATION_MODE;
    Write(ADDR_CiCON, canConfig.word);
    Write(ADDR_CiNBTCFG, nominalCfg.word); //write the nominal bit time register
    Write(ADDR_CiDBTCFG, dataCfg.word); //and write out the data rate register too
    Write(ADDR_CiTDC, txDelayConfig.word); //write out transmission delay compensation register.

    commonInit();

    if(!autoBaud)
    {
        // Return to Normal mode
        if(!Mode(CAN_NORMAL_MODE))
        {
            if (debuggingMode) Serial.println("Could not enter normal mode");
            return false;
        }
    }
    else
    {
        // Set to Listen Only mode
        if(!Mode(CAN_LISTEN_ONLY_MODE))
        {
            if (debuggingMode) Serial.println("Could not enter listen only mode");
            return false;
        }
    }
    // Enable interrupts
    Write(ADDR_CiINT, 0xB8030000); //Enable Invalid Msg, Bus err, sys err, rx overflow, rx fifo, tx fifo interrupts
    // Test that we can read back from the MCP2517FD what we wrote to it
    uint32_t rtn = Read(ADDR_CiINT);
    if ((rtn & 0xFFFF0000) == 0xB8030000)
    {
        if (debuggingMode) Serial.println("MCP2517 InitFD Success");
        errorFlags = 0;
        rxFault = false;
        txFault = false;
        faulted = false;
        return true;
    }
    else
    {
        if (debuggingMode)
        {
            Serial.println(rtn, HEX);
        }
        return false;
    }
    return false;
}

uint16_t MCP2517FD::available()
{
    if (!rxQueue) return 0; //why would this happen though?!
	return uxQueueMessagesWaiting(rxQueue);
}

int MCP2517FD::_setFilter(uint32_t id, uint32_t mask, bool extended)
{
    uint8_t filterCtrl;

    for (int i = 0; i < 32; i++)
    {
        filterCtrl = Read(ADDR_CiFLTCON + i);
        if ( !(filterCtrl & 0x80) )
        {
            _setFilterSpecific(i, id, mask, extended);
            return i;
        }
    }
    //if we got here then there were no free filters. Return value of deaaaaath!
    if (debuggingMode) Serial.println("Err: No filter set!");
    return -1;
}

//we've got 32 filters each with their own mask.
int MCP2517FD::_setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended)
{
    uint32_t packedID, packedMask;
    if (mailbox > 31) return 0; //past end of valid mailbox #said

    //First up, make sure it's disabled as you can't set an enabled filter/mask combination
    Write8(ADDR_CiFLTCON + mailbox, 0);
    //Then we can directly write the filter and mask out - Using extended to augment bit 30 appropriately
    packedID = id;
    packedMask = mask;
    if (extended) 
    {
        packedID = packExtValue(packedID);
        packedMask = packExtValue(packedMask);
    }
    else
    {
        packedID = packedID & 0x7FF;
        packedMask = packedMask & 0x7FF;
    }
    packedMask |= 1 << 30; //filter/mask combo must match extended/std exactly. Won't allow both
    if (extended) packedID |= 1 << 30; //only allow extended frames to match
    Write(ADDR_CiFLTOBJ + (CiFILTER_OFFSET * mailbox), packedID);
    Write(ADDR_CiMASK + (CiFILTER_OFFSET * mailbox), packedMask);
    Write8(ADDR_CiFLTCON + mailbox, 0x80 + 1); //Enable the filter and send it to FIFO1 which is the RX FIFO
    return mailbox;
}

uint32_t MCP2517FD::init(uint32_t ul_baudrate)
{
    return Init(ul_baudrate, 40);
}

uint32_t MCP2517FD::beginAutoSpeed()
{
    return Init(0, 40);
}

uint32_t MCP2517FD::set_baudrate(uint32_t ul_baudrate)
{
    return Init(ul_baudrate, 40);
}

uint32_t MCP2517FD::set_baudrateFD(uint32_t nominalSpeed, uint32_t dataSpeed)
{
    return _initFD(nominalSpeed, dataSpeed, 40, 4, false);
}

void MCP2517FD::setListenOnlyMode(bool state)
{
    if (state)
    {
        Mode(CAN_LISTEN_ONLY_MODE); //listen only seems to always accept FD frames
    }
    else
    {
        if (!inFDMode) Mode(CAN_CLASSIC_MODE);
        else Mode(CAN_NORMAL_MODE);
    }
}

void MCP2517FD::enable()
{
    if (!inFDMode) Mode(CAN_CLASSIC_MODE);
    else Mode(CAN_NORMAL_MODE);
}

void MCP2517FD::disable()
{
    Mode(CAN_CONFIGURATION_MODE);
}

bool MCP2517FD::sendFrame(CAN_FRAME& txFrame)
{
    EnqueueTX(txFrame);
    return true;
}

bool MCP2517FD::sendFrameFD(CAN_FRAME_FD& txFrame)
{
    EnqueueTX(txFrame);
    return true;
}

bool MCP2517FD::rx_avail()
{
    return available() > 0 ? true : false;
}

uint32_t MCP2517FD::get_rx_buff(CAN_FRAME &msg)
{
    CAN_FRAME_FD temp;
    bool ret = GetRXFrame(msg);
    //f (ret) ret = fdToCan(temp, msg);
    return ret;
}

uint32_t MCP2517FD::get_rx_buffFD(CAN_FRAME_FD &msg)
{
    return GetRXFrame(msg);
}

uint32_t MCP2517FD::getErrorFlags()
{
    return errorFlags;
}

uint32_t MCP2517FD::getCIBDIAG0()
{
    return Read(ADDR_CiBDIAG0);
}

uint32_t MCP2517FD::getCIBDIAG1()
{
    return Read(ADDR_CiBDIAG1);
}

uint32_t MCP2517FD::getBitConfig()
{
    return Read(ADDR_CiNBTCFG);
}

//Prints to screen basically all of the important registers including debugging registers.
void MCP2517FD::printDebug()
{
    Serial.println();
    Serial.print("Diag0: ");
    Serial.print(getCIBDIAG0(), HEX);
    Serial.print("  Diag1: ");
    Serial.print(getCIBDIAG1(), HEX);
    Serial.print("  ErrFlags: ");
    Serial.println(errorFlags, HEX);

    Serial.print("Ctrl: ");
    Serial.print(Read(ADDR_CiCON), HEX);
    Serial.print("  NomBits: ");
    Serial.print(Read(ADDR_CiNBTCFG), HEX);
    Serial.print("  DataBits: ");
    Serial.print(Read(ADDR_CiDBTCFG), HEX);
    Serial.print("  TDC: ");
    Serial.println(Read(ADDR_CiTDC), HEX);

    Serial.print("TxqCon: ");
    Serial.print(Read(ADDR_CiTXQCON), HEX);
    Serial.print("  TsCon: ");
    Serial.print(Read(ADDR_CiTSCON), HEX);
    Serial.print("  Int: ");
    Serial.println(Read(ADDR_CiINT), HEX);

    Serial.print("F0Ctrl: ");
    Serial.print(Read(ADDR_CiFIFOCON), HEX);
    Serial.print("  F0 Status: ");
    Serial.print(Read(ADDR_CiFIFOSTA), HEX);
    Serial.print("  F1Ctrl: ");
    Serial.print(Read(ADDR_CiFIFOCON + CiFIFO_OFFSET), HEX);
    Serial.print("  F1 Status: ");
    Serial.println(Read(ADDR_CiFIFOSTA + CiFIFO_OFFSET), HEX);
}

void MCP2517FD::Reset() {
    SPI.beginTransaction(fdSPISettings);
    digitalWrite(_CS,LOW);
    SPI.transfer(CMD_RESET); //always need to send 12 bit address too
    SPI.transfer(CMD_RESET); //so even reset is two bytes
    digitalWrite(_CS,HIGH);
    SPI.endTransaction();
}

uint32_t MCP2517FD::Read(uint16_t address) {
    SPI.beginTransaction(fdSPISettings);
    digitalWrite(_CS,LOW);
    SPI.transfer((CMD_READ << 4)| ( (address >> 8) & 0xF) );
    SPI.transfer(address & 0xFF);
    uint32_t data = SPI.transfer(0);
    data += (SPI.transfer(0) << 8);
    data += (SPI.transfer(0) << 16);
    data += (SPI.transfer(0) << 24);
    digitalWrite(_CS,HIGH);
    SPI.endTransaction();
    return data;
}

uint8_t MCP2517FD::Read8(uint16_t address)
{
    SPI.beginTransaction(fdSPISettings);
    digitalWrite(_CS,LOW);
    SPI.transfer((CMD_READ << 4) | ((address >> 8) & 0xF));
    SPI.transfer(address & 0xFF);
    uint8_t data = SPI.transfer(0x00);
    digitalWrite(_CS,HIGH);
    SPI.endTransaction();
    return data;
}

uint16_t MCP2517FD::Read16(uint16_t address)
{
    SPI.beginTransaction(fdSPISettings);
    digitalWrite(_CS,LOW);
    SPI.transfer((CMD_READ << 4) | ((address >> 8) & 0xF));
    SPI.transfer(address & 0xFF);
    uint16_t data = SPI.transfer(0);
    data += (SPI.transfer(0) << 8);
    digitalWrite(_CS,HIGH);
    SPI.endTransaction();
    return data;
}

void MCP2517FD::Read(uint16_t address, uint8_t data[], uint16_t bytes) {
    // allows for sequential reading of registers starting at address - see data sheet
    SPI.beginTransaction(fdSPISettings);
    digitalWrite(_CS,LOW);
    SPI.transfer((CMD_READ << 4) | ((address >> 8) & 0xF));
    SPI.transfer(address & 0xFF);
    SPI.transferBytes(NULL, data, bytes); //read only operation
    digitalWrite(_CS,HIGH);
    SPI.endTransaction();
}

uint32_t MCP2517FD::ReadFrameBuffer(uint16_t address, CAN_FRAME_FD &message) {
    uint32_t buffer[19]; //76 bytes

    //there is no read buffer command anymore. Need to read from RAM on the chip
    //quickly read the whole thing then process it afterward
    SPI.beginTransaction(fdSPISettings);
    digitalWrite(_CS,LOW);
    SPI.transfer((CMD_READ << 4) | ((address >> 8) & 0xF));
    SPI.transfer(address & 0xFF);
    //read enough of the RAM to get an 8 byte message. 
    //Then we check to see if we really need to read more.
    //This prevents having to read 64 data bytes if we were just receiving normal frames.
    SPI.transferBytes(NULL, (uint8_t *)&buffer[0], 20);
    message.length = buffer[1] & 0xF;
    switch (message.length)
    {
    case 9:
        message.length = 12;
        break;
    case 10:
        message.length = 16;
        break;
    case 11:
        message.length = 20;
        break;
    case 12:
        message.length = 24;
        break;
    case 13:
        message.length = 32;
        break;
    case 14:
        message.length = 48;
        break;
    case 15:
        message.length = 64;
        break;
    }
    int neededBytes = message.length - 8;
    if (neededBytes > 0) SPI.transferBytes(NULL, (uint8_t *)&buffer[5], neededBytes);
    digitalWrite(_CS,HIGH);
    SPI.endTransaction();
  /*message in RAM is as follows:
    The first 32 bits are the message ID, either 11 or 29 bit (12 bit not handled yet), then bit 29 is RRS
    The next 32 bits have:
       first 4 bits are DLC
       Then
       IDE (extended address) (bit 4)
       RTR (Remote request) (bit 5)
       BRS (Baud rate switch for data) (bit 6)
       FDF (FD mode) (Bit 7)
       ESI (1 = tx node error passive, 0 = tx node error active) (bit 8)
       Bits 11 - 15 = Filter hit (0-31)
    The next 32 bits are all the timestamp (in microseconds for us)
    Then each additional byte is a data byte (up to 64 bytes)
  */
    message.extended = (buffer[1] >> 4) & 1;
    if (message.extended) message.id = unpackExtValue(buffer[0] & 0x1FFFFFFFull);
    else message.id = buffer[0] & 0x7FF;
    message.fid = 0;
    message.priority = 0;
    message.fdMode = (buffer[1] >> 7) & 1;
    if (message.fdMode)
        message.rrs = buffer[0] >> 29 & 1;
    else
        message.rrs = buffer[1] >> 5 & 1;
    message.timestamp = buffer[2];
    if (!message.fdMode && message.length > 8) message.length = 8;
    //only copy the number of words we really have to.
    int copyWords = (message.length + 3) / 4;
    for (int j = 0; j < copyWords; j++) message.data.uint32[j] = buffer[3 + j];
    return (buffer[1] >> 11) & 31; //return which filter produced this message
}

void MCP2517FD::Write8(uint16_t address, uint8_t data) {
    //taskDISABLE_INTERRUPTS();
    SPI.beginTransaction(fdSPISettings);
    digitalWrite(_CS,LOW);
    SPI.transfer((CMD_WRITE << 4) | ((address >> 8) & 0xF) );
    SPI.transfer(address & 0xFF);
    SPI.transfer(data);
    digitalWrite(_CS,HIGH);
    SPI.endTransaction();
    //taskENABLE_INTERRUPTS();
}

void MCP2517FD::Write16(uint16_t address, uint16_t data) {
    //taskDISABLE_INTERRUPTS();
    SPI.beginTransaction(fdSPISettings);
    digitalWrite(_CS,LOW);
    SPI.transfer((CMD_WRITE << 4) | ((address >> 8) & 0xF) );
    SPI.transfer(address & 0xFF);
    SPI.transfer(data & 0xFF);
    SPI.transfer((data >> 8) & 0xFF);
    digitalWrite(_CS,HIGH);
    SPI.endTransaction();
    //taskENABLE_INTERRUPTS();
}

void MCP2517FD::Write(uint16_t address, uint32_t data) {
    //taskDISABLE_INTERRUPTS();
    SPI.beginTransaction(fdSPISettings);
    digitalWrite(_CS,LOW);
    SPI.transfer((CMD_WRITE << 4) | ((address >> 8) & 0xF) );
    SPI.transfer(address & 0xFF);
    SPI.transfer(data & 0xFF);
    SPI.transfer((data >> 8) & 0xFF);
    SPI.transfer((data >> 16) & 0xFF);
    SPI.transfer((data >> 24) & 0xFF);
    digitalWrite(_CS,HIGH);
    SPI.endTransaction();
    //taskENABLE_INTERRUPTS();
}

void MCP2517FD::Write(uint16_t address, uint8_t data[], uint16_t bytes) {
    // allows for sequential writing of registers starting at address - see data sheet
    uint8_t i;
    //taskDISABLE_INTERRUPTS();
    SPI.beginTransaction(fdSPISettings);
    digitalWrite(_CS,LOW);
    SPI.transfer((CMD_WRITE << 4) | ((address >> 8) & 0xF) );
    SPI.transfer(address & 0xFF);
    SPI.writeBytes(data, bytes);
    digitalWrite(_CS,HIGH);
    SPI.endTransaction();
    //taskENABLE_INTERRUPTS();
}

void MCP2517FD::LoadFrameBuffer(uint16_t address, CAN_FRAME_FD &message)
{
    uint8_t buffer[76];
    uint32_t *buffPtr;
    int dataBytes;
    /*message in RAM is as follows (same as RX but without the timestamp):
    The first 32 bits are the message ID, either 11 or 29 bit (12 bit not handled yet), then bit 29 is RRS
    The next 32 bits have:
       first 4 bits are DLC
       Then
       IDE (extended address) (bit 4)
       RTR (Remote request) (bit 5)
       BRS (Baud rate switch for data) (bit 6)
       FDF (FD mode) (Bit 7)
       ESI (1 = tx node error passive, 0 = tx node error active) (bit 8)
       SEQ (sequence number) (Bits 9-15) - Optional sequence number for your reference
    Then each additional byte is a data byte (up to 64 bytes)
    */
    buffer[0] = (CMD_WRITE << 4) | ((address >> 8) & 0xF);
    buffer[1] = address & 0xFF;
    buffPtr = (uint32_t *)&buffer[2];

    if (message.extended) buffPtr[0] = packExtValue(message.id);
    else buffPtr[0] = message.id & 0x7FF;

    buffPtr[1] = (message.extended) ? (1 << 4) : 0;
    buffPtr[1] |= (message.fdMode) ? (3 << 6) : 0; //set both the BRS and FDF bits at once
    if (message.fdMode)
        buffPtr[0] |= (message.rrs) ? (1 << 29) : 0;
    else
        buffPtr[1] |= (message.rrs) ? (1 << 5) : 0;
    if (!message.fdMode && message.length > 8) message.length = 8;
    dataBytes = message.length;

    //promote requested frame length to the next allowable size
    if (message.length > 8 && message.length < 13) buffPtr[1] |= 9;
    else if (message.length > 12 && message.length < 17) buffPtr[1] |= 10;
    else if (message.length > 16 && message.length < 21) buffPtr[1] |= 11;
    else if (message.length > 20 && message.length < 25) buffPtr[1] |= 12;
    else if (message.length > 24 && message.length < 33) buffPtr[1] |= 13;
    else if (message.length > 32 && message.length < 49) buffPtr[1] |= 14;
    else if (message.length > 48 && message.length < 65) buffPtr[1] |= 15;
    else
    {
        if (message.length < 9) buffPtr[1] |= message.length;
        else buffPtr[1] |= 8;
    }

    //only copy the number of data words we really have to.
    int copyWords = (dataBytes + 3) / 4;
    for (int j = 0; j < copyWords; j++)
    {
        buffPtr[2 + j] = message.data.uint32[j];
    }

    if (debuggingMode)
    {
        Serial.print("Load TXBufr Addr ");
        Serial.print(address, HEX);
        Serial.print(" Buff0 ");
        Serial.print(buffPtr[0], HEX);
        Serial.print(" Buff1 ");
        Serial.println(buffPtr[1], HEX);
    }

    //taskDISABLE_INTERRUPTS();
    SPI.beginTransaction(fdSPISettings);
    digitalWrite(_CS,LOW);
    SPI.writeBytes((uint8_t *) &buffer[0], 10 + (copyWords * 4)); //and only write just as many bytes as we need to for this frame
    digitalWrite(_CS,HIGH);
    SPI.endTransaction();
    //taskENABLE_INTERRUPTS();
    Write8(ADDR_CiFIFOCON + 1, 3); //Set UINC and TX_Request
}

bool MCP2517FD::Interrupt() {
    return (digitalRead(_INT)==LOW);
}

bool MCP2517FD::Mode(byte mode) {
    uint32_t tempMode;
    if (debuggingMode) Serial.println("Mode");
    tempMode = Read(ADDR_CiCON);
    tempMode &= 0xF8FFFFFF;
    tempMode |= (mode << 24ul);
    Write(ADDR_CiCON, tempMode);
    delay(6); // allow for any transmissions to complete
    uint8_t data = Read8(ADDR_CiCON + 2);
    if (debuggingMode) 
    {
        Serial.println(data, BIN);
        Serial.println(mode, BIN);
    }
    return ((data >> 5) == mode);
}

/*Initializes filters to either accept all frames or accept none
  By default on the MCP2517FD all filters are not enabled and probably
  that means you won't get anything unless you pass that you want
  permissive mode.
*/
void MCP2517FD::InitFilters(bool permissive) {
    uint32_t value;
    uint32_t value32;
    if (permissive)
    {
        value = 0;
        value32 = 0;
    }
    else
    {
        value = 0x7FF; //all 11 bits set
        value32 = 0x1FFFFFFF; //all 29 bits set
    }

    _setFilterSpecific(0, value, value, false);
    _setFilterSpecific(1, value32, value32, true);
}

//Places the given frame into the receive queue
void IRAM_ATTR MCP2517FD::EnqueueRX(CAN_FRAME_FD& newFrame) {
    xQueueSend(rxQueue, &newFrame, 0);
}

void IRAM_ATTR MCP2517FD::EnqueueRX(CAN_FRAME& newFrame) {
    xQueueSend(rxQueue, &newFrame, 0);
}


//Places the given frame either into a hardware FIFO (if there is space)
//or into the software side queue if there was no room
void MCP2517FD::EnqueueTX(CAN_FRAME_FD& newFrame) {
    handleTXFifo(0, newFrame);
}

void MCP2517FD::EnqueueTX(CAN_FRAME& newFrame) {
    handleTXFifo(0, newFrame);
}

bool MCP2517FD::GetRXFrame(CAN_FRAME_FD &frame) {
    if (!rxQueue) return false;
    if (xQueueReceive(rxQueue, &frame, 0) == pdTRUE) return true;
    return false;
}

bool MCP2517FD::GetRXFrame(CAN_FRAME &frame) {
    if (!rxQueue) return false;
    if (xQueueReceive(rxQueue, &frame, 0) == pdTRUE) return true;
    return false;
}

//Not truly an interrupt handler in the sense that it does NOT run in interrupt context
//but it does handle the MCP2517FD interrupt still.
void MCP2517FD::intHandler(void) {
    CAN_FRAME_FD messageFD;
    CAN_FRAME message;
    uint32_t ctrlVal;
    uint32_t status;
    uint16_t addr;
    uint32_t filtHit;

    if (!running) return;

    // determine which interrupt flags have been set
    uint32_t interruptFlags = Read(ADDR_CiINT);

    //if(interruptFlags & 1)  //Transmit FIFO interrupt
    //{
      //Only FIFO0 is TX so no need to ask for which FIFO.
      //Write8(ADDR_CiFIFOCON, 0x80); //Keep FIFO as TX but disable Queue Empty Interrupt
      //handleTXFifoISR(0);
    //}
    //else //didn't get TX interrupt but check if we've got msgs in FIFO and see if we can queue them into hardware
    {
      if (uxQueueMessagesWaiting(txQueue) > 0) handleTXFifoISR(0); //if we have messages to send then try to queue them in the TX fifo
    }

    if(interruptFlags & 2)  //Receive FIFO interrupt
    {
        //no need to ask which FIFO matched, there is only one RX FIFO configured in this library
        //So, ask for frames out of the FIFO until it no longer has any
        status = Read( ADDR_CiFIFOSTA + (CiFIFO_OFFSET * 1) );
        while (status & 1) // there is at least one message waiting to be received
        {
            //Get address to read from
            addr = Read(ADDR_CiFIFOUA + (CiFIFO_OFFSET * 1));
            //Then use that address to read the frame
            filtHit = ReadFrameBuffer(addr + 0x400, messageFD); //stupidly the returned address from FIFOUA needs an offset
            Write8(ADDR_CiFIFOCON + (CiFIFO_OFFSET * 1) + 1, 1); //set UINC (it's at bit 8 in the register so we move one byte into register and write 8 bits)
            status = Read( ADDR_CiFIFOSTA + (CiFIFO_OFFSET * 1) ); //read the register again to see if there are more frames waiting
            if (inFDMode)
                handleFrameDispatch(messageFD, filtHit);
            else
            {
                fdToCan(messageFD, message);
                handleFrameDispatch(message, filtHit);
            }
        }
    }
    if (interruptFlags & (1 << 11)) //Receive Object Overflow
    {
        //once again, we know which FIFO must have overflowed - what to do about it though?
        errorFlags |= 1;
    }
    if (interruptFlags & (1 << 12)) //System error
    {
        errorFlags |= 2;
    }
    if (interruptFlags & (1 << 13)) //CANBus error
    {
        errorFlags |= 4;
    }
    if (interruptFlags & (1 << 15)) //Invalid msg interrupt
    {
        errorFlags |= 8;
    }
    if (errorFlags > 0)
    {
        //if (debuggingMode) Serial.write('?');
        uint32_t diagBits = getCIBDIAG1(); //get a detailed fault status

        if (diagBits & 0x3030000) //either NBIT0 or NBIT1 error (or DBIT0, DBIT1)
        {
            txFault = true;
            faulted = true;
            Serial.print("**");
            cachedDiag1 |= diagBits;
            needMCPReset = true; //if set true we'd auto reset the hardware when bit errors happen.
            Write(ADDR_CiBDIAG1, 0);
        }
        if (diagBits & 0x40000) //18 - NACK fault
        {
            //Just set the TXfault flag not the genera faulted flag. This might be just a lack of CAN bus
            txFault = true;
        }
        if (diagBits & 0x38380000) //19 - RX Fixed form, Bit stuff, or CRC error, either FD or not
        {
            rxFault = true;
            faulted = true;
            Serial.print("!!");
            cachedDiag1 |= diagBits;
            needMCPReset = true; //could reset when these errors happen. Maybe count errors to decide?
            Write(ADDR_CiBDIAG1, 0);
        }
        if (diagBits & 0x800000) //23 - TXBOERR - device went bus-off (and auto recovered)
        {
            //it's OK if it goes bus off and tries to recover. Don't reset as we might mess up the bus if we're insane
        }
        if (diagBits & 0x40000000) //30 - ESI of RX FD message was set
        {
            rxFault = true;
            faulted = true;
        }
        if (diagBits & 0x80000000) //31 - DLC mismatch during RX or TX
        {
            //allow a dlc mismatch and don't do anything
            //rxFault = true;
        }
    }

    //Now, acknowledge the interrupts by clearing the intf bits
    Write16(ADDR_CiINT, 0);
}

//TX fifo is 0
void MCP2517FD::handleTXFifoISR(int fifo)
{
    uint32_t status;
    uint16_t addr;
    CAN_FRAME_FD frameFD;
    CAN_FRAME frame;
    uint8_t wroteFrames = 0;

    //if (fifo < 0) return;
    //if (fifo > 2) return;

    //taskDISABLE_INTERRUPTS();

    status = Read( ADDR_CiFIFOSTA + (CiFIFO_OFFSET * fifo) );
    //Write8(ADDR_CiFIFOSTA + (CiFIFO_OFFSET * fifo), 0); //reset all the fault bits in the FIFO (TXABT, TXLARB, TXERR)
    //Write8(ADDR_CiBDIAG0 + (CiFIFO_OFFSET * fifo) + 1, 0); //Clear TX error counter
    //Write8(ADDR_CiBDIAG1 + (CiFIFO_OFFSET * fifo) + 2, 0); //clear bits 16-23 in diag1 register
    if ((status & 0x20) == 0x20) needTXFIFOReset = true; //if the queue registered a fault then set flag to have it reset
    //While the FIFO has room and we still have frames to send
    while ( (status & 1) && (uxQueueMessagesWaiting(txQueue) > 0) ) //while fifo is not full and we have messages waiting
    {
        if (debuggingMode)
        {
            Serial.write('~');
        }
        //get address to write to
        addr = Read( ADDR_CiFIFOUA + (CiFIFO_OFFSET * fifo) );
        if (inFDMode) 
        {
            //if (debuggingMode) Serial.write('F');
            if (xQueueReceive(txQueue, &frameFD, 0) != pdTRUE) return; //abort if we can't load a frame from the queue!
        }
        else
        {
            //if (debuggingMode) Serial.write('S');
            if (xQueueReceive(txQueue, &frame, 0) != pdTRUE) return;
            //hardware loading below always uses the same buffer save whether CAN or CANFD
            if (!canToFD(frame, frameFD)) return;
        }
        LoadFrameBuffer( addr + 0x400, frameFD );
        wroteFrames = 1;
        status = Read(ADDR_CiFIFOSTA + (CiFIFO_OFFSET * fifo));
    }
    if (wroteFrames != 0)
    {
        //Write8(ADDR_CiFIFOCON + (CiFIFO_OFFSET * fifo), 0x84); //Keep as TX queue and enable interrupt for queue empty
        if (debuggingMode) Serial.write('\"');
    }
  //taskENABLE_INTERRUPTS();
}

/*
This routine used to try to load a frame into hardware if possible. But, testing has shown that
the ESP32 doesn't really take kindly to multiple pathways trying to use SPI at once. The SPI
library just plain dies if you do that. So, all SPI access is through the interrupt handler now
To make this work this routine just pretends an interrupt came in and so the interrupt handler
sees our frame and tries to send it.
*/
void MCP2517FD::handleTXFifo(int fifo, CAN_FRAME_FD &newFrame)
{
    uint32_t status;
    uint16_t addr;
    //BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (fifo < 0) return;
    if (fifo > 2) return;

    //status = Read( ADDR_CiFIFOSTA + (CiFIFO_OFFSET * fifo) );
    //if (status & 1) //FIFO has room for this message - immediately send it to hardware
    //{
    //  addr = Read( ADDR_CiFIFOUA + (CiFIFO_OFFSET * fifo) );
    //  LoadFrameBuffer( addr + 0x400, newFrame );
    //  Write8(ADDR_CiFIFOCON + (CiFIFO_OFFSET * fifo) + 1, 3); //Set UINC and TX_Request
    //}
    //else //no room on hardware. Locally buffer in software
    //{
    //try to queue, do not wait if we can't. If we can then pretend an interrupt happened.
    if (!txQueue) return;
    if (xQueueSend(txQueue, &newFrame, 0) == pdPASS)
    {
        if (debuggingMode) Serial.write('+');
        //xHigherPriorityTaskWoken = xTaskNotifyGive(intTaskFD); //send notice to the handler task that it can do the SPI transaction now
    }
    else //full queue, if debugging we'll dump the hardware state.
    {
        //Write8(ADDR_CiFIFOCON + (CiFIFO_OFFSET * fifo) + 1, 2); //Set TX_Request to make sure the FIFO is really trying to send
        if (debuggingMode)
        {
            Serial.write('<');
            Serial.println();
            printDebug();
        }
    }
  //}
}

void MCP2517FD::handleTXFifo(int fifo, CAN_FRAME &newFrame)
{
    uint32_t status;
    uint16_t addr;
    CAN_FRAME_FD fd;
    BaseType_t ret;

    if (fifo < 0) return;
    if (fifo > 2) return;

    if (!txQueue) return;

    if (inFDMode) //if we're in FD mode the queue takes CAN-FD frames
    {
        canToFD(newFrame, fd);
        ret = xQueueSend(txQueue, &fd, 0);
    }
    else
    {
        ret = xQueueSend(txQueue, &newFrame, 0);
    }
    if (ret == pdPASS)
    {
        if (debuggingMode) Serial.write('+');
    }
    else //full queue, if debugging we'll dump the hardware state.
    {
        if (debuggingMode)
        {
            Serial.write('<');
            Serial.println();
            printDebug();
        }
    }
}


/*The idea here is to use the fid member (which is not normally used) as a signal to
  downstream of the type of callback that needs to be done.
  The lowest 8 bits are used to pass the filter that matched (or FF if general callback)
  If bit 31 is set then it's an object callback and the upper byte (minus that top one) are the listener position
*/
void MCP2517FD::handleFrameDispatch(CAN_FRAME_FD &frame, int filterHit)
{
    CANListener *thisListener;

    //First, try to send a callback. If no callback registered then buffer the frame.
    if (cbCANFrame[filterHit]) 
    {
        frame.fid = filterHit;
        xQueueSend(callbackQueueMCP, &frame, 0);
        return;
    }
    else if (cbGeneral)
    {
        frame.fid = 0xFF;
        xQueueSend(callbackQueueMCP, &frame, 0);
        return;
    }
    else
    {
        for (int listenerPos = 0; listenerPos < SIZE_LISTENERS; listenerPos++)
        {
            thisListener = listener[listenerPos];
            if (thisListener != NULL)
            {
                if (thisListener->isCallbackActive(filterHit))
                {
                    frame.fid = 0x80000000ul + (listenerPos << 24ul) + filterHit;
                    xQueueSend(callbackQueueMCP, &frame, 0);
                    return;
                }
                else if (thisListener->isCallbackActive(numFilters)) //global catch-all
                {
                    frame.fid = 0x80000000ul + (listenerPos << 24ul) + 0xFF;
                    xQueueSend(callbackQueueMCP, &frame, 0);
                    return;
                }
            }
        }
    }
    //if none of the callback types caught this frame then queue it in the buffer
    xQueueSend(rxQueue, &frame, 0);
}

void MCP2517FD::handleFrameDispatch(CAN_FRAME &frame, int filterHit)
{
    CANListener *thisListener;

    //First, try to send a callback. If no callback registered then buffer the frame.
    if (cbCANFrame[filterHit]) 
    {
        frame.fid = filterHit;
        xQueueSend(callbackQueueMCP, &frame, 0);
        return;
    }
    else if (cbGeneral)
    {
        frame.fid = 0xFF;
        xQueueSend(callbackQueueMCP, &frame, 0);
        return;
    }
    else
    {
        for (int listenerPos = 0; listenerPos < SIZE_LISTENERS; listenerPos++)
        {
            thisListener = listener[listenerPos];
            if (thisListener != NULL)
            {
                if (thisListener->isCallbackActive(filterHit))
                {
                    frame.fid = 0x80000000ul + (listenerPos << 24ul) + filterHit;
                    xQueueSend(callbackQueueMCP, &frame, 0);
                    return;
                }
                else if (thisListener->isCallbackActive(numFilters)) //global catch-all
                {
                    frame.fid = 0x80000000ul + (listenerPos << 24ul) + 0xFF;
                    xQueueSend(callbackQueueMCP, &frame, 0);
                    return;
                }
            }
        }
    }
    //if none of the callback types caught this frame then queue it in the buffer
    xQueueSend(rxQueue, &frame, 0);
}
