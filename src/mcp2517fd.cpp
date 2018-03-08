#include "Arduino.h"
#include "SPI.h"
#include "mcp2517fd.h"
#include "mcp2517fd_defines.h"
#include "mcp2517fd_regs.h"

SPISettings canSPISettings(20000000, MSBFIRST, SPI_MODE0);

QueueHandle_t	callbackQueueMCP;

void MCP_INTHandler() {
  Can1.intHandler();
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
    CAN_FRAME_FD rxFrame;

    while (1)
    {
        //receive next CAN frame from queue and fire off the callback
        if(xQueueReceive(callbackQueueMCP, &rxFrame, portMAX_DELAY)==pdTRUE)
        {
            mcpCan->sendCallback(&rxFrame);
        }
    }
}

//TODO; Must be modified to support figuring out whether frame is FD or not and sending to appropriate callbacks accordingly
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

//try to put a standard CAN frame into a CAN_FRAME_FD structure
bool MCP2517FD::canToFD(CAN_FRAME &source, CAN_FRAME_FD &dest)
{
  dest.id = source.id;
  dest.fid = source.fid;
  dest.rrs = source.rtr;
  dest.priority = source.priority;
  dest.extended = source.extended;
  dest.fdMode = false;
  dest.timestamp = source.timestamp;
  dest.length = source.length;
  dest.data.uint64[0] = source.data.uint64;
  return true;
}

//Try to do inverse - turn a CANFD frame into a standard CAN_FRAME struct
bool MCP2517FD::fdToCan(CAN_FRAME_FD &source, CAN_FRAME &dest)
{
  if (source.length > 8) return false;
  if (source.fdMode > 0) return false;
  dest.id = source.id;
  dest.fid = source.fid;
  dest.rtr = source.rrs;
  dest.priority = source.priority;
  dest.extended = source.extended;
  dest.timestamp = source.timestamp;
  dest.length = source.length;
  dest.data.uint64 = source.data.uint64[0];
  return true;
}

MCP2517FD::MCP2517FD(uint8_t CS_Pin, uint8_t INT_Pin) : CAN_COMMON(32) {
  pinMode(CS_Pin, OUTPUT);
  digitalWrite(CS_Pin,HIGH);
  pinMode(INT_Pin,INPUT);
  digitalWrite(INT_Pin,HIGH);

  attachInterrupt(INT_Pin, MCP_INTHandler, FALLING);
  
  _CS = CS_Pin;
  _INT = INT_Pin;
  
  savedNominalBaud = 0;
  savedDataBaud = 0;
  savedFreq = 0;
  running = 0; 
  inFDMode = false;
  
  rxQueue = xQueueCreate(RX_BUFFER_SIZE, sizeof(CAN_FRAME_FD));
	txQueue[0] = xQueueCreate(TX_BUFFER_SIZE, sizeof(CAN_FRAME_FD));
	txQueue[1] = xQueueCreate(TX_BUFFER_SIZE, sizeof(CAN_FRAME_FD));
	txQueue[2] = xQueueCreate(TX_BUFFER_SIZE, sizeof(CAN_FRAME_FD));

  //as in the ESP32-Builtin CAN we create a queue and task to do callbacks outside the interrupt handler
  //TODO: Both versions should be pinned to application processor so we don't get cross-thread issues
  callbackQueueMCP = xQueueCreate(32, sizeof(CAN_FRAME_FD));
                  //func        desc    stack, params, priority, handle to task
  xTaskCreate(&task_MCPCAN, "CAN_RX_MCP", 2048, this, 5, NULL);
}

void MCP2517FD::setINTPin(uint8_t pin)
{
  detachInterrupt(_INT);
  _INT = pin;
  pinMode(_INT,INPUT);
  digitalWrite(_INT,HIGH);
  attachInterrupt(_INT, MCP_INTHandler, FALLING);
}

void MCP2517FD::setCSPin(uint8_t pin)
{
  _CS = pin;
  pinMode(_CS, OUTPUT);
  digitalWrite(_CS,HIGH);
}

/*
  Initialize MCP2515
  
  int CAN_Bus_Speed = transfer speed in kbps (or raw CAN speed in bits per second)
  int Freq = MCP2515 oscillator frequency in MHz
  int SJW = Synchronization Jump Width Length bits - 1 to 4 (see data sheet)
  
  returns baud rate set
  
  Sending a bus speed of 0 kbps initiates AutoBaud and returns zero if no
  baud rate could be determined.  There must be two other active nodes on the bus!
*/
int MCP2517FD::Init(uint32_t CAN_Bus_Speed, uint8_t Freq) {
  return Init(CAN_Bus_Speed, Freq, 1);
}

int MCP2517FD::Init(uint32_t CAN_Bus_Speed, uint8_t Freq, uint8_t SJW) {
  if(SJW < 1) SJW = 1;
  if(SJW > 4) SJW = 4;
  REG_CiINT interruptFlags;
  int i = 0;
  if(CAN_Bus_Speed > 0) {
    if(_init(CAN_Bus_Speed, Freq, SJW, false)) {
      savedNominalBaud = CAN_Bus_Speed;
      savedFreq = Freq;
      running = 1;
	    return CAN_Bus_Speed;
    }
  } 
  else {
	  for(i = 20; i < 1000; i = i + 5) {
	    if(_init(i, Freq, 1, true)) {
		    // check for bus activity
		    Write16(ADDR_CiINT,0); //write to INT flags to unset flags we can clear
		    delay(500); // need the bus to be communicating within this time frame
		    if(Interrupt()) {
		      // determine which interrupt flags have been set
		      interruptFlags.word = Read(ADDR_CiINT);
		      if(!interruptFlags.bF.IF.CERRIF) {
		        // to get here we must have received something without errors
		        Mode(CAN_NORMAL_MODE);
			      savedNominalBaud = i;
			      savedFreq = Freq;	
			      running = 1;
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
  if(nominalRate > 0) {
    if(_initFD(nominalRate, dataRate, 40, 4, false)) {
      savedNominalBaud = nominalRate;
      savedDataBaud = dataRate;
      savedFreq = 40;
      running = 1;
	    return nominalRate;
    }
  } 
  else {
	  for(i = 20; i < 1000; i = i + 5) {
	    if(_initFD(i, dataRate, 40, 4, true)) {
		    // check for bus activity
		    Write16(ADDR_CiINTENABLE,0); //write to INT flags to unset flags we can clear
		    delay(500); // need the bus to be communicating within this time frame
		    if(Interrupt()) {
		      // determine which interrupt flags have been set
		      interruptFlags.word = Read(ADDR_CiINT);
		      if(!interruptFlags.bF.IF.CERRIF) {
		        // to get here we must have received something without errors
		        Mode(CAN_NORMAL_MODE);
			      savedNominalBaud = i;
            savedDataBaud = dataRate;
			      savedFreq = 40;	
			      running = 1;
			      return i;
		      }
		    }
	    }
	  }
  }
  return 0;
}

//chunks of the hardware init that are in common between standard and FD mode
//Setup timer hardware, FIFOs, and Transmit Queue
void MCP2517FD::commonInit()
{
  REG_CiTXQCON txQCon;
  REG_CiFIFOCON fifoCon;
  REG_CiTSCON tsCon;

  txQCon.word = 0;
  txQCon.txBF.PayLoadSize = 7; //64 bytes
  txQCon.txBF.FifoSize = 2; //3 frame long FIFO
  txQCon.txBF.TxAttempts = 1; //3 attempts then quit
  txQCon.txBF.TxPriority = 15; //middle priority
  txQCon.txBF.TxEmptyIE = 1; //enable interrupt for empty FIFO
  Write(ADDR_CiTXQCON, txQCon.word);

  tsCon.bF.TBCPrescaler = 39; //40x slow down means 1us resolution
  tsCon.bF.TBCEnable = 1;
  Write(ADDR_CiTSCON ,tsCon.word);

  fifoCon.txBF.TxEnable = 1; //Make FIFO1 a TX FIFO
  fifoCon.txBF.TxPriority = 0;
  fifoCon.txBF.FifoSize = 2; //3 frames long
  fifoCon.txBF.PayLoadSize = 7;
  fifoCon.txBF.TxAttempts = 2; //3 attempts then quit
  fifoCon.txBF.TxEmptyIE = 1;
  Write(ADDR_CiFIFOCON + CiFIFO_OFFSET, fifoCon.word); //Write to FIFO1

  fifoCon.txBF.TxPriority = 31;
  Write(ADDR_CiFIFOCON + (CiFIFO_OFFSET * 2), fifoCon.word); //Write to FIFO2

  fifoCon.word = 0; //clear it all out to start fresh
  fifoCon.rxBF.TxEnable = 0; //Make FIFO3 a RX FIFO
  fifoCon.rxBF.FifoSize = 17; //18 frames long
  fifoCon.rxBF.PayLoadSize = 7; //64 byte payload possible
  fifoCon.rxBF.RxFullIE = 1; //if the FIFO fills up let the code know (hopefully never happens!)
  fifoCon.rxBF.RxNotEmptyIE = 1; //if the FIFO isn't empty let the code know 
  fifoCon.rxBF.RxTimeStampEnable = 1;
  Write(ADDR_CiFIFOCON + (CiFIFO_OFFSET * 3), fifoCon.word); //Write to FIFO3
}

bool MCP2517FD::_init(uint32_t CAN_Bus_Speed, uint8_t Freq, uint8_t SJW, bool autoBaud) {
  REG_CiNBTCFG nominalCfg;
  REG_CiCON canConfig;

  // Reset MCP2517FD which puts us in config mode automatically
  Reset();
  delay(1);

  Write(ADDR_CiINT,0); //disable all interrupts during init

  /*the MCP2517 has a very wide range for setting the baud rate. The sys clock is 40Mhz by default.
    But, someone can pass a different value if using a different crystal.
    We'll target around "Freq" TQ (with a target 75% sample point) for each bit on the CAN 
    so we want a prescaler that takes 1 million / target baud to get a prescaler.
    Obviously this makes the upper limit 1M CAN rate be a prescaler of 1. 
    250K is a prescaler of 4, 33.333k is a prescale of 30
    then we set TSEG1 = ((Freq / 4) * 3) - 1 and TSEG2 = (Freq - TSEG1 - 1)
  */
  // Set registers
  nominalCfg.bF.SJW = SJW;
  nominalCfg.bF.TSEG1 = (unsigned int)(((float)Freq / 4.0) * 3.0) - 2;
  nominalCfg.bF.TSEG2 = Freq - nominalCfg.bF.TSEG1 - 1;
  nominalCfg.bF.BRP = (1000000ul / CAN_Bus_Speed) - 1;
  canConfig.bF.IsoCrcEnable = 0; //Don't use newer ISO format. I think it might screw up normal can? Maybe this doesn't matter at all for std can
  canConfig.bF.DNetFilterCount = 0; //Don't use device net filtering
  canConfig.bF.ProtocolExceptionEventDisable = 0; //Allow softer handling of protocol exception
  canConfig.bF.WakeUpFilterEnable = 0; //Not using wakeup filter
  canConfig.bF.BitRateSwitchDisable = 1; //We won't allow FD rate switching in this standard init function
  canConfig.bF.RestrictReTxAttempts = 1; //Don't try sending frames forever if no one acks them
  canConfig.bF.EsiInGatewayMode = 0; //ESI reflects error status 
  canConfig.bF.SystemErrorToListenOnly = 1; //Auto switch to listen only on system err bit
  canConfig.bF.StoreInTEF = 0; // Don't store transmitted messages back into RAM
  canConfig.bF.TXQEnable = 1; //But, do enable the transmission queue and save space for it in RAM
  canConfig.bF.TxBandWidthSharing = 4; //wait 16 bit times before trying to send another frame. Allows other nodes a bit of room to butt in
  canConfig.bF.RequestOpMode = CAN_CONFIGURATION_MODE;
  Write(ADDR_CiCON, canConfig.word);
  Write(ADDR_CiNBTCFG, nominalCfg.word); //write the nominal bit time register
  
  commonInit();

  if(!autoBaud) {
    // Return to Normal mode
    if(!Mode(CAN_CLASSIC_MODE)) 
    {
        //SerialUSB.println("Could not enter normal mode");
        return false;
    }
  } else {
    // Set to Listen Only mode
    if(!Mode(CAN_LISTEN_ONLY_MODE)) 
    {
        //SerialUSB.println("Could not enter listen only mode");
        return false;
    }
  }
  // Enable interrupts
  Write(ADDR_CiINT, 0xB8030000); //Enable Invalid Msg, Bus err, sys err, rx overflow, rx fifo, tx fifo interrupts
  // Test that we can read back from the MCP2515 what we wrote to it
  byte rtn = Read(ADDR_CiINT);
  if ((rtn & 0xFFFF0000) == 0xB8030000) 
  {
    inFDMode = false;
    return true;
  }
  else 
  {
    //Serial.println(data, HEX);
    //Serial.println(rtn, HEX);
    return false;
  }

  return false;
}

bool MCP2517FD::_initFD(uint32_t nominalSpeed, uint32_t dataSpeed, uint8_t freq, uint8_t sjw, bool autoBaud)
{
  REG_CiNBTCFG nominalCfg;
  REG_CiDBTCFG dataCfg;
  REG_CiCON canConfig;
  REG_CiTXQCON txQCon;
  REG_CiFIFOCON fifoCon;
  REG_CiTSCON tsCon;

  uint32_t neededTQ;

  if (nominalSpeed < 125000) return 0; //won't work, die
  if (dataSpeed < 1000000ul) return 0; //also won't work.

  // Reset MCP2517FD which puts us in config mode automatically
  Reset();
  delay(1);

  Write(ADDR_CiINT,0); //disable all interrupts during init

  /*Forget everything said about the baud rate generator in the _init function above. When we
    plan to be in FD mode it is best if the baud rate generator uses the same prescaler for both
    the nominal and data rates. So, this is forced to a prescaler of 1 for both. This gives the
    best resolution and results for FD mode. As such, you can't use a baud rate under 125k or
    this routine will die. If you need less than 125k you aren't using FD mode and should be using
    the non-FD routines.
  */
  // Set registers
  nominalCfg.bF.SJW = sjw;
  neededTQ = (freq * 1000000ul) / nominalSpeed;
  nominalCfg.bF.TSEG1 = ((neededTQ * 8) / 10) - 1; //set sample point at 80%
  nominalCfg.bF.TSEG2 = (neededTQ - nominalCfg.bF.TSEG1) - 1;
  nominalCfg.bF.BRP = 0;

  /*As above, we lock the prescaler at 1x and figure out the Seg1/Seg2 based on that.
  */
  dataCfg.bF.SJW = sjw;
  neededTQ = (freq * 1000000ul) / dataSpeed;
  dataCfg.bF.TSEG1 = ((neededTQ * 8) / 10) - 1; //set sample point at 80%
  dataCfg.bF.TSEG2 = (neededTQ - nominalCfg.bF.TSEG1) - 1;
  dataCfg.bF.BRP = 0;
  
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

  commonInit();

  if(!autoBaud) {
    // Return to Normal mode
    if(!Mode(CAN_NORMAL_MODE)) 
    {
        //SerialUSB.println("Could not enter normal mode");
        return false;
    }
  } else {
    // Set to Listen Only mode
    if(!Mode(CAN_LISTEN_ONLY_MODE)) 
    {
        //SerialUSB.println("Could not enter listen only mode");
        return false;
    }
  }
  // Enable interrupts
  Write(ADDR_CiINT, 0xB8030000); //Enable Invalid Msg, Bus err, sys err, rx overflow, rx fifo, tx fifo interrupts
  // Test that we can read back from the MCP2517FD what we wrote to it
  byte rtn = Read(ADDR_CiINT);
  if ((rtn & 0xFFFF0000) == 0xB8030000) 
  {
    inFDMode = true;
    return true;
  }
  else 
  {
    //Serial.println(data, HEX);
    //Serial.println(rtn, HEX);
    return false;
  }
  return false;  
}

uint16_t MCP2517FD::available()
{
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
    return -1;
}

//we don't exactly have mailboxes, we have filters (6 of them) but it's the same basic idea
int MCP2517FD::_setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended)
{
  if (mailbox > 31) return 0; //past end of valid mailbox #said

  //First up, make sure it's disabled as you can't set an enabled filter/mask combination
  Write8(ADDR_CiFLTCON + mailbox, 0);
  //Then we can directly write the filter and mask out - Using extended to augment bit 30 appropriately
  mask |= 1 << 30; //filter/mask combo must match extended/std exactly. Won't allow both
  if (extended) id |= 1 << 30; //only allow extended frames to match
  Write(ADDR_CiFLTOBJ + (CiFILTER_OFFSET * mailbox), id);
  Write(ADDR_CiMASK + (CiFILTER_OFFSET * mailbox), mask);
  Write8(ADDR_CiFLTCON + mailbox, 0x80 + 3); //Enable the filter and send it to FIFO3 which is the RX FIFO
}

uint32_t MCP2517FD::init(uint32_t ul_baudrate)
{
    Init(ul_baudrate, 40);
}

uint32_t MCP2517FD::beginAutoSpeed()
{
    Init(0, 40);
}

uint32_t MCP2517FD::set_baudrate(uint32_t ul_baudrate)
{
  Init(ul_baudrate, 40);
}

uint32_t MCP2517FD::set_baudrateFD(uint32_t nominalSpeed, uint32_t dataSpeed)
{
  _initFD(nominalSpeed, dataSpeed, 40, 4, false);
}

void MCP2517FD::setListenOnlyMode(bool state)
{
    if (state)
        Mode(CAN_LISTEN_ONLY_MODE); //listen only seems to always accept FD frames
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
    Mode(CAN_SLEEP_MODE);
}

bool MCP2517FD::sendFrame(CAN_FRAME& txFrame)
{
    CAN_FRAME_FD temp;
    canToFD(txFrame, temp);
    EnqueueTX(temp);
}

bool MCP2517FD::sendFrameFD(CAN_FRAME_FD& txFrame)
{
    EnqueueTX(txFrame);
}

bool MCP2517FD::rx_avail()
{
    return available() > 0 ? true : false;
}

uint32_t MCP2517FD::get_rx_buff(CAN_FRAME &msg)
{
    CAN_FRAME_FD temp;
    bool ret = GetRXFrame(temp);
    if (ret) ret = fdToCan(temp, msg);
    return ret;
}

uint32_t MCP2517FD::get_rx_buffFD(CAN_FRAME_FD &msg)
{
    return GetRXFrame(msg);
}

void MCP2517FD::Reset() {
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CMD_RESET);
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
}

uint32_t MCP2517FD::Read(uint16_t address) {
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CMD_READ | ((address >> 8)&0xF));
  SPI.transfer(address & 0xFF);
  uint32_t data = SPI.transfer(0x00);
  data += (SPI.transfer(0x00)) >> 8;
  data += (SPI.transfer(0x00)) >> 16;
  data += (SPI.transfer(0x00)) >> 24;
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
  return data;
}

uint8_t MCP2517FD::Read8(uint16_t address) 
{
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CMD_READ | ((address >> 8)&0xF));
  SPI.transfer(address & 0xFF);
  uint8_t data = SPI.transfer(0x00);
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
  return data;
}

uint16_t MCP2517FD::Read16(uint16_t address) 
{
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CMD_READ | ((address >> 8)&0xF));
  SPI.transfer(address & 0xFF);
  uint16_t data = SPI.transfer(0x00);
  data += (SPI.transfer(0x00)) >> 8;
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
  return data;
}

void MCP2517FD::Read(uint16_t address, uint8_t data[], uint16_t bytes) {
  // allows for sequential reading of registers starting at address - see data sheet
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CMD_READ | ((address >> 8)&0xF));
  SPI.transfer(address & 0xFF);
  SPI.transferBytes(NULL, data, bytes); //read only operation
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
}

uint32_t MCP2517FD::ReadFrameBuffer(uint16_t address, CAN_FRAME_FD &message) {
  uint32_t buffer[19]; //76 bytes
  
  //there is no read buffer command anymore. Need to read from RAM on the chip
  //quickly read the whole thing then process it afterward
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CMD_READ | ((address >> 8)&0xF));
  SPI.transfer(address & 0xFF);
  SPI.transferBytes(NULL, (uint8_t *)&buffer[0], 76);
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

  message.id = buffer[0] & 0x1FFFFFFFull;
  message.fid = 0;
  message.priority = 0;
  message.extended = (buffer[1] >> 4) & 1;
  message.fdMode = (buffer[1] >> 7) & 1;
  if (message.fdMode) 
    message.rrs = buffer[0] >> 29 & 1;
  else
    message.rrs = buffer[1] >> 5 & 1;
  message.timestamp = buffer[2];
  message.length = buffer[1];
  if (message.fdMode)
  {
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
  }
  for (int j = 0; j < 32; j++) message.data.uint32[j] = buffer[3 + j];
  return (buffer[1] >> 11) & 31;
}

void MCP2517FD::Write8(uint16_t address, uint8_t data) {
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CMD_WRITE | ((address >> 8) & 0xF) );
  SPI.transfer(address & 0xFF);
  SPI.transfer(data);
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
}

void MCP2517FD::Write16(uint16_t address, uint16_t data) {
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CMD_WRITE | ((address >> 8) & 0xF) );
  SPI.transfer(address & 0xFF);
  SPI.transfer16(data);
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
}

void MCP2517FD::Write(uint16_t address, uint32_t data) {
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CMD_WRITE | ((address >> 8) & 0xF) );
  SPI.transfer(address & 0xFF);
  SPI.transfer32(data);
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
}

void MCP2517FD::Write(uint16_t address, uint8_t data[], uint16_t bytes) {
  // allows for sequential writing of registers starting at address - see data sheet
  uint8_t i;
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CMD_WRITE | ((address >> 8) & 0xF) );
  SPI.transfer(address & 0xFF);
  SPI.writeBytes(data, bytes);
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
}

void MCP2517FD::LoadFrameBuffer(uint16_t address, CAN_FRAME_FD &message) {
  uint32_t buffer[19];
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
       Bits 11 - 15 = Filter hit (0-31)
    Then each additional byte is a data byte (up to 64 bytes)
  */
  buffer[0] = message.id;
  buffer[1] = (message.extended) ? (1 << 4) : 0;
  buffer[1] |= (message.fdMode) ? (1 << 7) : 0; 
  if (message.fdMode)
    buffer[1] |= (message.rrs) ? (1 << 29) : 0;
  else
    buffer[1] |= (message.rrs) ? (1 << 5) : 0;
  if (message.fdMode && message.length > 8) message.length = 8;
  switch (message.length)
  {
  case 12:
    buffer[1] |= 9;
    break;
  case 16:
    buffer[1] |= 10;
    break;
  case 20:
    buffer[1] |= 11;
    break;
  case 24:
    buffer[1] |= 12;
    break;
  case 32:
    buffer[1] |= 13;
    break;
  case 48:
    buffer[1] |= 14;
    break;
  case 64:
    buffer[1] |= 15;
    break;
  default:
    if (message.length < 9) 
      buffer[1] |= message.length;
    else buffer[1] |= 8;
  }
  for (int j = 0; j < 32; j++) buffer[2 + j] = message.data.uint32[j];

  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CMD_WRITE | ((address >> 8) & 0xF) );  
  SPI.transfer(address & 0xFF);
  SPI.writeBytes((uint8_t *) &buffer[0], 72);
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
}

bool MCP2517FD::Interrupt() {
  return (digitalRead(_INT)==LOW);
}

bool MCP2517FD::Mode(byte mode) {
  uint32_t tempMode;
  tempMode = Read(ADDR_CiCON);
  tempMode &= 0xF8FFFFFF;
  tempMode |= (mode << 24ul);
  Write(ADDR_CiCON, tempMode);
  delay(6); // allow for any transmissions to complete
  uint8_t data = Read8(ADDR_CiCON + 2);
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
	if (permissive) {
		value = 0;
    value32 = 0;
	}	
	else {
		value = 0x7FF; //all 11 bits set
    value32 = 0x1FFFFFFF; //all 29 bits set
	}

  _setFilterSpecific(0, value, value, false);
  _setFilterSpecific(1, value32, value32, true);
}

//Places the given frame into the receive queue
void MCP2517FD::EnqueueRX(CAN_FRAME_FD& newFrame) {
	xQueueSendFromISR(rxQueue, &newFrame, NULL); //we're probably in ISR when we call this so use that version
}

//Places the given frame either into a hardware FIFO (if there is space)
//or into the software side queue if there was no room
void MCP2517FD::EnqueueTX(CAN_FRAME_FD& newFrame) {
  //TXQ has priority of 15 (middle), FIFO1 is priority 0, FIFO2 is priority 31
  //So we try to find the nearest of the three to put the incoming frame. This allows
  //for an actual three priorities for CAN frames not the 32 you'd expect. But, there is
  //still some concept of priority so it still counts.
  if (newFrame.priority < 8) //FIFO1
  {
    handleTXFifo(1, newFrame);
  }
  else if (newFrame.priority > 23) //FIFO2
  {
    handleTXFifo(2, newFrame);
  }
  else //TXQ (FIFO0)
  {
    handleTXFifo(0, newFrame);
  }
}

bool MCP2517FD::GetRXFrame(CAN_FRAME_FD &frame) {
  if (xQueueReceive(rxQueue, &frame, 0) == pdTRUE) return true;
	return false;
}

void MCP2517FD::intHandler(void) {
    CAN_FRAME_FD message;
    uint32_t ctrlVal;
    uint32_t status;
    uint16_t addr;
    uint32_t filtHit;
    // determine which interrupt flags have been set
    uint32_t interruptFlags = Read(ADDR_CiINT);
    //Now, acknowledge the interrupts by clearing the intf bits
    Write16(ADDR_CiINT, 0); 	
    
    if(interruptFlags & 1)  //Transmit FIFO interrupt
    {
      //FIFOs 0, 1, 2 are TX so we do need to ask which one(s) triggered
      uint8_t fifos = Read8(ADDR_CiTXIF);
      //The idea here is to check whether the FIFO is not full and whether we have frames to send.
      //If it is both not full and we have a frame queued then push it into the FIFO and check again
      if (fifos & 1) //FIFO 0 - Mid priority
      {
        handleTXFifoISR(0);
      }
      if (fifos & 2) //FIFO 1 - Low priority
      {
        handleTXFifoISR(1);
      }
      if (fifos & 4) //FIFO 2 - Hi priority
      {
        handleTXFifoISR(2);
      }
    }
    if(interruptFlags & 2)  //Receive FIFO interrupt
    {
      //no need to ask which FIFO matched, there is only one RX FIFO configured in this library
      //So, ask for frames out of the FIFO until it no longer has any
      status = Read( ADDR_CiFIFOSTA + (CiFIFO_OFFSET * 2) );
      while (status & 1) // there is at least one message waiting to be received
      {
        //Get address to read from
        addr = Read(ADDR_CiFIFOUA + (CiFIFO_OFFSET * 3));
        //Then use that address to read the frame
        filtHit = ReadFrameBuffer(addr + 0x400, message); //stupidly the returned address from FIFOUA needs an offset
        Write8(ADDR_CiFIFOCON + (CiFIFO_OFFSET * 3) + 1, 1); //set UINC (it's at bit 8 in the register so we move one byte into register and write 8 bits)
        status = Read( ADDR_CiFIFOSTA + (CiFIFO_OFFSET * 3) ); //read the register again to see if there are more frames waiting
        handleFrameDispatch(message, filtHit);
      }
    }
    if (interruptFlags & (1 << 11)) //Receive Object Overflow
    {
      //once again, we know which FIFO must have overflowed - what to do about it though?
    }
    if (interruptFlags & (1 << 12)) //System error
    {

    }
    if (interruptFlags & (1 << 13)) //CANBus error
    {

    }
    if (interruptFlags & (1 << 15)) //Invalid msg interrupt
    {

    }
}

//TX fifos are 0, 1, 2
void MCP2517FD::handleTXFifoISR(int fifo)
{
  uint32_t status;
  uint16_t addr;
  CAN_FRAME_FD frame;

  if (fifo < 0) return;
  if (fifo > 2) return;
  status = Read( ADDR_CiFIFOSTA + (CiFIFO_OFFSET * fifo) );
  //While the FIFO has room and we still have frames to send
  while ( (status & 1) && (uxQueueMessagesWaitingFromISR(txQueue[fifo]) > 0) )
  {
    //get address to write to
    addr = Read( ADDR_CiFIFOUA + (CiFIFO_OFFSET * fifo) );
    if (xQueueReceiveFromISR(txQueue[fifo], &frame, NULL) != pdTRUE) return; //abort if we can't load a frame from the queue!
    LoadFrameBuffer( addr + 0x400, frame );
    Write8(ADDR_CiFIFOCON + (CiFIFO_OFFSET * fifo) + 1, 3); //Set UINC and TX_Request
    status = Read(ADDR_CiFIFOSTA + (CiFIFO_OFFSET * fifo));
  }
}

void MCP2517FD::handleTXFifo(int fifo, CAN_FRAME_FD &newFrame)
{
  uint32_t status;
  uint16_t addr;

  if (fifo < 0) return;
  if (fifo > 2) return;

  status = Read( ADDR_CiFIFOSTA + (CiFIFO_OFFSET * fifo) );
  if (status & 1) //FIFO has room for this message - immediately send it to hardware
  {
    addr = Read( ADDR_CiFIFOUA + (CiFIFO_OFFSET * fifo) );
    LoadFrameBuffer( addr + 0x400, newFrame );
    Write8(ADDR_CiFIFOCON + (CiFIFO_OFFSET * fifo) + 1, 3); //Set UINC and TX_Request
  }
  else //no room on hardware. Locally buffer in software
  {
    xQueueSend(txQueue[fifo], &newFrame, 0); //try to queue, do not wait if we can't.
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
    xQueueSendFromISR(callbackQueueMCP, &frame, 0);
    return;
	}
	else if (cbGeneral) 
	{
		frame.fid = 0xFF;
    xQueueSendFromISR(callbackQueueMCP, &frame, 0);
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
          xQueueSendFromISR(callbackQueueMCP, &frame, 0);
          return;
				}
				else if (thisListener->isCallbackActive(numFilters)) //global catch-all 
				{
          frame.fid = 0x80000000ul + (listenerPos << 24ul) + 0xFF;
					xQueueSendFromISR(callbackQueueMCP, &frame, 0);
          return;
				}
			}
		}
	}
	//if none of the callback types caught this frame then queue it in the buffer
  xQueueSendFromISR(rxQueue, &frame, NULL);
}
