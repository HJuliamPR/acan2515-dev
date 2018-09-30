//——————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2515
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//
//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2515.h>
#include <SPI.h>

//——————————————————————————————————————————————————————————————————————————————
//   MCP2515 COMMANDS
//——————————————————————————————————————————————————————————————————————————————

static const uint8_t RESET_COMMAND = 0xC0 ;
static const uint8_t WRITE_COMMAND = 0x02 ;
static const uint8_t READ_COMMAND  = 0x03 ;
static const uint8_t BIT_MODIFY_COMMAND  = 0x05 ;
static const uint8_t LOAD_TX_BUFFER_COMMAND = 0x40 ;
static const uint8_t REQUEST_TO_SEND_COMMAND = 0x80 ;
static const uint8_t READ_FROM_RXB0SIDH_COMMAND = 0x90 ;
static const uint8_t READ_FROM_RXB1SIDH_COMMAND = 0x94 ;
static const uint8_t READ_STATUS_COMMAND = 0xA0 ;
static const uint8_t RX_STATUS_COMMAND = 0xB0 ;

//——————————————————————————————————————————————————————————————————————————————
//   MCP2515 REGISTERS
//——————————————————————————————————————————————————————————————————————————————

static const uint8_t BFPCTRL_REGISTER   = 0x0C ;
static const uint8_t TXRTSCTRL_REGISTER = 0x0D ;
static const uint8_t CANSTAT_REGISTER   = 0x0E ;
static const uint8_t CANCTRL_REGISTER   = 0x0F ;
static const uint8_t CNF3_REGISTER      = 0x28 ;
static const uint8_t CNF2_REGISTER      = 0x29 ;
static const uint8_t CNF1_REGISTER      = 0x2A ;
static const uint8_t CANINTF_REGISTER   = 0x2C ;
static const uint8_t RXB0CTRL_REGISTER  = 0x60 ;
static const uint8_t RXB1CTRL_REGISTER  = 0x70 ;

//——————————————————————————————————————————————————————————————————————————————
//   CONSTRUCTOR, HARDWARE SPI
//——————————————————————————————————————————————————————————————————————————————

ACAN2515::ACAN2515 (const uint8_t inCS,  // CS input of MCP2515
                    SPIClass & inSPI) : // Hardware SPI object
mCS (inCS),
mSPI (NULL),
mReceiveBuffer (),
mTransmitBuffer (),
mTXBIsFree () {
  mSPI = new ACAN2515HardSPI (inSPI, 10 * 1000 * 1000) ; // 10 MHz
}

//——————————————————————————————————————————————————————————————————————————————
//   CONSTRUCTOR, SOFTWARE SPI
//——————————————————————————————————————————————————————————————————————————————

ACAN2515::ACAN2515 (const uint8_t inCS,
                    const uint8_t inCLK,
                    const uint8_t inSI,
                    const uint8_t inSO) :
mCS (inCS),
mSPI (NULL),
mReceiveBuffer (),
mTransmitBuffer (),
mTXBIsFree () {
  mSPI = new ACAN2515SoftSPI (inCLK, inSI, inSO) ;
}

//——————————————————————————————————————————————————————————————————————————————
//   BEGIN
//——————————————————————————————————————————————————————————————————————————————

uint32_t ACAN2515::begin (const ACANSettings2515 & inSettings) {
  noInterrupts () ;
  //----------------------------------- Configure ports
    pinMode (mCS,  OUTPUT) ;
    digitalWrite (mCS,  HIGH) ; // CS is high outside a command
    mSPI->configure () ;
  //----------------------------------- Internal begin
    mSPI->beginTransaction () ;
      const uint32_t errorCode = internalBeginOperation (inSettings) ;
    mSPI->endTransaction () ;
  interrupts () ;
//----------------------------------- Return
  return errorCode ;
}

//——————————————————————————————————————————————————————————————————————————————
//   MESSAGE EMISSION
//——————————————————————————————————————————————————————————————————————————————

bool ACAN2515::tryToSend (const CANMessage & inMessage) {
//--- Find send buffer index
  uint8_t idx = inMessage.idx ;
  if (idx > 2) {
    idx = 0 ;
  }
//---
  noInterrupts () ;
    bool ok = mTXBIsFree [idx] ;
    if (ok) { // Transmit buffer and TXB are both free: transmit immediatly
      mTXBIsFree [idx] = false ;
      mSPI->beginTransaction () ;
        internalSendMessage (inMessage, idx) ;
      mSPI->endTransaction () ;
    }else{ // Enter in transmit buffer, if not full
      ok = mTransmitBuffer [idx].append (inMessage) ;
    }
  interrupts () ;
  return ok ;
}

//——————————————————————————————————————————————————————————————————————————————
//   MESSAGE RECEPTION
//——————————————————————————————————————————————————————————————————————————————

bool ACAN2515::available (void) {
  noInterrupts () ;
    const bool hasReceivedMessage = mReceiveBuffer.count () > 0 ;
  interrupts () ;
  return hasReceivedMessage ;
}

//——————————————————————————————————————————————————————————————————————————————

bool ACAN2515::receive (CANMessage & outMessage) {
  noInterrupts () ;
    const bool hasReceivedMessage = mReceiveBuffer.remove (outMessage) ;
  interrupts () ;
//---
  return hasReceivedMessage ;
}

//——————————————————————————————————————————————————————————————————————————————
//   POLLING
//——————————————————————————————————————————————————————————————————————————————

void ACAN2515::polling (void) {
  noInterrupts () ;
    isr () ;
  interrupts () ;
}

//——————————————————————————————————————————————————————————————————————————————
//  INTERRUPTS ARE DISABLED WHEN THESE FUNCTIONS ARE EXECUTED
//——————————————————————————————————————————————————————————————————————————————

uint32_t ACAN2515::internalBeginOperation (const ACANSettings2515 & inSettings) {
  uint32_t errorCode = 0 ; // Ok be default
//----------------------------------- Send software reset to MCP2515
  delayMicroseconds (1) ;
  digitalWrite (mCS,  LOW) ;
  mSPI->sendByte (RESET_COMMAND) ;
  delayMicroseconds (1) ;
  digitalWrite (mCS,  HIGH) ;
 //---
  delayMicroseconds (10) ;
//----------------------------------- Check if MCP2515 is accessible
  write2515Register (CNF1_REGISTER, 0x55) ;
  bool ok = read2515Register (CNF1_REGISTER) == 0x55 ;
  if (ok) {
    write2515Register (CNF1_REGISTER, 0xAA) ;
    ok = read2515Register (CNF1_REGISTER) == 0xAA ;
  }
  if (!ok) {
    errorCode = kNoMCP2515 ;
  }
//----------------------------------- If ok, check if settings are correct
  if (!inSettings.mBitSettingOk) {
    errorCode |= kInvalidSettings ;
  }
//----------------------------------- If ok, perform configuration
  if (errorCode == 0) {
  //----------------------------------- Allocate receive buffer
    mReceiveBuffer.initWithSize (inSettings.mReceiveBufferSize) ;
  //----------------------------------- Allocate transmit buffers
    mTransmitBuffer [0].initWithSize (inSettings.mTransmitBuffer0Size) ;
    mTransmitBuffer [1].initWithSize (inSettings.mTransmitBuffer1Size) ;
    mTransmitBuffer [2].initWithSize (inSettings.mTransmitBuffer1Size) ;
    mTXBIsFree [0] = true ;
    mTXBIsFree [1] = true ;
    mTXBIsFree [2] = true ;
  //----------------------------------- Set CNF3, CNF2, CNF1 and CANINTE registers
    digitalWrite (mCS,  LOW) ;
    mSPI->sendByte (WRITE_COMMAND) ;
    mSPI->sendByte (CNF3_REGISTER) ;
  //--- Register CNF3:
  //  Bit 7: SOF
  //  bit 6 --> 0: No Wake-up Filter bit
  //  Bit 5-3: -
  //  Bit 2-0: PHSEG2 - 1
    const uint8_t cnf3 =
      ((inSettings.mSignalOnCLKOUT_SOF_pin == ACAN2515SignalOnCLKOUT_SOF_pin::SOF) << 6) /* SOF */ |
      ((inSettings.mPhaseSegment2 - 1) << 0) /* PHSEG2 */
    ;
   mSPI->sendByte (cnf3) ;
  //--- Register CNF2:
  //  Bit 7 --> 1: BLTMODE
  //  bit 6: SAM
  //  Bit 5-3: PHSEG1 - 1
  //  Bit 2-0: PRSEG - 1
    const uint8_t cnf2 =
      0x80 /* BLTMODE */ |
      (inSettings.mTripleSampling << 6) /* SAM */ |
      ((inSettings.mPhaseSegment1 - 1) << 3) /* PHSEG1 */ |
      ((inSettings.mPropagationSegment - 1) << 0) /* PRSEG */
    ;
    mSPI->sendByte (cnf2) ;
  //--- Register CNF1:
  //  Bit 7-6: SJW - 1
  //  Bit 5-0: BRP - 1
    const uint8_t cnf1 =
      (inSettings.mSJW << 6) /* SJW */ |
      ((inSettings.mBitRatePrescaler - 1) << 0) /* BRP */
    ;
    mSPI->sendByte (cnf1) ;
  //--- Register CANINTE: activate interrupts
  //  Bit 7 --> 0: MERRE
  //  Bit 6 --> 0: WAKIE
  //  Bit 5 --> 0: ERRIE
  //  Bit 4 --> 1: TX2IE
  //  Bit 3 --> 1: TX1IE
  //  Bit 2 --> 1: TX0IE
  //  Bit 1 --> 1: RX1IE
  //  Bit 0 --> 1: RX0IE
    mSPI->sendByte (0x1F) ;
    digitalWrite (mCS,  HIGH) ;
  //----------------------------------- Deactivate the RXnBF Pins (High Impedance State)
    write2515Register (BFPCTRL_REGISTER, 0) ;
  //----------------------------------- Set TXnRTS as inputs
    write2515Register (TXRTSCTRL_REGISTER, 0);
  //----------------------------------- Turn off filters => receive any message
    write2515Register (RXB0CTRL_REGISTER, 0x60) ;
    write2515Register (RXB1CTRL_REGISTER, 0x60) ;
  //----------------------------------- Reset device to requested mode
    uint8_t mode = inSettings.mOneShotModeEnabled ? (1 << 3) : 0 ; ;
    switch (inSettings.mSignalOnCLKOUT_SOF_pin) {
    case ACAN2515SignalOnCLKOUT_SOF_pin::CLOCK :
      mode = 0x04 | 0x00 ;
      break ;
    case ACAN2515SignalOnCLKOUT_SOF_pin::CLOCK2 :
      mode |= 0x04 | 0x01 ;
      break ;
    case ACAN2515SignalOnCLKOUT_SOF_pin::CLOCK4 :
      mode |= 0x04 | 0x02 ;
      break ;
    case ACAN2515SignalOnCLKOUT_SOF_pin::CLOCK8 :
      mode |= 0x04 | 0x03 ;
      break ;
    case ACAN2515SignalOnCLKOUT_SOF_pin::SOF :
      mode |= 0x04 ;
      break ;
    case ACAN2515SignalOnCLKOUT_SOF_pin::HiZ :
      break ;
    }
    switch (inSettings.mRequestedMode) {
    case ACAN2515RequestedMode::NormalMode :
      break ;
    case ACAN2515RequestedMode::ListenOnlyMode :
      mode |= 0x03 << 5 ;
      break ;
    case ACAN2515RequestedMode::LoopBackMode :
      mode |= 0x02 << 5 ;
      break ;
    }
    write2515Register (CANCTRL_REGISTER, mode) ;
  }
//-----------------------------------
  return errorCode ;
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515::isr (void) {
  mSPI->beginTransaction () ;
  uint8_t itStatus = read2515Register (CANSTAT_REGISTER) & 0x0E ;
  while (itStatus != 0) {
    switch (itStatus) {
    case 0 : // No interrupt
      break ;
    case 1 << 1 : // Error interrupt
      break ;
    case 2 << 1 : // Wake-up interrupt
      break ;
    case 3 << 1 : // TXB0 interrupt
      handleTXBInterrupt (0) ;
      break ;
    case 4 << 1 : // TXB1 interrupt
      handleTXBInterrupt (1) ;
      break ;
    case 5 << 1 : // TXB2 interrupt
      handleTXBInterrupt (2) ;
      break ;
    case 6 << 1 : // RXB0 interrupt
    case 7 << 1 : // RXB1 interrupt
      handleRXBInterrupt () ;
      break ;
    }
    itStatus = read2515Register (CANSTAT_REGISTER) & 0x0E ;
  }
  mSPI->endTransaction () ;
}

//——————————————————————————————————————————————————————————————————————————————
// This function is called by ISR when a MCP2515 receive buffer becomes full

void ACAN2515::handleRXBInterrupt (void) {
//--- Receive message ?
  const uint8_t rxStatus = read2515RxStatus () ; // Bit 0: message in RXB0, bit 1: message in RXB1
  const bool received = (rxStatus & 0xC0) != 0 ;
  if (received) { // Message in RXB0 and / or RXB1
    CANMessage message ;
    const bool accessRXB0 = (rxStatus & 0x40) != 0 ;
    message.rtr = (rxStatus & 0x10) != 0 ;
    message.ext = (rxStatus & 0x08) != 0 ;
    delayMicroseconds (1) ;
    digitalWrite (mCS,  LOW) ;
    mSPI->sendByte (accessRXB0 ? READ_FROM_RXB0SIDH_COMMAND : READ_FROM_RXB1SIDH_COMMAND) ;
  //--- SIDH
    message.id = mSPI->readByte () ; // Read SIDH
  //--- SIDL
    const uint8_t sidl = mSPI->readByte () ; // Read SIDL
    message.id <<= 3 ;
    message.id |= sidl >> 5 ;
    message.ext = (sidl & 0x08) != 0 ;
  //--- EID8
    const uint8_t eid8 = mSPI->readByte () ; // Read EID8
    if (message.ext) {
      message.id <<= 8 ;
      message.id |= eid8 ;
    }
  //--- EID0
    const uint8_t eid0 = mSPI->readByte () ; // Read EID0
    if (message.ext) {
      message.id <<= 8 ;
      message.id |= eid0 ;
    }
  //--- DLC
    const uint8_t dlc = mSPI->readByte () ; // Read DLC
    message.len = dlc & 0x0F ;
  //--- Read data
    for (int i=0 ; i<message.len ; i++) {
      message.data [i] = mSPI->readByte () ;
    }
  //---
    delayMicroseconds (1) ;
    digitalWrite (mCS,  HIGH) ;
  //--- Free receive buffer command
    bitModify2515Register (CANINTF_REGISTER, accessRXB0 ? 0x01 : 0x02, 0) ;
  //--- Enter received message in receive buffer (if not full)
    mReceiveBuffer.append (message) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————
// This function is called by ISR when a MCP2515 transmit buffer becomes empty

void ACAN2515::handleTXBInterrupt (const uint8_t inTXB) { // inTXB value is 0, 1 or 2
//--- Acknowledge interrupt
  bitModify2515Register (CANINTF_REGISTER, 0x04 << inTXB, 0) ;
//--- Send an other message ?
  CANMessage message ;
  const bool ok = mTransmitBuffer [inTXB].remove (message) ;
  if (ok) {
    internalSendMessage (message, inTXB) ;
  }else{
    mTXBIsFree [inTXB] = true ;
  }
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515::internalSendMessage (const CANMessage & inFrame, const uint8_t inTXB) { // inTXB is 0, 1 or 2
//--- Send command
//      send via TXB0: 0x81
//      send via TXB1: 0x82
//      send via TXB2: 0x84
  const uint8_t sendCommand = REQUEST_TO_SEND_COMMAND | (1 << inTXB) ;
//--- Load TX buffer command
//      Load TXB0, start at TXB0SIDH: 0x40
//      Load TXB1, start at TXB1SIDH: 0x42
//      Load TXB2, start at TXB2SIDH: 0x44
  const uint8_t loadTxBuffer = LOAD_TX_BUFFER_COMMAND | (inTXB << 1) ;
//--- Send message
  delayMicroseconds (1) ;
  digitalWrite (mCS,  LOW) ;
  mSPI->sendByte (loadTxBuffer) ;
  if (inFrame.ext) { // Extended frame
    uint32_t v = inFrame.id >> 21 ;
    mSPI->sendByte ((uint8_t) v) ; // ID28 ... ID21 --> SIDH
    v  = (inFrame.id >> 13) & 0xE0 ; // ID20, ID19, ID18 in bits 7, 6, 5
    v |= (inFrame.id >> 16) & 0x03 ; // ID17, ID16 in bits 1, 0
    v |= 0x08 ; // Extended bit
    mSPI->sendByte ((uint8_t) v) ; // ID20, ID19, ID18, -, 1, -, ID17, ID16 --> SIDL
    v  = (inFrame.id >> 8) & 0xFF ; // ID15, ..., ID8
    mSPI->sendByte ((uint8_t) v) ; // ID15, ID14, ID13, ID12, ID11, ID10, ID9, ID8 --> EID8
    v  = inFrame.id & 0xFF ; // ID7, ..., ID0
    mSPI->sendByte ((uint8_t) v) ; // ID7, ID6, ID5, ID4, ID3, ID2, ID1, ID0 --> EID0
  }else{ // Standard frame
    uint32_t v = inFrame.id >> 3 ;
    mSPI->sendByte ((uint8_t) v) ; // ID10 ... ID3 --> SIDH
    v  = (inFrame.id << 5) & 0xE0 ; // ID2, ID1, ID0 in bits 7, 6, 5
    mSPI->sendByte ((uint8_t) v) ; // ID2, ID1, ID0, -, 0, -, 0, 0 --> SIDL
    mSPI->sendByte (0x00) ; // any value --> EID8
    mSPI->sendByte (0x00) ; // any value --> EID0
  }
//--- DLC
  uint8_t v = inFrame.len & 0x0F ;
  if (inFrame.rtr) {
    v |= 0x40 ;
  }
  mSPI->sendByte (v) ;
//--- Send data
  if (!inFrame.rtr) {
    for (unsigned i=0 ; i<inFrame.len ; i++) {
      mSPI->sendByte (inFrame.data [i]) ;
    }
  }
  delayMicroseconds (1) ;
  digitalWrite (mCS,  HIGH) ;
//--- Write send command
  delayMicroseconds (1) ;
  digitalWrite (mCS,  LOW) ;
  mSPI->sendByte (sendCommand) ;
  delayMicroseconds (1) ;
  digitalWrite (mCS,  HIGH) ;
}

//——————————————————————————————————————————————————————————————————————————————
//  INTERNAL SPI FUNCTIONS (INTERRUPTS ARE DISABLED WHEN THEY ARE EXECUTED)
//——————————————————————————————————————————————————————————————————————————————

void ACAN2515::write2515Register (const uint8_t inRegister, const uint8_t inValue) {
  delayMicroseconds (1) ;
  digitalWrite (mCS,  LOW) ;
  mSPI->sendByte (WRITE_COMMAND) ;
  mSPI->sendByte (inRegister) ;
  mSPI->sendByte (inValue) ;
  delayMicroseconds (1) ;
  digitalWrite (mCS,  HIGH) ;
}

//——————————————————————————————————————————————————————————————————————————————

uint8_t ACAN2515::read2515Register (const uint8_t inRegister) {
  delayMicroseconds (1) ;
  digitalWrite (mCS,  LOW) ;
  mSPI->sendByte (READ_COMMAND) ;
  mSPI->sendByte (inRegister) ;
  const uint8_t readValue = mSPI->readByte () ;
  delayMicroseconds (1) ;
  digitalWrite (mCS,  HIGH) ;
  return readValue ;
}

//——————————————————————————————————————————————————————————————————————————————

uint8_t ACAN2515::read2515Status (void) {
  delayMicroseconds (1) ;
  digitalWrite (mCS,  LOW) ;
  mSPI->sendByte (READ_STATUS_COMMAND) ;
  const uint8_t readValue = mSPI->readByte () ;
  delayMicroseconds (1) ;
  digitalWrite (mCS,  HIGH) ;
  return readValue ;
}

//——————————————————————————————————————————————————————————————————————————————

uint8_t ACAN2515::read2515RxStatus (void) {
  delayMicroseconds (1) ;
  digitalWrite (mCS,  LOW) ;
  mSPI->sendByte (RX_STATUS_COMMAND) ;
  const uint8_t readValue = mSPI->readByte () ;
  delayMicroseconds (1) ;
  digitalWrite (mCS,  HIGH) ;
  return readValue ;
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515::bitModify2515Register (const uint8_t inRegister,
                                      const uint8_t inMask,
                                      const uint8_t inData) {
  delayMicroseconds (1) ;
  digitalWrite (mCS,  LOW) ;
  mSPI->sendByte (BIT_MODIFY_COMMAND) ;
  mSPI->sendByte (inRegister) ;
  mSPI->sendByte (inMask) ;
  mSPI->sendByte (inData) ;
  delayMicroseconds (1) ;
  digitalWrite (mCS,  HIGH) ;
}

//——————————————————————————————————————————————————————————————————————————————
//  SOFTWARE SPI FUNCTIONS
//——————————————————————————————————————————————————————————————————————————————

ACAN2515SoftSPI::ACAN2515SoftSPI (const uint8_t inCLK, // CLK input of MCP2515
                                  const uint8_t inSI,  // SI input of MCP2515
                                  const uint8_t inSO) :  // SO output of MCP2515)
ACAN2515AbstractSPI (),
mCLK (inCLK),
mSI (inSI),
mSO (inSO) {
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515SoftSPI::configure (void) {
  pinMode (mCLK, OUTPUT) ;
  pinMode (mSI,  OUTPUT) ;
  pinMode (mSO,  INPUT_PULLUP) ;
  digitalWrite (mCLK, LOW) ;  // CLK is low outside a command
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515SoftSPI::sendByte (const uint8_t inByte) {
  uint8_t v = inByte ;
  for (int i=0 ; i<8 ; i++) {
    delayMicroseconds (1) ;
    digitalWrite (mSI, (v & 0x80) != 0) ;
    delayMicroseconds (1) ;
    digitalWrite (mCLK, HIGH) ;
    delayMicroseconds (1) ;
    digitalWrite (mCLK, LOW) ;
    v <<= 1 ;
  }
}

//——————————————————————————————————————————————————————————————————————————————

uint8_t ACAN2515SoftSPI::readByte (void) {
  uint8_t readValue = 0 ;
  for (int i=0 ; i<8 ; i++) {
    delayMicroseconds (1) ;
    readValue <<= 1 ;
    readValue |= digitalRead (mSO) ;
    delayMicroseconds (1) ;
    digitalWrite (mCLK, HIGH) ;
    delayMicroseconds (1) ;
    digitalWrite (mCLK, LOW) ;
  }
  return readValue ;
}

//——————————————————————————————————————————————————————————————————————————————
//  HARDWARE SPI FUNCTIONS
//——————————————————————————————————————————————————————————————————————————————

ACAN2515HardSPI::ACAN2515HardSPI (SPIClass & inSPI, // Hardware SPI object
                                  const uint32_t inSPISpeed) :  // in byte / s
ACAN2515AbstractSPI (),
mHardSPI (& inSPI),
mSPISettings (inSPISpeed, MSBFIRST, SPI_MODE0) {
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515HardSPI::configure (void) {
  mHardSPI->begin () ;
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515HardSPI::beginTransaction (void) {
  mHardSPI->beginTransaction (mSPISettings) ;
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515HardSPI::sendByte (const uint8_t inByte) {
  mHardSPI->transfer (inByte) ;
}

//——————————————————————————————————————————————————————————————————————————————

uint8_t ACAN2515HardSPI::readByte (void) {
  return mHardSPI->transfer (0) ;
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515HardSPI::endTransaction (void) {
  mHardSPI->endTransaction () ;
}

//——————————————————————————————————————————————————————————————————————————————
