//——————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2515
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//
//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2515.h>

//——————————————————————————————————————————————————————————————————————————————
//   MCP2515 COMMANDS
//——————————————————————————————————————————————————————————————————————————————

static const uint8_t RESET_COMMAND = 0xC0 ;
static const uint8_t WRITE_COMMAND = 0x02 ;
static const uint8_t READ_COMMAND  = 0x03 ;
static const uint8_t BIT_MODIFY_COMMAND         = 0x05 ;
static const uint8_t LOAD_TX_BUFFER_COMMAND     = 0x40 ;
static const uint8_t REQUEST_TO_SEND_COMMAND    = 0x80 ;
static const uint8_t READ_FROM_RXB0SIDH_COMMAND = 0x90 ;
static const uint8_t READ_FROM_RXB1SIDH_COMMAND = 0x94 ;
static const uint8_t READ_STATUS_COMMAND        = 0xA0 ;
static const uint8_t RX_STATUS_COMMAND          = 0xB0 ;

//——————————————————————————————————————————————————————————————————————————————
//   MCP2515 REGISTERS
//——————————————————————————————————————————————————————————————————————————————

static const uint8_t RXF0SIDH_REGISTER  = 0x00 ;
static const uint8_t RXF1SIDH_REGISTER  = 0x04 ;
static const uint8_t RXF2SIDH_REGISTER  = 0x08 ;
static const uint8_t BFPCTRL_REGISTER   = 0x0C ;
static const uint8_t TXRTSCTRL_REGISTER = 0x0D ;
static const uint8_t CANSTAT_REGISTER   = 0x0E ;
static const uint8_t CANCTRL_REGISTER   = 0x0F ;
static const uint8_t RXF3SIDH_REGISTER  = 0x10 ;
static const uint8_t RXF4SIDH_REGISTER  = 0x14 ;
static const uint8_t RXF5SIDH_REGISTER  = 0x18 ;
static const uint8_t RXM0SIDH_REGISTER  = 0x20 ;
static const uint8_t RXM1SIDH_REGISTER  = 0x24 ;
static const uint8_t CNF3_REGISTER      = 0x28 ;
static const uint8_t CNF2_REGISTER      = 0x29 ;
static const uint8_t CNF1_REGISTER      = 0x2A ;
static const uint8_t CANINTF_REGISTER   = 0x2C ;
static const uint8_t TXB0CTRL_REGISTER  = 0x30 ;
static const uint8_t TXB1CTRL_REGISTER  = 0x40 ;
static const uint8_t TXB2CTRL_REGISTER  = 0x50 ;
static const uint8_t RXB0CTRL_REGISTER  = 0x60 ;
static const uint8_t RXB1CTRL_REGISTER  = 0x70 ;

//——————————————————————————————————————————————————————————————————————————————
//   CONSTRUCTOR, HARDWARE SPI
//——————————————————————————————————————————————————————————————————————————————

ACAN2515::ACAN2515 (const uint8_t inCS,  // CS input of MCP2515
                    SPIClass & inSPI, // Hardware SPI object
                    const uint8_t inINT) : // INT output of MCP2515
mSPI (&inSPI),
mSPISettings (10 * 1000 * 1000, MSBFIRST, SPI_MODE0),  // 10 MHz
mCS (inCS),
mINT (inINT),
mReceiveBuffer (),
mTransmitBuffer (),
mTXBIsFree () {
}

//——————————————————————————————————————————————————————————————————————————————
//   BEGIN
//——————————————————————————————————————————————————————————————————————————————

uint32_t ACAN2515::begin (const ACANSettings2515 & inSettings,
                          void (* inInterruptServiceRoutine) (void)) {
  uint32_t errorCode = 0 ; // Means no error
//----------------------------------- check mINT has interrupt capability
  const int8_t itPin = digitalPinToInterrupt (mINT) ;
  if (itPin == NOT_AN_INTERRUPT) {
    errorCode = kINTPinIsNotAnInterrupt ;
  }
//----------------------------------- check isr is not NULL
  if (inInterruptServiceRoutine == NULL) {
    errorCode |= kISRIsNull ;
  }
//----------------------------------- if no error, configure port and MCP2515
  if (errorCode == 0) {
  //--- Configure ports
    pinMode (mCS, OUTPUT) ;
    digitalWrite (mCS, HIGH) ;  // CS is high outside a command
    mSPI->begin () ;
  //--- Send software reset to MCP2515
    mSPI->beginTransaction (mSPISettings) ;
      select () ;
        mSPI->transfer (RESET_COMMAND) ;
      unselect () ;
    mSPI->endTransaction () ;
  //---
    delayMicroseconds (10) ;
  //--- Configure MCP2515_IRQ as external interrupt
    pinMode (mINT, INPUT_PULLUP) ;
    attachInterrupt (itPin, inInterruptServiceRoutine, LOW) ;
    mSPI->usingInterrupt (itPin) ;
  //--- Internal begin
    errorCode = internalBeginOperation (inSettings) ;
  }
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
  mSPI->beginTransaction (mSPISettings) ;
    bool ok = mTXBIsFree [idx] ;
    if (ok) { // Transmit buffer and TXB are both free: transmit immediatly
      mTXBIsFree [idx] = false ;
      internalSendMessage (inMessage, idx) ;
    }else{ // Enter in transmit buffer, if not full
      ok = mTransmitBuffer [idx].append (inMessage) ;
    }
  mSPI->endTransaction () ;
  return ok ;
}

//——————————————————————————————————————————————————————————————————————————————
//   MESSAGE RECEPTION
//——————————————————————————————————————————————————————————————————————————————

bool ACAN2515::available (void) {
  mSPI->beginTransaction (mSPISettings) ;
    const bool hasReceivedMessage = mReceiveBuffer.count () > 0 ;
  mSPI->endTransaction () ;
  return hasReceivedMessage ;
}

//——————————————————————————————————————————————————————————————————————————————

bool ACAN2515::receive (CANMessage & outMessage) {
  mSPI->beginTransaction (mSPISettings) ;
    const bool hasReceivedMessage = mReceiveBuffer.remove (outMessage) ;
  mSPI->endTransaction () ;
//---
  return hasReceivedMessage ;
}

//——————————————————————————————————————————————————————————————————————————————
//  INTERRUPTS ARE DISABLED WHEN THESE FUNCTIONS ARE EXECUTED
//——————————————————————————————————————————————————————————————————————————————

uint32_t ACAN2515::internalBeginOperation (const ACANSettings2515 & inSettings) {
  uint32_t errorCode = 0 ; // Ok be default
//----------------------------------- Check if MCP2515 is accessible
  mSPI->beginTransaction (mSPISettings) ;
    write2515Register (CNF1_REGISTER, 0x55) ;
    bool ok = read2515Register (CNF1_REGISTER) == 0x55 ;
    if (ok) {
      write2515Register (CNF1_REGISTER, 0xAA) ;
      ok = read2515Register (CNF1_REGISTER) == 0xAA ;
    }
    if (!ok) {
      errorCode = kNoMCP2515 ;
    }
  mSPI->endTransaction () ;
//----------------------------------- If ok, check if settings are correct
  if (!inSettings.mBitSettingOk) {
    errorCode |= kInvalidSettings ;
  }
//----------------------------------- If ok, perform configuration
  if (errorCode == 0) {
    mSPI->beginTransaction (mSPISettings) ;
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
    select () ;
    mSPI->transfer (WRITE_COMMAND) ;
    mSPI->transfer (CNF3_REGISTER) ;
  //--- Register CNF3:
  //  Bit 7: SOF
  //  bit 6 --> 0: No Wake-up Filter bit
  //  Bit 5-3: -
  //  Bit 2-0: PHSEG2 - 1
    const uint8_t cnf3 =
      ((inSettings.mSignalOnCLKOUT_SOF_pin == ACAN2515SignalOnCLKOUT_SOF_pin::SOF) << 6) /* SOF */ |
      ((inSettings.mPhaseSegment2 - 1) << 0) /* PHSEG2 */
    ;
   mSPI->transfer (cnf3) ;
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
    mSPI->transfer (cnf2) ;
  //--- Register CNF1:
  //  Bit 7-6: SJW - 1
  //  Bit 5-0: BRP - 1
    const uint8_t cnf1 =
      (inSettings.mSJW << 6) /* SJW */ |
      ((inSettings.mBitRatePrescaler - 1) << 0) /* BRP */
    ;
    mSPI->transfer (cnf1) ;
  //--- Register CANINTE: activate interrupts
  //  Bit 7 --> 0: MERRE
  //  Bit 6 --> 0: WAKIE
  //  Bit 5 --> 0: ERRIE
  //  Bit 4 --> 1: TX2IE
  //  Bit 3 --> 1: TX1IE
  //  Bit 2 --> 1: TX0IE
  //  Bit 1 --> 1: RX1IE
  //  Bit 0 --> 1: RX0IE
    mSPI->transfer (0x1F) ;
    unselect () ;
  //----------------------------------- Deactivate the RXnBF Pins (High Impedance State)
    write2515Register (BFPCTRL_REGISTER, 0) ;
  //----------------------------------- Set TXnRTS as inputs
    write2515Register (TXRTSCTRL_REGISTER, 0);
  //----------------------------------- RXBnCTRL
    write2515Register (RXB0CTRL_REGISTER, ((uint8_t) inSettings.mRolloverEnable) << 2) ;
    write2515Register (RXB1CTRL_REGISTER, 0x00) ;
  //----------------------------------- Setup mask registers
    setupMaskRegister (inSettings.mRXM0, RXM0SIDH_REGISTER) ;
    setupMaskRegister (inSettings.mRXM1, RXM1SIDH_REGISTER) ;
    setupMaskRegister (inSettings.mRXF0, RXF0SIDH_REGISTER) ;
    setupMaskRegister (inSettings.mRXF1, RXF1SIDH_REGISTER) ;
    setupMaskRegister (inSettings.mRXF2, RXF2SIDH_REGISTER) ;
    setupMaskRegister (inSettings.mRXF3, RXF3SIDH_REGISTER) ;
    setupMaskRegister (inSettings.mRXF4, RXF4SIDH_REGISTER) ;
    setupMaskRegister (inSettings.mRXF5, RXF5SIDH_REGISTER) ;
  //----------------------------------- Set TXBi priorities
    write2515Register (TXB0CTRL_REGISTER, inSettings.mTXBPriority & 3) ;
    write2515Register (TXB1CTRL_REGISTER, (inSettings.mTXBPriority >> 2) & 3) ;
    write2515Register (TXB2CTRL_REGISTER, (inSettings.mTXBPriority >> 4) & 3) ;
  //----------------------------------- Reset device to requested mode
    uint8_t canctrl = inSettings.mOneShotModeEnabled ? (1 << 3) : 0 ;
    switch (inSettings.mSignalOnCLKOUT_SOF_pin) {
    case ACAN2515SignalOnCLKOUT_SOF_pin::CLOCK :
      canctrl = 0x04 | 0x00 ;
      break ;
    case ACAN2515SignalOnCLKOUT_SOF_pin::CLOCK2 :
      canctrl |= 0x04 | 0x01 ;
      break ;
    case ACAN2515SignalOnCLKOUT_SOF_pin::CLOCK4 :
      canctrl |= 0x04 | 0x02 ;
      break ;
    case ACAN2515SignalOnCLKOUT_SOF_pin::CLOCK8 :
      canctrl |= 0x04 | 0x03 ;
      break ;
    case ACAN2515SignalOnCLKOUT_SOF_pin::SOF :
      canctrl |= 0x04 ;
      break ;
    case ACAN2515SignalOnCLKOUT_SOF_pin::HiZ :
      break ;
    }
  //--- Requested mode
    uint8_t requestedMode = 0 ;
    switch (inSettings.mRequestedMode) {
    case ACAN2515RequestedMode::NormalMode :
      break ;
    case ACAN2515RequestedMode::ListenOnlyMode :
      requestedMode = 0x03 << 5 ;
      break ;
    case ACAN2515RequestedMode::LoopBackMode :
      requestedMode = 0x02 << 5 ;
      break ;
    }
  //--- Request mode
    write2515Register (CANCTRL_REGISTER, canctrl | requestedMode) ;
    mSPI->endTransaction () ;
  //--- Wait until requested mode is reached (during 1 or 2 ms)
    bool wait = true ;
    const uint32_t deadline = millis () + 2 ;
    while (wait) {
      mSPI->beginTransaction (mSPISettings) ;
        const uint8_t actualMode = read2515Register (CANSTAT_REGISTER) & 0xE0 ;
      mSPI->endTransaction () ;
      wait = actualMode != requestedMode ;
      if (wait && (millis () >= deadline)) {
        errorCode |= kRequestedModeTimeOut ;
        wait = false ;
      }
    }
  }
//-----------------------------------
  return errorCode ;
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515::isr (void) {
  mSPI->beginTransaction (mSPISettings) ;
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
  const uint8_t rxStatus = read2515RxStatus () ; // Bit 6: message in RXB0, bit 7: message in RXB1
  const bool received = (rxStatus & 0xC0) != 0 ;
  if (received) { // Message in RXB0 and / or RXB1
    const bool accessRXB0 = (rxStatus & 0x40) != 0 ;
    CANMessage message ;
    message.rtr = (rxStatus & 0x10) != 0 ;
    message.ext = (rxStatus & 0x08) != 0 ;
    message.idx = rxStatus & 0x07 ;
    select () ;
    mSPI->transfer (accessRXB0 ? READ_FROM_RXB0SIDH_COMMAND : READ_FROM_RXB1SIDH_COMMAND) ;
  //--- SIDH
    message.id = mSPI->transfer (0) ;
  //--- SIDL
    const uint32_t sidl = mSPI->transfer (0) ;
    message.id <<= 3 ;
    message.id |= sidl >> 5 ;
  //--- EID8
    const uint32_t eid8 = mSPI->transfer (0) ;
    if (message.ext) {
      message.id <<= 2 ;
      message.id |= (sidl & 0x03) ;
      message.id <<= 8 ;
      message.id |= eid8 ;
    }
  //--- EID0
    const uint32_t eid0 = mSPI->transfer (0) ;
    if (message.ext) {
      message.id <<= 8 ;
      message.id |= eid0 ;
    }
  //--- DLC
    const uint8_t dlc = mSPI->transfer (0) ;
    message.len = dlc & 0x0F ;
  //--- Read data
    for (int i=0 ; i<message.len ; i++) {
      message.data [i] = mSPI->transfer (0) ;
    }
  //---
    unselect () ;
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
  const uint8_t loadTxBufferCommand = LOAD_TX_BUFFER_COMMAND | (inTXB << 1) ;
//--- Send message
  select () ;
  mSPI->transfer (loadTxBufferCommand) ;
  if (inFrame.ext) { // Extended frame
    uint32_t v = inFrame.id >> 21 ;
    mSPI->transfer ((uint8_t) v) ; // ID28 ... ID21 --> SIDH
    v  = (inFrame.id >> 13) & 0xE0 ; // ID20, ID19, ID18 in bits 7, 6, 5
    v |= (inFrame.id >> 16) & 0x03 ; // ID17, ID16 in bits 1, 0
    v |= 0x08 ; // Extended bit
    mSPI->transfer ((uint8_t) v) ; // ID20, ID19, ID18, -, 1, -, ID17, ID16 --> SIDL
    v  = (inFrame.id >> 8) & 0xFF ; // ID15, ..., ID8
    mSPI->transfer ((uint8_t) v) ; // ID15, ID14, ID13, ID12, ID11, ID10, ID9, ID8 --> EID8
    v  = inFrame.id & 0xFF ; // ID7, ..., ID0
    mSPI->transfer ((uint8_t) v) ; // ID7, ID6, ID5, ID4, ID3, ID2, ID1, ID0 --> EID0
  }else{ // Standard frame
    uint32_t v = inFrame.id >> 3 ;
    mSPI->transfer ((uint8_t) v) ; // ID10 ... ID3 --> SIDH
    v  = (inFrame.id << 5) & 0xE0 ; // ID2, ID1, ID0 in bits 7, 6, 5
    mSPI->transfer ((uint8_t) v) ; // ID2, ID1, ID0, -, 0, -, 0, 0 --> SIDL
    mSPI->transfer (0x00) ; // any value --> EID8
    mSPI->transfer (0x00) ; // any value --> EID0
  }
//--- DLC
  uint8_t v = inFrame.len ;
  if (v > 8) {
    v = 8 ;
  }
  if (inFrame.rtr) {
    v |= 0x40 ;
  }
  mSPI->transfer (v) ;
//--- Send data
  if (!inFrame.rtr) {
    for (unsigned i=0 ; i<inFrame.len ; i++) {
      mSPI->transfer (inFrame.data [i]) ;
    }
  }
  unselect () ;
//--- Write send command
  select () ;
    mSPI->transfer (sendCommand) ;
  unselect () ;
}

//——————————————————————————————————————————————————————————————————————————————
//  INTERNAL SPI FUNCTIONS
//——————————————————————————————————————————————————————————————————————————————

void ACAN2515::write2515Register (const uint8_t inRegister, const uint8_t inValue) {
  select () ;
    mSPI->transfer (WRITE_COMMAND) ;
    mSPI->transfer (inRegister) ;
    mSPI->transfer (inValue) ;
  unselect () ;
}

//——————————————————————————————————————————————————————————————————————————————

uint8_t ACAN2515::read2515Register (const uint8_t inRegister) {
  select () ;
    mSPI->transfer (READ_COMMAND) ;
    mSPI->transfer (inRegister) ;
    const uint8_t readValue = mSPI->transfer (0) ;
  unselect () ;
  return readValue ;
}

//——————————————————————————————————————————————————————————————————————————————

uint8_t ACAN2515::read2515Status (void) {
  select () ;
    mSPI->transfer (READ_STATUS_COMMAND) ;
    const uint8_t readValue = mSPI->transfer (0) ;
  unselect () ;
  return readValue ;
}

//——————————————————————————————————————————————————————————————————————————————

uint8_t ACAN2515::read2515RxStatus (void) {
  select () ;
    mSPI->transfer (RX_STATUS_COMMAND) ;
    const uint8_t readValue = mSPI->transfer (0) ;
  unselect () ;
  return readValue ;
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515::bitModify2515Register (const uint8_t inRegister,
                                      const uint8_t inMask,
                                      const uint8_t inData) {
  select () ;
    mSPI->transfer (BIT_MODIFY_COMMAND) ;
    mSPI->transfer (inRegister) ;
    mSPI->transfer (inMask) ;
    mSPI->transfer (inData) ;
  unselect () ;
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515::setupMaskRegister (const ACAN2515Mask inMask, const uint8_t inRegister) {
  select () ;
    mSPI->transfer (WRITE_COMMAND) ;
    mSPI->transfer (inRegister) ;
    mSPI->transfer (inMask.mSIDH) ;
    mSPI->transfer (inMask.mSIDL) ;
    mSPI->transfer (inMask.mEID8) ;
    mSPI->transfer (inMask.mEID0) ;
  unselect () ;
}

//——————————————————————————————————————————————————————————————————————————————
