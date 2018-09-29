//——————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2515
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//
//——————————————————————————————————————————————————————————————————————————————

#include "ACAN2515.h"

//——————————————————————————————————————————————————————————————————————————————
//   MCP2515 COMMANDS
//——————————————————————————————————————————————————————————————————————————————

static const byte RESET_COMMAND = 0xC0 ;
static const byte WRITE_COMMAND = 0x02 ;
static const byte READ_COMMAND  = 0x03 ;
static const byte BIT_MODIFY_COMMAND  = 0x05 ;
static const byte LOAD_TX_BUFFER_COMMAND = 0x40 ;
static const byte SEND_COMMAND = 0x80 ;
static const byte READ_FROM_RXB0SIDH_COMMAND = 0x90 ;
static const byte READ_FROM_RXB1SIDH_COMMAND = 0x94 ;
static const byte READ_STATUS_COMMAND = 0xA0 ;
static const byte RX_STATUS_COMMAND = 0xB0 ;

//——————————————————————————————————————————————————————————————————————————————
//   MCP2515 REGISTERS
//——————————————————————————————————————————————————————————————————————————————

static const byte BFPCTRL_REGISTER   = 0x0C ;
static const byte TXRTSCTRL_REGISTER = 0x0D ;
static const byte CANCTRL_REGISTER   = 0x0F ;
static const byte CNF3_REGISTER      = 0x28 ;
static const byte CNF2_REGISTER      = 0x29 ;
static const byte CNF1_REGISTER      = 0x2A ;
static const byte CANINTF_REGISTER   = 0x2C ;
static const byte RXB0CTRL_REGISTER  = 0x60 ;
static const byte RXB1CTRL_REGISTER  = 0x70 ;

//——————————————————————————————————————————————————————————————————————————————

ACAN2515::ACAN2515 (const byte inCS,
                    const byte inCLK,
                    const byte inSI,
                    const byte inSO,
                    const byte inIRQ) :
mCS (inCS),
mCLK (inCLK),
mSI (inSI),
mSO (inSO),
mIRQ (inIRQ),
mReceiveBuffer (NULL),
mReceiveBufferSize (0),
mReceiveBufferReadIndex (0),
mReceiveBufferWriteIndex (0),
mReceiveBufferCount (0),
mReceiveBufferPeakCount (0),
mTransmitBuffer (NULL),
mTransmitBufferSize (0),
mTransmitBufferReadIndex (0),
mTransmitBufferWriteIndex (0),
mTransmitBufferCount (0),
mTransmitBufferPeakCount (0) {
}

//——————————————————————————————————————————————————————————————————————————————

uint32_t ACAN2515::begin (const ACANSettings2515 & inSettings) {
  uint32_t errorCode = 0 ; // Ok be default
//----------------------------------- Configure ports
  pinMode (mCS,  OUTPUT) ;
  pinMode (mCLK, OUTPUT) ;
  pinMode (mSI,  OUTPUT) ;
  pinMode (mSO,  INPUT_PULLUP) ;
  pinMode (mIRQ, INPUT_PULLUP) ;
  digitalWrite (mCLK, LOW) ;  // CLK is low outside a command
  digitalWrite (mCS,  HIGH) ; // CS is high outside a command
  digitalWrite (mSI, LOW) ;
//----------------------------------- Send software reset to MCP2515
  delayMicroseconds (1) ;
  digitalWrite (mCS,  LOW) ;
  sendByte (RESET_COMMAND) ;
  delayMicroseconds (1) ;
  digitalWrite (mCS,  HIGH) ;
 //---
  delayMicroseconds (10) ;
//----------------------------------- Check if MCP2515 is accessible
  writeRegister (CNF1_REGISTER, 0x55) ;
  bool ok = readRegister (CNF1_REGISTER) == 0x55 ;
  if (ok) {
    writeRegister (CNF1_REGISTER, 0xAA) ;
    ok = readRegister (CNF1_REGISTER) == 0xAA ;
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
    mReceiveBuffer = new CANMessage [inSettings.mReceiveBufferSize] ;
    mReceiveBufferSize = inSettings.mReceiveBufferSize ;
    mReceiveBufferReadIndex = 0 ;
    mReceiveBufferWriteIndex = 0 ;
    mReceiveBufferCount = 0 ;
    mReceiveBufferPeakCount = 0 ;
  //----------------------------------- Allocate transmit buffer
    mTransmitBuffer = new CANMessage [inSettings.mTransmitBufferSize] ;
    mTransmitBufferSize = inSettings.mTransmitBufferSize ;
    mTransmitBufferReadIndex = 0 ;
    mTransmitBufferWriteIndex = 0 ;
    mTransmitBufferCount = 0 ;
    mTransmitBufferPeakCount = 0 ;
  //----------------------------------- Set CNF3, CNF2, CNF1 and CANINTE registers
    digitalWrite (mCS,  LOW) ;
    sendByte (WRITE_COMMAND) ;
    sendByte (CNF3_REGISTER) ;
  //--- Register CNF3:
  //  Bit 7: SOF
  //  bit 6 --> 0: No Wake-up Filter bit
  //  Bit 5-3: -
  //  Bit 2-0: PHSEG2 - 1
    const byte cnf3 =
      ((inSettings.mSignalOnCLKOUT_SOF_pin == ACAN2515SignalOnCLKOUT_SOF_pin::SOF) << 6) /* SOF */ |
      ((inSettings.mPhaseSegment2 - 1) << 0) /* PHSEG2 */
    ;
   sendByte (cnf3) ;
  //--- Register CNF2:
  //  Bit 7 --> 1: BLTMODE
  //  bit 6: SAM
  //  Bit 5-3: PHSEG1 - 1
  //  Bit 2-0: PRSEG - 1
    const byte cnf2 =
      0x80 /* BLTMODE */ |
      (inSettings.mTripleSampling << 6) /* SAM */ |
      ((inSettings.mPhaseSegment1 - 1) << 3) /* PHSEG1 */ |
      ((inSettings.mPropagationSegment - 1) << 0) /* PRSEG */
    ;
    sendByte (cnf2) ;
  //--- Register CNF1:
  //  Bit 7-6: SJW - 1
  //  Bit 5-0: BRP - 1
    const byte cnf1 =
      (inSettings.mSJW << 6) /* SJW */ |
      ((inSettings.mBitRatePrescaler - 1) << 0) /* BRP */
    ;
    sendByte (cnf1) ;
  //--- Register CANINTE: activate receive interrupts
  //  Bit 7 --> 0: MERRE
  //  Bit 6 --> 0: WAKIE
  //  Bit 5 --> 0: ERRIE
  //  Bit 4 --> 0: TX2IE
  //  Bit 3 --> 0: TX1IE
  //  Bit 2 --> 0: TX0IE
  //  Bit 1 --> 1: RX1IE
  //  Bit 0 --> 1: RX0IE
    sendByte (0x03) ;
    digitalWrite (mCS,  HIGH) ;
  //----------------------------------- Deactivate the RXnBF Pins (High Impedance State)
    writeRegister (BFPCTRL_REGISTER, 0) ;
  //----------------------------------- Set TXnRTS as inputs
    writeRegister (TXRTSCTRL_REGISTER, 0);
  //----------------------------------- Turn off filters => receive any message
    writeRegister (RXB0CTRL_REGISTER, 0x60) ;
    writeRegister (RXB1CTRL_REGISTER, 0x60) ;
  //----------------------------------- Reset device to requested mode
    byte mode = inSettings.mOneShotModeEnabled ? (1 << 3) : 0 ; ;
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
    writeRegister (CANCTRL_REGISTER, mode) ;
  }
//-----------------------------------
  return errorCode ;
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515::handleMessages (void) {
//--- Receive message ?
  const byte rxStatus = readRxStatus () ; // Bit 0: message in RXB0, bit 1: message in RXB1
  const bool received = (rxStatus & 0xC0) != 0 ;
  if (received) { // Message in RXB0 and / or RXB1
    CANMessage message ;
    const bool accessRXB0 = (rxStatus & 0x40) != 0 ;
    message.rtr = (rxStatus & 0x10) != 0 ;
    message.ext = (rxStatus & 0x08) != 0 ;
    delayMicroseconds (1) ;
    digitalWrite (mCS,  LOW) ;
    sendByte (accessRXB0 ? READ_FROM_RXB0SIDH_COMMAND : READ_FROM_RXB1SIDH_COMMAND) ;
  //--- SIDH
    message.id = readByte () ; // Read SIDH
  //--- SIDL
    const uint8_t sidl = readByte () ; // Read SIDL
    message.id <<= 3 ;
    message.id |= sidl >> 5 ;
    message.ext = (sidl & 0x08) != 0 ;
  //--- EID8
    const uint8_t eid8 = readByte () ; // Read EID8
    if (message.ext) {
      message.id <<= 8 ;
      message.id |= eid8 ;
    }
  //--- EID0
    const uint8_t eid0 = readByte () ; // Read EID0
    if (message.ext) {
      message.id <<= 8 ;
      message.id |= eid0 ;
    }
  //--- DLC
    const uint8_t dlc = readByte () ; // Read DLC
    message.len = dlc & 0x0F ;
  //--- Read data
    for (int i=0 ; i<message.len ; i++) {
      message.data [i] = readByte () ;
    }
  //---
    delayMicroseconds (1) ;
    digitalWrite (mCS,  HIGH) ;
  //--- Free receive buffer command
    bitModifyRegister (CANINTF_REGISTER, accessRXB0 ? 0x01 : 0x02, 0) ;
  //--- Enter received message in receive buffer (if not full)
    if (mReceiveBufferCount == mReceiveBufferSize) {
      mReceiveBufferPeakCount = mReceiveBufferSize + 1 ; // Receive buffer overflow
    }else{
      mReceiveBuffer [mReceiveBufferWriteIndex] = message ;
      mReceiveBufferWriteIndex += 1 ;
      if (mReceiveBufferWriteIndex == mReceiveBufferSize) {
        mReceiveBufferWriteIndex = 0 ;
      }
      mReceiveBufferCount += 1 ;
      if (mReceiveBufferPeakCount < mReceiveBufferCount) {
        mReceiveBufferPeakCount = mReceiveBufferCount ;
      }
    }
  }
//--- Send message ?
  if (mTransmitBufferCount > 0) {
    const bool sent = internalSendMessage (mTransmitBuffer [mTransmitBufferReadIndex]) ;
    if (sent) {
      mTransmitBufferCount -= 1 ;
      mTransmitBufferReadIndex += 1 ;
      if (mTransmitBufferReadIndex == mTransmitBufferSize) {
        mTransmitBufferReadIndex = 0 ;
      }
    }
  }
}

//——————————————————————————————————————————————————————————————————————————————

bool ACAN2515::available (void) {
  return mReceiveBufferCount > 0 ;
}

//——————————————————————————————————————————————————————————————————————————————

bool ACAN2515::getReceivedMessage (CANMessage & outMessage) {
  const bool hasReceivedMessage = mReceiveBufferCount > 0 ;
  if (hasReceivedMessage) {
    outMessage = mReceiveBuffer [mReceiveBufferReadIndex] ;
    mReceiveBufferCount -= 1 ;
    mReceiveBufferReadIndex += 1 ;
    if (mReceiveBufferReadIndex == mReceiveBufferSize) {
      mReceiveBufferReadIndex = 0 ;
    }
  }
//---
  return hasReceivedMessage ;
}

//——————————————————————————————————————————————————————————————————————————————

bool ACAN2515::internalSendMessage (const CANMessage & inFrame) {
//--- Get status (bit 2, 4 and 6 are related to send buffer 0, 1 and 2)
  const byte status = readStatus () ;
//--- Find a free send buffer
  bool ok = true ;
  byte loadTxBuffer = LOAD_TX_BUFFER_COMMAND ;
  byte sendCommand = SEND_COMMAND ;
  if ((status & 0x04) == 0) { // Send buffer 0 is free
    sendCommand |= 0x01 ;
  }else if ((status & 0x10) == 0) { // Send buffer 1 is free
    loadTxBuffer = LOAD_TX_BUFFER_COMMAND | 0x02 ;
    sendCommand |= 0x02 ;
  }else if ((status & 0x40) == 0) { // Send buffer 2 is free
    loadTxBuffer = LOAD_TX_BUFFER_COMMAND | 0x04 ;
    sendCommand |= 0x04 ;
  }else{
    ok = false ; // No free buffer
  }
//--- Send message if a free buffer has been found
  if (ok) {
    delayMicroseconds (1) ;
    digitalWrite (mCS,  LOW) ;
    sendByte (loadTxBuffer) ;
    if (inFrame.ext) { // Extended frame
      uint32_t v = inFrame.id >> 21 ;
      sendByte ((byte) v) ; // ID28 ... ID21 --> SIDH
      v  = (inFrame.id >> 13) & 0xE0 ; // ID20, ID19, ID18 in bits 7, 6, 5
      v |= (inFrame.id >> 16) & 0x03 ; // ID17, ID16 in bits 1, 0
      v |= 0x08 ; // Extended bit
      sendByte ((byte) v) ; // ID20, ID19, ID18, -, 1, -, ID17, ID16 --> SIDL
      v  = (inFrame.id >> 8) & 0xFF ; // ID15, ..., ID8
      sendByte ((byte) v) ; // ID15, ID14, ID13, ID12, ID11, ID10, ID9, ID8 --> EID8
      v  = inFrame.id & 0xFF ; // ID7, ..., ID0
      sendByte ((byte) v) ; // ID7, ID6, ID5, ID4, ID3, ID2, ID1, ID0 --> EID0
    }else{ // Standard frame
      uint32_t v = inFrame.id >> 3 ;
      sendByte ((byte) v) ; // ID10 ... ID3 --> SIDH
      v  = (inFrame.id << 5) & 0xE0 ; // ID2, ID1, ID0 in bits 7, 6, 5
      sendByte ((byte) v) ; // ID2, ID1, ID0, -, 0, -, 0, 0 --> SIDL
      sendByte (0x00) ; // any value --> EID8
      sendByte (0x00) ; // any value --> EID0
    }
  //--- DLC
    byte v = inFrame.len & 0x0F ;
    if (inFrame.rtr) {
      v |= 0x40 ;
    }
    sendByte (v) ;
  //--- Send data
    if (!inFrame.rtr) {
      for (unsigned i=0 ; i<inFrame.len ; i++) {
        sendByte (inFrame.data [i]) ;
      }
    }
    delayMicroseconds (1) ;
    digitalWrite (mCS,  HIGH) ;
  //--- Write send command
    delayMicroseconds (1) ;
    digitalWrite (mCS,  LOW) ;
    sendByte (sendCommand) ;
    delayMicroseconds (1) ;
    digitalWrite (mCS,  HIGH) ;
  }
//--- Return
  return ok ;
}

//——————————————————————————————————————————————————————————————————————————————

bool ACAN2515::tryToSend (const CANMessage & inMessage) {
  const bool ok = mTransmitBufferCount < mTransmitBufferSize ;
  if (ok) {
    mTransmitBuffer [mTransmitBufferWriteIndex] = inMessage ;
    mTransmitBufferWriteIndex += 1 ;
    if (mTransmitBufferWriteIndex == mTransmitBufferSize) {
      mTransmitBufferWriteIndex = 0 ;
    }
    mTransmitBufferCount += 1 ;
    if (mTransmitBufferPeakCount < mTransmitBufferCount) {
      mTransmitBufferPeakCount = mTransmitBufferCount ;
    }
  }
  return ok ;
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515::sendByte (const byte inByte) {
  byte v = inByte ;
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

void ACAN2515::writeRegister (const byte inRegister, const byte inValue) {
  delayMicroseconds (1) ;
  digitalWrite (mCS,  LOW) ;
  sendByte (WRITE_COMMAND) ;
  sendByte (inRegister) ;
  sendByte (inValue) ;
  delayMicroseconds (1) ;
  digitalWrite (mCS,  HIGH) ;
}

//——————————————————————————————————————————————————————————————————————————————

byte ACAN2515::readByte (void) {
  byte readValue = 0 ;
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

byte ACAN2515::readRegister (const byte inRegister) {
  delayMicroseconds (1) ;
  digitalWrite (mCS,  LOW) ;
  sendByte (READ_COMMAND) ;
  sendByte (inRegister) ;
  const byte readValue = readByte () ;
  delayMicroseconds (1) ;
  digitalWrite (mCS,  HIGH) ;
  return readValue ;
}

//——————————————————————————————————————————————————————————————————————————————

byte ACAN2515::readStatus (void) {
  delayMicroseconds (1) ;
  digitalWrite (mCS,  LOW) ;
  sendByte (READ_STATUS_COMMAND) ;
  const byte readValue = readByte () ;
  delayMicroseconds (1) ;
  digitalWrite (mCS,  HIGH) ;
  return readValue ;
}

//——————————————————————————————————————————————————————————————————————————————

byte ACAN2515::readRxStatus (void) {
  delayMicroseconds (1) ;
  digitalWrite (mCS,  LOW) ;
  sendByte (RX_STATUS_COMMAND) ;
  const byte readValue = readByte () ;
  delayMicroseconds (1) ;
  digitalWrite (mCS,  HIGH) ;
  return readValue ;
}

//——————————————————————————————————————————————————————————————————————————————

void ACAN2515::bitModifyRegister (const uint8_t inRegister,
                                  const uint8_t inMask,
                                  const uint8_t inData) {
  delayMicroseconds (1) ;
  digitalWrite (mCS,  LOW) ;
  sendByte (BIT_MODIFY_COMMAND) ;
  sendByte (inRegister) ;
  sendByte (inMask) ;
  sendByte (inData) ;
  delayMicroseconds (1) ;
  digitalWrite (mCS,  HIGH) ;
}

//——————————————————————————————————————————————————————————————————————————————

