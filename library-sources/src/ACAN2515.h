//——————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2515
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//
//——————————————————————————————————————————————————————————————————————————————

#pragma once

//——————————————————————————————————————————————————————————————————————————————

#include <CANMessage.h>
#include "ACAN2515Settings.h"

//——————————————————————————————————————————————————————————————————————————————

class ACAN2515 {
//--- Constructor: using soft SPI
  public: ACAN2515 (const byte inCS,  // CS input of MCP2515
                    const byte inCLK, // CLK input of MCP2515
                    const byte inSI,  // SI input of MCP2515
                    const byte inSO,  // SO output of MCP2515
                    const byte inIRQ) ; // IRQ output of MCP2515

//--- Initialisation: return 0 if ok, otherwise see error codes below
  public: uint32_t begin (const ACANSettings2515 & inSettings) ;

//--- Error codes returned by begin
  public: static const uint32_t kNoMCP2515           = 1 <<  0 ;
  public: static const uint32_t kInvalidSettings     = 1 <<  1 ;

//--- Receiving messages
  public: bool available (void) ;
  public: bool getReceivedMessage (CANMessage & outFrame) ;

//--- Transmitting messages
  public: bool tryToSend (const CANMessage & inMessage) ;
  public: inline uint32_t transmitBufferSize (void) const { return mTransmitBufferSize ; }
  public: inline uint32_t transmitBufferCount (void) const { return mTransmitBufferCount ; }
  public: inline uint32_t transmitBufferPeakCount (void) const { return mTransmitBufferPeakCount ; }

//--- Handling messages to send and receiving messages
  public: void handleMessages (void) ;

 //--- Properties
  public: const byte mCS ;
  public: const byte mCLK ;
  public: const byte mSI ;
  public: const byte mSO ;
  public: const byte mIRQ ;

//--- Receive buffer
  private: CANMessage * volatile mReceiveBuffer = NULL ;
  private: uint32_t mReceiveBufferSize = 0 ;
  private: uint32_t mReceiveBufferReadIndex = 0 ; // Only used in user mode --> no volatile
  private: uint32_t mReceiveBufferWriteIndex = 0 ; // Only used in isr --> no volatile
  private: volatile uint32_t mReceiveBufferCount = 0 ; // Used in isr and user mode --> volatile
  private: volatile uint32_t mReceiveBufferPeakCount = 0 ; // == mReceiveBufferSize if overflow did occur

//--- Driver transmit buffer
  private: CANMessage * volatile mTransmitBuffer = NULL ;
  private: uint32_t mTransmitBufferSize = 0 ;
  private: uint32_t mTransmitBufferReadIndex = 0 ; // Only used in isr --> no volatile
  private: uint32_t mTransmitBufferWriteIndex = 0 ; // Only used in user mode --> no volatile
  private: volatile uint32_t mTransmitBufferCount = 0 ; // Used in isr and user mode --> volatile
  private: volatile uint32_t mTransmitBufferPeakCount = 0 ; // == mTransmitBufferSize if tentative overflow did occur
  private: bool internalSendMessage (const CANMessage & inFrame) ;

//--- Private methods
  private: void sendByte (const byte inByte) ;
  private: byte readByte (void) ;
  private: void writeRegister (const byte inRegister, const byte inValue) ;
  private: byte readRegister (const byte inRegister) ;
  private: byte readStatus (void) ;
  private: byte readRxStatus (void) ;
  private: void bitModifyRegister (const uint8_t inRegister, const uint8_t inMask, const uint8_t inData) ;

//--- No Copy
  private: ACAN2515 (const ACAN2515 &) ;
  private: ACAN2515 & operator = (const ACAN2515 &) ;
} ;

//——————————————————————————————————————————————————————————————————————————————

