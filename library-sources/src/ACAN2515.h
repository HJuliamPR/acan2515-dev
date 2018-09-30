//——————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2515
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//
//——————————————————————————————————————————————————————————————————————————————

#pragma once

//——————————————————————————————————————————————————————————————————————————————

#include <ACANBuffer.h>
#include <ACAN2515Settings.h>

//——————————————————————————————————————————————————————————————————————————————

class ACAN2515 {
//--- Constructor: using soft SPI
  public: ACAN2515 (const uint8_t inCS,  // CS input of MCP2515
                    const uint8_t inCLK, // CLK input of MCP2515
                    const uint8_t inSI,  // SI input of MCP2515
                    const uint8_t inSO) ;  // SO output of MCP2515

//--- Initialisation: return 0 if ok, otherwise see error codes below
  public: uint32_t begin (const ACANSettings2515 & inSettings) ;

//--- Error codes returned by begin
  public: static const uint32_t kNoMCP2515           = 1 <<  0 ;
  public: static const uint32_t kInvalidSettings     = 1 <<  1 ;

//--- Receiving messages
  public: bool available (void) ;
  public: bool receive (CANMessage & outFrame) ;

//--- Transmitting messages
  public: bool tryToSend (const CANMessage & inMessage) ;

//--- Handling messages to send and receiving messages
  public: void polling (void) ;
  public: void isr (void) ;
  private: void handleTXBInterrupt (const uint8_t inTXB) ;
  private: void handleRXBInterrupt (void) ;

 //--- Properties
  public: const uint8_t mCS ;
  public: const uint8_t mCLK ;
  public: const uint8_t mSI ;
  public: const uint8_t mSO ;
  private: bool mTXB0IsFree ;

//--- Receive buffer
  private: ACANBuffer mReceiveBuffer ;

//--- Driver transmit buffer
  private: ACANBuffer mTransmitBuffer [3] ;
  private: bool mTXBIsFree [3] ;
  private: void internalSendMessage (const CANMessage & inFrame, const uint8_t inTXB) ;

  public: inline uint32_t transmitBufferSize (const uint8_t inIndex) const {
    return mTransmitBuffer [inIndex].size () ;
  }

  public: inline uint32_t transmitBufferCount (const uint8_t inIndex) const {
    return mTransmitBuffer [inIndex].count () ;
  }

  public: inline uint32_t transmitBufferPeakCount (const uint8_t inIndex) const {
    return mTransmitBuffer [inIndex].peakCount () ;
  }

//--- Private methods
  private: uint32_t internalBeginOperation (const ACANSettings2515 & inSettings) ;
  private: void write2515Register (const uint8_t inRegister, const uint8_t inValue) ;
  private: uint8_t read2515Register (const uint8_t inRegister) ;
  private: uint8_t read2515Status (void) ;
  private: uint8_t read2515RxStatus (void) ;
  private: void bitModify2515Register (const uint8_t inRegister, const uint8_t inMask, const uint8_t inData) ;
  private: void sendByte (const uint8_t inByte) ;
  private: uint8_t readByte (void) ;

//--- No Copy
  private: ACAN2515 (const ACAN2515 &) ;
  private: ACAN2515 & operator = (const ACAN2515 &) ;
} ;

//——————————————————————————————————————————————————————————————————————————————

