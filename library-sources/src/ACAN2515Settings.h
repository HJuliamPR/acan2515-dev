//——————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2515
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//
//——————————————————————————————————————————————————————————————————————————————

#pragma once

//——————————————————————————————————————————————————————————————————————————————

#include <stdint.h>

//——————————————————————————————————————————————————————————————————————————————

typedef enum {NormalMode, ListenOnlyMode, LoopBackMode} ACAN2515RequestedMode ;

typedef enum {CLOCK, CLOCK2, CLOCK4, CLOCK8, SOF, HiZ} ACAN2515SignalOnCLKOUT_SOF_pin ;

//——————————————————————————————————————————————————————————————————————————————

class ACAN2515Mask {

//--- Default constructor
  public: ACAN2515Mask (void) :
  mSIDH (0),
  mSIDL (0),
  mEID8 (0),
  mEID0 (0) {
  }

//--- Properties
  public: uint8_t mSIDH ;
  public: uint8_t mSIDL ;
  public: uint8_t mEID8 ;
  public: uint8_t mEID0 ;
} ;

//——————————————————————————————————————————————————————————————————————————————

class ACANSettings2515 {
//--- Constructor for a given baud rate
  public: explicit ACANSettings2515 (const uint32_t inQuartzFrequency, // In Hertz
                                     const uint32_t inWhishedBitRate,
                                     const uint32_t inTolerancePPM = 1000) ;

//--- CAN bit timing
  public: const uint32_t mQuartzFrequency ;
  public: uint32_t mWhishedBitRate = mQuartzFrequency / 64 ; // In kb/s
  public: uint8_t mPropagationSegment = 5 ; // 1...8
  public: uint8_t mPhaseSegment1 = 5 ; // 1...8
  public: uint8_t mPhaseSegment2 = 5 ;  // 2...8
  public: uint8_t mSJW = 4 ; // 1...4
  public: uint16_t mBitRatePrescaler = 32 / (1 + mPropagationSegment + mPhaseSegment1 + mPhaseSegment2) ; // 1...64
  public: bool mTripleSampling = false ; // true --> triple sampling, false --> single sampling
  public: bool mBitSettingOk = true ; // The above configuration is correct


//--- One shot mode
//      true --> Enabled; messages will only attempt to transmit one time
//      false --> Disabled; messages will reattempt transmission if required
  public: bool mOneShotModeEnabled = false ;


//--- MCP2515 TXBi priorities
//    bits 7-6: unused
//    bits 5-4: TXB2 priority
//    bits 3-2: TXB1 priority
//    bits 1-0: TXB0 priority
  public: uint8_t mTXBPriority = 0 ;


//--- Requested mode
  public: ACAN2515RequestedMode mRequestedMode = NormalMode ;


//--- Signal on CLKOUT/SOF pin
  public: ACAN2515SignalOnCLKOUT_SOF_pin mSignalOnCLKOUT_SOF_pin = CLOCK ;


//--- Rollover Enable Bit (is set to the BUKT bit of the RXB0CTRL register)
//       true  --> RXB0 message will roll over and be written to RXB1 if RXB0 is full
//       false --> Rollover is disabled
  public : bool mRolloverEnable = false ;


//--- Receive buffer size
  public: uint16_t mReceiveBufferSize = 32 ;


//--- Transmit buffer sizes
  public: uint16_t mTransmitBuffer0Size = 16 ;
  public: uint16_t mTransmitBuffer1Size = 0 ;
  public: uint16_t mTransmitBuffer2Size = 0 ;


//--- Receive acceptance masks
  public: ACAN2515Mask mRXM0 ;
  public: ACAN2515Mask mRXM1 ;
  public: static ACAN2515Mask standard2515Mask (const uint16_t inIdentifier,
                                                const uint8_t inByte0,
                                                const uint8_t inByte1) ;
  public: static ACAN2515Mask extended2515Mask (const uint32_t inIdentifier) ;


//--- Receive acceptance filters
  public: ACAN2515Mask mRXF0 ;
  public: ACAN2515Mask mRXF1 ;
  public: ACAN2515Mask mRXF2 ;
  public: ACAN2515Mask mRXF3 ;
  public: ACAN2515Mask mRXF4 ;
  public: ACAN2515Mask mRXF5 ;
  public: static ACAN2515Mask standard2515Filter (const uint16_t inIdentifier,
                                                  const uint8_t inByte0,
                                                  const uint8_t inByte1) ;
  public: static ACAN2515Mask extended2515Filter (const uint32_t inIdentifier) ;

//--- Compute actual bit rate
  public: uint32_t actualBitRate (void) const ;

//--- Exact bit rate ?
  public: bool exactBitRate (void) const ;

//--- Distance between actual bit rate and requested bit rate (in ppm, part-per-million)
  public: uint32_t ppmFromWishedBitRate (void) const ;

//--- Distance of sample point from bit start (in ppc, part-per-cent, denoted by %)
  public: uint32_t samplePointFromBitStart (void) const ;

//--- Bit settings are consistent ? (returns 0 if ok)
  public: uint32_t CANBitSettingConsistency (void) const ;

//--- Constants returned by CANBitSettingConsistency
  public: static const uint32_t kBitRatePrescalerIsZero           = 1 <<  0 ;
  public: static const uint32_t kBitRatePrescalerIsGreaterThan64  = 1 <<  1 ;
  public: static const uint32_t kPropagationSegmentIsZero         = 1 <<  2 ;
  public: static const uint32_t kPropagationSegmentIsGreaterThan8 = 1 <<  3 ;
  public: static const uint32_t kPhaseSegment1IsZero              = 1 <<  4 ;
  public: static const uint32_t kPhaseSegment1IsGreaterThan8      = 1 <<  5 ;
  public: static const uint32_t kPhaseSegment2IsLowerThan2        = 1 <<  6 ;
  public: static const uint32_t kPhaseSegment2IsGreaterThan8      = 1 <<  7 ;
  public: static const uint32_t kSJWIsZero                        = 1 <<  8 ;
  public: static const uint32_t kSJWIsGreaterThan4                = 1 <<  9 ;
  public: static const uint32_t kSJWIsGreaterThanOrEqualToPhaseSegment2 = 1 << 10 ;
  public: static const uint32_t kPhaseSegment2IsGreaterThanPSPlusPS1    = 1 <<  11 ;
} ;

//——————————————————————————————————————————————————————————————————————————————

