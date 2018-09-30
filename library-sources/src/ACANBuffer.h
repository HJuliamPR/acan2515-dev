//——————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2515
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//
//——————————————————————————————————————————————————————————————————————————————

#ifndef ACAN_BUFFER_CLASS_DEFINED
#define ACAN_BUFFER_CLASS_DEFINED

//——————————————————————————————————————————————————————————————————————————————

#include <CANMessage.h>

//——————————————————————————————————————————————————————————————————————————————

class ACANBuffer {
//--- Default constructor
  public: ACANBuffer (void) ;

//--- Properties
  private: CANMessage * mBuffer ;
  private: uint32_t mSize ;
  private: uint32_t mReadIndex ;
  private: uint32_t mWriteIndex ;
  private: uint32_t mCount ;
  private: uint32_t mPeakCount ; // > mSize if overflow did occur

//--- Accessors
  public: inline uint32_t size (void) const { return mSize ; }
  public: inline uint32_t count (void) const { return mCount ; }
  public: inline uint32_t peakCount (void) const { return mPeakCount ; }

//--- Public functions
  public: void initWithSize (const uint32_t inSize) ;
  public: bool append (const CANMessage & inMessage) ;
  public: bool remove (CANMessage & outMessage) ;

//--- No Copy
  private: ACANBuffer (const ACANBuffer &) ;
  private: ACANBuffer & operator = (const ACANBuffer &) ;
} ;

//——————————————————————————————————————————————————————————————————————————————

#endif

