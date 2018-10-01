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
  public: ACANBuffer (void)  :
  mBuffer (NULL),
  mSize (0),
  mReadIndex (0),
  mWriteIndex (0),
  mCount (0),
  mPeakCount (0) {
  }

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
  public: void initWithSize (const uint32_t inSize) {
    mBuffer = new CANMessage [inSize] ;
    mSize = inSize ;
    mReadIndex = 0 ;
    mWriteIndex = 0 ;
    mCount = 0 ;
    mPeakCount = 0 ;
  }

  public: bool append (const CANMessage & inMessage) {
    const bool ok = mCount < mSize ;
    if (ok) {
      mBuffer [mWriteIndex] = inMessage ;
      mWriteIndex += 1 ;
      if (mWriteIndex == mSize) {
        mWriteIndex = 0 ;
      }
      mCount ++ ;
      if (mPeakCount < mCount) {
        mPeakCount = mCount ;
      }
    }
    return ok ;
  }

  public: bool remove (CANMessage & outMessage) {
    const bool ok = mCount > 0 ;
    if (ok) {
      outMessage = mBuffer [mReadIndex] ;
      mCount -= 1 ;
      mReadIndex += 1 ;
      if (mReadIndex == mSize) {
        mReadIndex = 0 ;
      }
    }
    return ok ;
  }

//--- No Copy
  private: ACANBuffer (const ACANBuffer &) ;
  private: ACANBuffer & operator = (const ACANBuffer &) ;
} ;

//——————————————————————————————————————————————————————————————————————————————

#endif

