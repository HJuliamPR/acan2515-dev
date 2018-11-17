//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2515
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#ifndef ACAN_TINY_BUFFER_CLASS_DEFINED
#define ACAN_TINY_BUFFER_CLASS_DEFINED

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#include <CANMessage.h>

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

class ACANBufferTiny {

//······················································································································
// Default constructor
//······················································································································

  public: ACANBufferTiny (void)  :
  mBuffer (NULL),
  mSize (0),
  mReadIndex (0),
//  mWriteIndex (0),
  mCount (0),
  mPeakCount (0) {
  }

//······················································································································
// Destructor
//······················································································································

  public: ~ ACANBufferTiny (void) {
    delete [] mBuffer ;
  }

//······················································································································
// Private properties
//······················································································································

  private: CANMessage * mBuffer ;
  private: uint8_t mSize ;
  private: uint8_t mReadIndex ;
//  private: uint8_t mWriteIndex ;
  private: uint8_t mCount ;
  private: uint8_t mPeakCount ; // > mSize if overflow did occur

//······················································································································
// Accessors
//······················································································································

  public: inline uint8_t size (void) const { return mSize ; }
  public: inline uint8_t count (void) const { return mCount ; }
  public: inline uint8_t peakCount (void) const { return mPeakCount ; }

//······················································································································
// initWithSize
//······················································································································

  public: void initWithSize (const uint8_t inSize) {
    mBuffer = new CANMessage [inSize] ;
    mSize = inSize ;
    mReadIndex = 0 ;
//    mWriteIndex = 0 ;
    mCount = 0 ;
    mPeakCount = 0 ;
  }

//······················································································································
// append
//······················································································································

  public: bool append (const CANMessage & inMessage) {
    const bool ok = mCount < mSize ;
    if (ok) {
      uint16_t writeIndex = mReadIndex + mCount ;
      if (writeIndex >= mSize) {
        writeIndex -= mSize ;
      }
      mBuffer [writeIndex] = inMessage ;
//       mBuffer [mWriteIndex] = inMessage ;
//       mWriteIndex += 1 ;
//       if (mWriteIndex == mSize) {
//         mWriteIndex = 0 ;
//       }
      mCount += 1 ;
      if (mPeakCount < mCount) {
        mPeakCount = mCount ;
      }
    }
    return ok ;
  }

//······················································································································
// Remove
//······················································································································

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

//······················································································································
// No copy
//······················································································································

  private: ACANBufferTiny (const ACANBufferTiny &) ;
  private: ACANBufferTiny & operator = (const ACANBufferTiny &) ;
} ;

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#endif

