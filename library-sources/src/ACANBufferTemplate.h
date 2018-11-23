//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2515
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//
// This file is common with the acan2517 library
// https://github.com/pierremolinaro/acan2517
//
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#ifndef ACAN_BUFFER_TEMPLATE_CLASS_DEFINED
#define ACAN_BUFFER_TEMPLATE_CLASS_DEFINED

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#include <CANMessage.h>

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

template <typename T> class ACANBufferTemplate {

//······················································································································
// Default constructor
//······················································································································

  public: ACANBufferTemplate (void)  :
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

  public: ~ ACANBufferTemplate (void) {
    delete [] mBuffer ;
  }

//······················································································································
// Private properties
//······················································································································

  private: CANMessage * mBuffer ;
  private: T mSize ;
  private: T mReadIndex ;
//  private: T mWriteIndex ;
  private: T mCount ;
  private: T mPeakCount ; // == mSize+1 if overflow did occur

//······················································································································
// Accessors
//······················································································································

  public: inline T size (void) const { return mSize ; }
  public: inline T count (void) const { return mCount ; }
  public: inline T peakCount (void) const { return mPeakCount ; }

//······················································································································
// initWithSize
//······················································································································

  public: void initWithSize (const T inSize) {
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
      T writeIndex = mReadIndex + mCount ;
      if (writeIndex >= mSize) {
        writeIndex -= mSize ;
      }
      mBuffer [writeIndex] = inMessage ;
//       mBuffer [mWriteIndex] = inMessage ;
//       mWriteIndex += 1 ;
//       if (mWriteIndex == mSize) {
//         mWriteIndex = 0 ;
//       }
      mCount ++ ;
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
      mCount -= 1 ;
      outMessage = mBuffer [mReadIndex] ;
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

  private: ACANBufferTemplate (const ACANBufferTemplate &) ;
  private: ACANBufferTemplate & operator = (const ACANBufferTemplate &) ;
} ;

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#endif

