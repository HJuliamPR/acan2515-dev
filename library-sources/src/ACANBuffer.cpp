//——————————————————————————————————————————————————————————————————————————————
// A CAN buffer
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan
// https://github.com/pierremolinaro/acan2515
//
//——————————————————————————————————————————————————————————————————————————————

#include <ACANBuffer.h>

//——————————————————————————————————————————————————————————————————————————————
//   DEFAULT CONSTRUCTOR
//——————————————————————————————————————————————————————————————————————————————

ACANBuffer::ACANBuffer (void) :
mBuffer (NULL),
mSize (0),
mReadIndex (0),
mWriteIndex (0),
mCount (0),
mPeakCount (0) {
}

//——————————————————————————————————————————————————————————————————————————————
//   INIT WITH SIZE
//——————————————————————————————————————————————————————————————————————————————

void ACANBuffer::initWithSize (const uint32_t inSize) {
  mBuffer = new CANMessage [inSize] ;
  mSize = inSize ;
  mReadIndex = 0 ;
  mWriteIndex = 0 ;
  mCount = 0 ;
  mPeakCount = 0 ;
}

//——————————————————————————————————————————————————————————————————————————————
//   APPEND
//——————————————————————————————————————————————————————————————————————————————

bool ACANBuffer::append (const CANMessage & inMessage) {
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

//——————————————————————————————————————————————————————————————————————————————
//   REMOVE
//——————————————————————————————————————————————————————————————————————————————

bool ACANBuffer::remove (CANMessage & outMessage) {
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

//——————————————————————————————————————————————————————————————————————————————
