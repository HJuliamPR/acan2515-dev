//——————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2515
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//
//——————————————————————————————————————————————————————————————————————————————

#ifndef ACAN_SPI_CLASSES_DEFINED
#define ACAN_SPI_CLASSES_DEFINED

//——————————————————————————————————————————————————————————————————————————————

#include <SPI.h>

//——————————————————————————————————————————————————————————————————————————————

class ACANAbstractSPI {

  //···········································································
  //   Constructor
  //···········································································

  public : ACANAbstractSPI (void) {}

  //···········································································
  //   Virtual destructor
  //···········································································

  public : virtual ~ ACANAbstractSPI (void) {}

  //···········································································

  public: virtual void configure (void) = 0 ;

  //···········································································

  public: virtual void beginTransaction (void) = 0 ;

  //···········································································

  public: virtual void sendByte (const uint8_t inByte) = 0 ;

  //···········································································

  public: virtual uint8_t readByte (void) = 0 ;

  //···········································································

  public: virtual void endTransaction (void) = 0 ;

  //···········································································
  // No Copy
  //···········································································

  private: ACANAbstractSPI (const ACANAbstractSPI &) ;
  private: ACANAbstractSPI & operator = (const ACANAbstractSPI &) ;
} ;

//——————————————————————————————————————————————————————————————————————————————

class ACANSoftSPI : public ACANAbstractSPI {

  //···········································································
  //   Constructor
  //···········································································

  public : ACANSoftSPI (const uint8_t inCLK, // CLK input of MCP2515
                        const uint8_t inSI,  // SI input of MCP2515
                        const uint8_t inSO) :  // SO output of MCP2515)
  ACANAbstractSPI (),
  mCLK (inCLK),
  mSI (inSI),
  mSO (inSO) {
  }

  //···········································································

  public: virtual void configure (void) {
    pinMode (mCLK, OUTPUT) ;
    pinMode (mSI,  OUTPUT) ;
    pinMode (mSO,  INPUT_PULLUP) ;
    digitalWrite (mCLK, LOW) ;  // CLK is low outside a command
  }

  //···········································································

  public: virtual void beginTransaction (void) {
  }

  //···········································································

  public: virtual void sendByte (const uint8_t inByte) {
    uint8_t v = inByte ;
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

  //···········································································

  public: virtual uint8_t readByte (void) {
    uint8_t readValue = 0 ;
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

  //···········································································

  public: virtual void endTransaction (void) {
  }

  //···········································································
  //   Properties
  //···········································································

  public: const uint8_t mCLK ;
  public: const uint8_t mSI ;
  public: const uint8_t mSO ;

  //···········································································
  // No Copy
  //···········································································

  private: ACANSoftSPI (const ACANSoftSPI &) ;
  private: ACANSoftSPI & operator = (const ACANSoftSPI &) ;
} ;

//——————————————————————————————————————————————————————————————————————————————

class ACANHardSPI : public ACANAbstractSPI {

  //···········································································
  //   Constructor
  //···········································································

  public : ACANHardSPI (SPIClass & inSPI, // Hardware SPI object
                        const uint32_t inSPISpeed) : // in byte / s
  ACANAbstractSPI (),
  mHardSPI (& inSPI),
  mSPISettings (inSPISpeed, MSBFIRST, SPI_MODE0) {
  }

  //···········································································

  public: virtual void configure (void) {
    mHardSPI->begin () ;
  }

  //···········································································

  public: virtual void beginTransaction (void) {
    mHardSPI->beginTransaction (mSPISettings) ;
  }

  //···········································································

  public: virtual void sendByte (const uint8_t inByte) {
    mHardSPI->transfer (inByte) ;
  }

  //···········································································

  public: virtual uint8_t readByte (void) {
    return mHardSPI->transfer (0) ;
  }

  //···········································································

  public: virtual void endTransaction (void) {
    mHardSPI->endTransaction () ;
  }

  //···········································································
  //   Properties
  //···········································································

  private: SPIClass * mHardSPI ;
  private: const SPISettings mSPISettings ;

  //···········································································
  // No Copy
  //···········································································

  private: ACANHardSPI (const ACANHardSPI &) ;
  private: ACANHardSPI & operator = (const ACANHardSPI &) ;
} ;

//——————————————————————————————————————————————————————————————————————————————

#endif
