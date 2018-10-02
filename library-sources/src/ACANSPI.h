//——————————————————————————————————————————————————————————————————————————————
// A CAN driver for MCP2515
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan2515
//
// See https://dorkbotpdx.org/blog/paul/spi_transactions_in_arduino
//——————————————————————————————————————————————————————————————————————————————

#ifndef ACAN_SPI_CLASSES_DEFINED
#define ACAN_SPI_CLASSES_DEFINED

//——————————————————————————————————————————————————————————————————————————————

#include <SPI.h>

//——————————————————————————————————————————————————————————————————————————————
//   class ACANAbstractSPI
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
  //   Methods
  //···········································································

  public: virtual void configure (void) = 0 ;

  //···········································································

  public: virtual void beginTransaction (void) = 0 ;

  //···········································································

  public: virtual void select (void) = 0 ;

  //···········································································

  public: virtual void sendByte (const uint8_t inByte) = 0 ;

  //···········································································

  public: virtual uint8_t readByte (void) = 0 ;

  //···········································································

  public: virtual void unselect (void) = 0 ;

  //···········································································

  public: virtual void endTransaction (void) = 0 ;

  //···········································································
  // No Copy
  //···········································································

  private: ACANAbstractSPI (const ACANAbstractSPI &) ;
  private: ACANAbstractSPI & operator = (const ACANAbstractSPI &) ;
} ;

//——————————————————————————————————————————————————————————————————————————————
//   class ACANSoftSPI
//——————————————————————————————————————————————————————————————————————————————

class ACANSoftSPI : public ACANAbstractSPI {

  //···········································································
  //   Constructor
  //···········································································

  public : ACANSoftSPI (const uint8_t inCS,
                        const uint8_t inSCK,
                        const uint8_t inMOSI,
                        const uint8_t inMISO) :
  ACANAbstractSPI (),
  mCS (inCS),
  mSCK (inSCK),
  mMOSI (inMOSI),
  mMISO (inMISO) {
  }

  //···········································································
  //   Methods
  //···········································································

  public: virtual void configure (void) {
    pinMode (mCS, OUTPUT) ;
    digitalWrite (mCS, HIGH) ;  // CS is high outside a command
    pinMode (mSCK, OUTPUT) ;
    pinMode (mMOSI,  OUTPUT) ;
    pinMode (mMISO,  INPUT_PULLUP) ;
    digitalWrite (mSCK, LOW) ;  // SCK is low outside a command
  }

  //···········································································

  public: virtual void beginTransaction (void) {
  }

  //···········································································

  public: virtual void select (void) {
    digitalWrite (mCS, LOW) ;
  }

  //···········································································

  public: virtual void sendByte (const uint8_t inByte) {
    uint8_t v = inByte ;
    for (int i=0 ; i<8 ; i++) {
 //     delayMicroseconds (1) ;
      digitalWrite (mMOSI, (v & 0x80) != 0) ;
 //     delayMicroseconds (1) ;
      digitalWrite (mSCK, HIGH) ;
 //     delayMicroseconds (1) ;
      digitalWrite (mSCK, LOW) ;
      v <<= 1 ;
    }
  }

  //···········································································

  public: virtual uint8_t readByte (void) {
    uint8_t readValue = 0 ;
    for (int i=0 ; i<8 ; i++) {
  //    delayMicroseconds (1) ;
      readValue <<= 1 ;
      readValue |= digitalRead (mMISO) ;
  //    delayMicroseconds (1) ;
      digitalWrite (mSCK, HIGH) ;
  //    delayMicroseconds (1) ;
      digitalWrite (mSCK, LOW) ;
    }
    return readValue ;
  }

  //···········································································

  public: virtual void unselect (void) {
    digitalWrite (mCS, HIGH) ;
  }

  //···········································································

  public: virtual void endTransaction (void) {
  }

  //···········································································
  //   Properties
  //···········································································

  private: const uint8_t mCS ;
  private: const uint8_t mSCK ;
  private: const uint8_t mMOSI ;
  private: const uint8_t mMISO ;

  //···········································································
  // No Copy
  //···········································································

  private: ACANSoftSPI (const ACANSoftSPI &) ;
  private: ACANSoftSPI & operator = (const ACANSoftSPI &) ;
} ;

//——————————————————————————————————————————————————————————————————————————————
//   class ACANHardSPI
//——————————————————————————————————————————————————————————————————————————————

class ACANHardSPI : public ACANAbstractSPI {

  //···········································································
  //   Constructor
  //···········································································

  public : ACANHardSPI (const uint8_t inCS,
                        SPIClass & inSPI, // Hardware SPI object
                        const uint32_t inSPISpeed) : // in byte / s
  ACANAbstractSPI (),
  mHardSPI (& inSPI),
  mSPISettings (inSPISpeed, MSBFIRST, SPI_MODE0),
  mCS (inCS) {
  }

  //···········································································
  //   Methods
  //···········································································

  public: virtual void configure (void) {
    pinMode (mCS, OUTPUT) ;
    digitalWrite (mCS, HIGH) ;  // CS is high outside a command
    mHardSPI->begin () ;
  }

  //···········································································

  public: virtual void beginTransaction (void) {
    mHardSPI->beginTransaction (mSPISettings) ;
  }

  //···········································································

  public: virtual void select (void) {
    digitalWrite (mCS, LOW) ;
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

  public: virtual void unselect (void) {
    digitalWrite (mCS, HIGH) ;
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
  private: const uint8_t mCS ;

  //···········································································
  // No Copy
  //···········································································

  private: ACANHardSPI (const ACANHardSPI &) ;
  private: ACANHardSPI & operator = (const ACANHardSPI &) ;
} ;

//——————————————————————————————————————————————————————————————————————————————

#endif
