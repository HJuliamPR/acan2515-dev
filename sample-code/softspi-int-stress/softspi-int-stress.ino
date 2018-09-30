//——————————————————————————————————————————————————————————————————————————————
//  ACAN2515 / ACAN Demo 
//  ACAN2515 uses soft SPI and an external interrupt pin
//  This sketch runs only on a Teensy 3.x
//  It uses the Teensy 3.x builtin CAN0 interface for testing intensive
//  communication with a MCP2515 CAN controller.
//  The builtin CAN0 interface and the MCP2515 controller should be connected
//  throught transceivers.
//  Note that the Tx and Rx alternate pins are used for the Tennsy builtin CAN0.
//——————————————————————————————————————————————————————————————————————————————

#include <ACAN.h>      // For the Teensy 3.x builtin CAN
#include <ACAN2515.h>  // For the external MCP2515

//——————————————————————————————————————————————————————————————————————————————
// Select CAN baud rate: soft spi is not very efficient, baud rate
// of 125 kb/s or more fail.
// Select a baud rate common to the builtin CAN interface and the MCP2515

static const uint32_t CAN_BIT_RATE = 62500 ;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 connections: adapt theses settings to your design
//  As Soft SPI is used (SPI is emulated with digital pins), you can use any
//  digital pins.
//  User code should configure MCP2515_IRQ pin as external interrupt
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2515_CS  = 20 ; // CS input of MCP2515 
static const byte MCP2515_CLK = 27 ; // CLK input of MCP2515 
static const byte MCP2515_SI  = 28 ; // SI input of MCP2515  
static const byte MCP2515_SO  = 39 ; // SO output of MCP2515 
static const byte MCP2515_IRQ = 37 ; // INT output of MCP2515

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Quartz: adapt to your design
//——————————————————————————————————————————————————————————————————————————————

static const uint32_t QUARTZ_FREQUENCY = 16 * 1000 * 1000 ; // 16 MHz

//——————————————————————————————————————————————————————————————————————————————

ACAN2515 can (MCP2515_CS, MCP2515_CLK, MCP2515_SI, MCP2515_SO) ;

//——————————————————————————————————————————————————————————————————————————————

void canISR (void) {
  can.isr () ;
}

//——————————————————————————————————————————————————————————————————————————————

void setup () {
//--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
//--- Start serial
  Serial.begin (38400) ;
//--- Wait for serial (blink led à 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
//--- Configure MCP2515_IRQ as external input
  pinMode (MCP2515_IRQ, INPUT_PULLUP) ;
  attachInterrupt (digitalPinToInterrupt (MCP2515_IRQ), canISR, LOW) ;
//--- Configure ACAN2515
  ACANSettings2515 settings2515 (QUARTZ_FREQUENCY, CAN_BIT_RATE) ;
  const uint32_t errorCode2515 = can.begin (settings2515) ;
  if (errorCode2515 == 0) {
    Serial.println ("ACAN2515 configuration: ok") ;
  }else{
    Serial.print ("ACAN2515 configuration error 0x") ;
    Serial.println (errorCode2515, HEX) ;
  }
//--- Configure ACAN
  ACANSettings settings (CAN_BIT_RATE) ;
  settings.mUseAlternateTxPin = true ;
  settings.mUseAlternateRxPin = true ;
  const uint32_t errorCode = ACAN::can0.begin (settings) ;
  if (errorCode == 0) {
    Serial.println ("ACAN configuration: ok") ;
  }else{
    Serial.print ("ACAN configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————

static unsigned gBlinkLedDate = 0 ;
static unsigned gReceivedFrameCount = 0 ;
static unsigned gReceivedFrameCount2515 = 0 ;
static unsigned gSentFrameCount = 0 ;
static unsigned gSentFrameCount2515 = 0 ;

static const unsigned MESSAGE_COUNT = 10 * 1000 ;

//——————————————————————————————————————————————————————————————————————————————
// A CAN network requires that stations do not send frames with the same identifier.
// So: 
//   - MCP2515 sends frame with even identifier values;
//   - builtin CAN0 sends frame with odd identifier values;

void loop () {
//--- Blink led
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 500 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print (gSentFrameCount) ;
    Serial.print (" ") ;
    Serial.print (gReceivedFrameCount) ;
    Serial.print (" ") ;
    Serial.print (gSentFrameCount2515) ;
    Serial.print (" ") ;
    Serial.println (gReceivedFrameCount2515) ;
  }
  CANMessage frame ;
//--- Send messages via the MCP2515
  if (gSentFrameCount2515 < MESSAGE_COUNT) {
  //--- Make an even identifier for MCP2515
    frame.id = millis () & 0x7FE ;
  //--- Send frame via MCP2515
    const bool ok = can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount2515 += 1 ;
    }
  }
//--- Send messages via the builtin CAN0
  if (gSentFrameCount < MESSAGE_COUNT) {
  //--- Make an odd identifier for builtin CAN0
    frame.id = millis () & 0x7FE ;
    frame.id |= 1 ;
  //--- Send frame via builtin CAN0
    const bool ok = ACAN::can0.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
    }
  }
//--- Receive frame via MCP2515
  if (can.available ()) {
    can.receive (frame) ;
    gReceivedFrameCount2515 ++ ;
  }
//--- Receive frame via builtin CAN0
  if (ACAN::can0.available ()) {
    ACAN::can0.receive (frame) ;
    gReceivedFrameCount ++ ;
  }
}

//——————————————————————————————————————————————————————————————————————————————
