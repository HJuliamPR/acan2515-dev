//——————————————————————————————————————————————————————————————————————————————
//  ACAN2515 Demo in loopback mode
//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2515.h>

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
volatile int x = 0 ;
void canISR (void) {
  x ++ ;
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
  Serial.println ("Configure ACAN2515") ;
  ACANSettings2515 settings (QUARTZ_FREQUENCY, 125 * 1000) ; // CAN bit rate 125 kb/s
  settings.mRequestedMode = ACAN2515RequestedMode::LoopBackMode ; // Select loopback mode
  const uint32_t errorCode = can.begin (settings) ;
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Propagation Segment: ") ;
    Serial.println (settings.mPropagationSegment) ;
    Serial.print ("Phase segment 1: ") ;
    Serial.println (settings.mPhaseSegment1) ;
    Serial.print ("Phase segment 2: ") ;
    Serial.println (settings.mPhaseSegment2) ;
    Serial.print ("SJW:") ;
    Serial.println (settings.mSJW) ;
    Serial.print ("Triple Sampling: ") ;
    Serial.println (settings.mTripleSampling ? "yes" : "no") ;
    Serial.print ("Actual bit rate: ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact bit rate ? ") ;
    Serial.println (settings.exactBitRate () ? "yes" : "no") ;
    Serial.print ("Sample point: ") ;
    Serial.print (settings.samplePointFromBitStart ()) ;
    Serial.println ("%") ;
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

static unsigned gBlinkLedDate = 0 ;
static unsigned gReceivedFrameCount = 0 ;
static unsigned gSentFrameCount = 0 ;
int xx = 0 ;
//——————————————————————————————————————————————————————————————————————————————

void loop() {
  if (xx < x) {
    xx = x ;
    Serial.println (xx) ;
  }
  CANMessage frame ;
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    const bool ok = can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentFrameCount) ;
    }else{
      Serial.println ("Send failure") ;
    }
  }
  if (can.available ()) {
    can.receive (frame) ;
    gReceivedFrameCount ++ ;
    Serial.print ("Received: ") ;
    Serial.println (gReceivedFrameCount) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————
