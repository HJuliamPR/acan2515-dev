//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//
//  main.cpp
//
//  test-ACAN2515Settings-on-desktop
//
//  Created by Pierre Molinaro on 06/10/2018.
//  Copyright © 2018 Pierre Molinaro. All rights reserved.
//
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#include "ACAN2515Settings.h"
#include "Set.h"

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

#include <iostream>
using namespace std ;

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

static const uint32_t QUARTZ_FREQUENCY = 16 * 1000 * 1000 ;

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

static const uint32_t firstTestedBitRate = 1 ; // 1 bit/s
static const uint32_t lastTestedBitRate = 1000 * 1000 ; // 1 Mbit/s

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

static void compute (const uint32_t inDesiredBaudRate) {
  ACAN2515Settings settings (QUARTZ_FREQUENCY, inDesiredBaudRate) ;
  cout << "Desired baud rate: " << settings.mDesiredBitRate << " bit/s"  << endl ;
  cout << "  Divisor : " << (unsigned) settings.mBitRatePrescaler << endl ;
  cout << "  Prop seg: " << (unsigned) settings.mPropagationSegment << endl ;
  cout << "  Segment1: " << (unsigned) settings.mPhaseSegment1 << endl ;
  cout << "  Segment2: " << (unsigned) settings.mPhaseSegment2 << endl ;
  cout << "  SJW     : " << (unsigned) settings.mSJW << endl ;
  cout << "  Sampling: " << (settings.mTripleSampling ? "triple" : "single") << endl ;
  cout << "  Actual baud rate: " << settings.actualBitRate () << " bit/s" << endl ;
  cout << "  ppm: " << settings.ppmFromDesiredBitRate () << endl ;
  cout << "  Sample Point: " << settings.samplePointFromBitStart () << "%" << endl ;
  cout << "  Bit setting closed to desired bit rate ok: " << ((settings.ppmFromDesiredBitRate () < 1000) ? "yes" : "no") << endl ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

static void exploreAllSettings (void) {
  cout << "Explore all settings" << endl ;
  for (uint32_t br = firstTestedBitRate ; br <= lastTestedBitRate ; br ++) {
    ACAN2515Settings settings (QUARTZ_FREQUENCY, br) ;
    const uint32_t errorCode = settings.CANBitSettingConsistency () ;
    if (errorCode != 0) {
      cout << "Error 0x" << hex << errorCode << " for br : " << dec << br << endl ;
      if ((errorCode & ACAN2515Settings::kBitRatePrescalerIsZero) != 0) {
        cout << "  -> kBitRatePrescalerIsZero" << endl ;
      }
      if ((errorCode & ACAN2515Settings::kBitRatePrescalerIsGreaterThan64) != 0) {
        cout << "  -> kBitRatePrescalerIsGreaterThan64" << endl ;
      }
      if ((errorCode & ACAN2515Settings::kPropagationSegmentIsZero) != 0) {
        cout << "  -> kPropagationSegmentIsZero" << endl ;
      }
      if ((errorCode & ACAN2515Settings::kPropagationSegmentIsGreaterThan8) != 0) {
        cout << "  -> kPropagationSegmentIsGreaterThan8" << endl ;
      }
      if ((errorCode & ACAN2515Settings::kPhaseSegment1IsZero) != 0) {
        cout << "  -> kPhaseSegment1IsZero" << endl ;
      }
      if ((errorCode & ACAN2515Settings::kPhaseSegment1IsGreaterThan8) != 0) {
        cout << "  -> kPhaseSegment1IsGreaterThan8" << endl ;
      }
      if ((errorCode & ACAN2515Settings::kPhaseSegment2IsLowerThan2) != 0) {
        cout << "  -> kPhaseSegment2IsLowerThan2" << endl ;
      }
      if ((errorCode & ACAN2515Settings::kPhaseSegment2IsGreaterThan8) != 0) {
        cout << "  -> kPhaseSegment2IsGreaterThan8" << endl ;
      }
      if ((errorCode & ACAN2515Settings::kPhaseSegment1Is1AndTripleSampling) != 0) {
        cout << "  -> kPhaseSegment1Is1AndTripleSampling" << endl ;
      }
      if ((errorCode & ACAN2515Settings::kSJWIsZero) != 0) {
        cout << "  -> kSJWIsZero" << endl ;
      }
      if ((errorCode & ACAN2515Settings::kSJWIsGreaterThan4) != 0) {
        cout << "  -> kSJWIsGreaterThan4" << endl ;
      }
      if ((errorCode & ACAN2515Settings::kSJWIsGreaterThanOrEqualToPhaseSegment2) != 0) {
        cout << "  -> kSJWIsGreaterThanOrEqualToPhaseSegment2" << endl ;
      }
      if ((errorCode & ACAN2515Settings::kPhaseSegment2IsGreaterThanPSPlusPS1) != 0) {
        cout << "  -> kPhaseSegment2IsGreaterThanPSPlusPS1" << endl ;
      }
      cout << "  Divisor : " << (unsigned) settings.mBitRatePrescaler << endl ;
      cout << "  Prop seg: " << (unsigned) settings.mPropagationSegment << endl ;
      cout << "  Segment1: " << (unsigned) settings.mPhaseSegment1 << endl ;
      cout << "  Segment2: " << (unsigned) settings.mPhaseSegment2 << endl ;
      cout << "  SJW     : " << (unsigned) settings.mSJW << endl ;
      cout << "  Sampling: " << (settings.mTripleSampling ? "triple" : "single") << endl ;
      cout << "  Actual baud rate: " << settings.actualBitRate () << " bit/s" << endl ;
      cout << "  ppm: " << settings.ppmFromDesiredBitRate () << endl ;
      cout << "  Sample Point: " << settings.samplePointFromBitStart () << "%" << endl ;
      cout << "  Bit setting closed to desired bit rate ok: " << ((settings.ppmFromDesiredBitRate () < 1000) ? "yes" : "no") << endl ;
      exit (1) ;
    }
  }
  cout << "  Ok" << endl ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

static void allCorrectSettings (Set <uint32_t> & ioValidSettingSet) {
  cout << "All valid settings" << endl ;
  for (uint32_t br = firstTestedBitRate ; br <= lastTestedBitRate ; br ++) {
    ACAN2515Settings settings (QUARTZ_FREQUENCY, br) ;
    if (settings.mBitRateClosedToDesiredRate) {
      ioValidSettingSet.insert (br) ;
    }
  }
  cout << "  Completed, " << ioValidSettingSet.count () << " valid settings" << endl ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//  EXACT SETTINGS
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

static void allExactSettings (Set <uint32_t> & ioExactSettingSet) {
  cout << "All exact settings" << endl ;
  for (uint32_t br = firstTestedBitRate ; br <= lastTestedBitRate ; br ++) {
    ACAN2515Settings settings (QUARTZ_FREQUENCY, br, 0) ;
    if (settings.mBitRateClosedToDesiredRate) {
      ioExactSettingSet.insert (br) ;
    }
  }
  cout << "  Completed, " << ioExactSettingSet.count () << " exact settings" << endl ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

static void exhaustiveSearchOfAllExactSettings (Set <uint32_t> & ioExactSettingSet) {
  for (uint32_t brp = 1 ; brp <= 64 ; brp ++) {
    for (uint32_t TQCount = 5 ; TQCount <= 25 ; TQCount ++) {
      const uint32_t bitRate = QUARTZ_FREQUENCY / 2 / brp / TQCount ;
      const bool exact = (bitRate * brp * TQCount) == (QUARTZ_FREQUENCY / 2) ;
      if (exact && (bitRate <= 1000 * 1000)) {
        ioExactSettingSet.insert (bitRate) ;
      }
    }
  }
  cout << "  Exhaustive search completed, " << ioExactSettingSet.count () << " exact settings" << endl ;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————
//   MAIN
//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

int main (int /* argc */, const char * /* argv */ []) {
//  compute (250 * 1000) ;
//  compute (125 * 1000) ;
//  compute (500 * 1000) ;
//  compute (1000 * 1000) ;
//  compute (10 * 1000) ;
//  compute (842 * 1000) ;
  compute (440 * 1000) ;
//  compute (821 * 1000) ;
  compute (727 * 1000) ;
//  compute (2000) ;
//  compute (20 * 1000 * 1000) ;
//  compute (2509) ;
//--- Explore all settings
  exploreAllSettings () ;
//--- Check valid settings
  Set <uint32_t> validSettingSet ;
  allCorrectSettings (validSettingSet) ;
//--- Check all exact settings
  Set <uint32_t> exactSettingSet ;
  allExactSettings (exactSettingSet) ;
  Set <uint32_t> exhaustiveExactSettingSet ;
  exhaustiveSearchOfAllExactSettings (exhaustiveExactSettingSet) ;
  if (exactSettingSet != exhaustiveExactSettingSet) {
    cout << "  EQUALITY ERROR" << endl ;
    exit (1) ;
  }else{
    for (size_t i=0 ; i<exactSettingSet.count () ; i++) {
      const uint32_t bitRate = exactSettingSet.valueAtIndex (i) ;
      if ((bitRate % 1000) == 0) {
        cout << "  " << (bitRate / 1000) << " kbit/s" << endl ;
      }else{
        cout << "  " << bitRate << " bit/s" << endl ;
      }
    }
  }
  return 0;
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

