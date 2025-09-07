/*******************************************************************************************************************************
This Arduino sketch is meant to work with Anabit's Precision Multimeter product which features a 32 bit ADC that delivers
Benchtop DMM performance with measurement resolutions of 6.5+ digits. The Precision Multimeter supports DCV, ACV, 2-wire resistance,
4-wire resistance, DCI, ACI.

Product link:

This example sketch demonstrates how to make high accuracy and high resolution measurements with Anabit's Precision Multimeter.
This sketch can make DC voltage, resistance, and DC Current measurements. To use this sketch your arduino needs to support hardware 
SPI communication and will need three digital pins to control the input voltage range switches. 

Link to ADS126X.h library on Github: https://github.com/Molorius/ADS126X

Please report any issue with the sketch to the Anabit forum: https://anabit.co/community/forum/analog-to-digital-converters-adcs

Example code developed by Your Anabit LLC © 2025 Licensed under the Apache License, Version 2.0.
**********************************************************************************************************************************/
#include "Meter.h"
#include "definitions.h"

#define MEASURE_MODE VOLTS_DC

Meter mtr; //declare object for meter library
PrecisionMeterSettings mtrConfig; //struct for meter settings

void setup() {
  Serial.begin(115200);
  Serial.print("Precision Multimeter Initializing.....");
  
  //compiler directives for measurement mode and settings as well as display units
  #if MEASURE_MODE == VOLTS_DC
    mtrConfig.meterMode = VOLTS_DC; //sets mode DMM will run in
    mtrConfig.sampleRateSetting = ADS126X_RATE_2_5;    // Sample rate in samples per second
    mtrConfig.filterType = ADS126X_SINC4; //See library file ADS126X_definitions.h for constnat to set filter type
    mtrConfig.measRange = RANGE_2V; //range we start out in, before autorange. This is only applicable in DC volt mode
    mtrConfig.choppingEnabled = true;           // Chopping Mode A (true = enabled)
    mtrConfig.iDACRotation = false; //only applicable when making resistance measurements, increases IDAC accuracy but doubles measurement time
    //if true measurements will be made at the fastest rate based on sample time, filter type, and chop mode
    //If false measurements are made at rate of constant SLOW_MODE_DELAY in msec
    mtrConfig.performCalibration = false;
  #elif MEASURE_MODE == AMPS_DC
    
  #elif MEASURE_MODE == RES_2W
    
  #else
     //note that for sample rate 14400 and above ADC automatically sets sinc5 filter regardless of the filter register settings
    //mtr.meterMeasureSettings(VOLTS_AC, ADS126X_RATE_38400, ADS126X_SINC1, EXT_2048_VREF, false, false);
  #endif
  //settings that are common to all measurement modes
  mtrConfig.vRef = EXT_2048_VREF; //set internal voltage reference we plan to use
  mtrConfig.cSelect = 10; // Arduino pin conected to CS on ADS126X
  mtrConfig.rng2V = 2;
  mtrConfig.rng20V = 3;
  mtrConfig.rng200V = 4;
  //start ADC measurements with prev defined settings
  mtr.meterBegin(mtrConfig);

  for(int8_t a = 0; a<2; a++) { //we want to discard the first 2 ADC readings
    Serial.print(".....");
    mtr.discardFirstReadingDelayConTime(); 
  }
  Serial.println();
  Serial.println("Initilization complete");
 
}

void loop() {
  float measVal = mtr.makeMeterMeasurement(); 
  if(mtr.checkRangingState() == NO_RANGE_CHANGE) {
    Serial.print("Measured value: "); Serial.println(measVal,6); // send voltage through serial
  }
  else if(mtr.checkRangingState() == RANGE_CHANGING) Serial.println("Meter is changing range....");
  else Serial.println("Overrange, remove signal immediately from meter input or risk damage");
}