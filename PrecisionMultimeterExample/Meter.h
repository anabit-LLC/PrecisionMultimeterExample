/*
 Meter.h
 
 Created: 6/12/2025
 Author: nforcier

 This class handles the multimeter functions of the precision meter
 It is essectially a wrapper for the ADC IC library
*/

#ifndef Meter_h
#define Meter_h

#include "definitions.h"
#include <ADS126X.h>

//thre purpose of this structure is to set all the precision meter settings that will be used while this sketch is running
//adjust this to set measurement mode, filter type, sample rate, etc
//please note not all sample rates can be used with all the filter settings. See datasheet table 9-13 for more information
struct PrecisionMeterSettings{ 
    uint8_t meterMode = VOLTS_DC; //sets mode DMM will run in
    uint8_t sampleRateSetting = ADS126X_RATE_2_5;    // Sample rate in samples per second
    uint8_t filterType = ADS126X_SINC4; //See library file ADS126X_definitions.h for constnat to set filter type
    uint8_t measRange = RANGE_2V; //range we start out in, before autorange. This is only applicable in DC volt mode
    uint8_t vRef = EXT_2048_VREF; //set internal voltage reference we plan to use
    bool choppingEnabled = true;           // Chopping Mode A (true = enabled)
    bool iDACRotation = false; //only applicable when making resistance measurements, increases IDAC accuracy but doubles measurement time
    //if true measurements will be made at the fastest rate based on sample time, filter type, and chop mode
    //If false measurements are made at rate of constant SLOW_MODE_DELAY in msec
    bool performCalibration = false;
    uint8_t cSelect = 10; // Arduino pin conected to CS on ADS126X
    uint8_t rng2V = 2;
    uint8_t rng20V = 3;
    uint8_t rng200V = 4;
};

class Meter
{
  public:
    Meter();
    void meterBegin(const PrecisionMeterSettings& meterConfig);
    static PrecisionMeterSettings defaultConfig();
    void initializeVoltageRangeControlPins(uint8_t pin2VRange, uint8_t pin20VRange, uint8_t pin200VRange);
    float makeMeterMeasurement();
    void discardFirstReadingDelayConTime();
    float conversionTime();
    uint8_t getMeasurementType();
    float scaleVoltMeas4Range(float vVal, uint8_t range);
    float getCurrentFromVoltage(float vVal, float sVal);
    float calculate2WireResistance(float measuredVoltage, float currentSource);
    void setADC1PGA(bool enable, uint8_t gain);
    void setsUpADCVoltRef(uint8_t voltRef);
    uint8_t setNewRange(uint8_t cRange, float vMeas); 
    void setVoltMeasurementRange(uint8_t newRange);
    uint8_t checkRangingState();
    void getMaxAndMinADCValues(uint32_t* aMax, uint32_t* aMin, uint16_t sRate, uint8_t sCnt);
    
  private:
    PrecisionMeterSettings _meterConfig;
    float calculateADCConversionTime();
    float getADCtoVoltage(int32_t aVal,float bits, uint8_t rVoltSetting);
    uint8_t rangeState = NO_RANGE_CHANGE;
    //if true measurements will be made at the fastest rate based on sample time, filter type, and chop mode
    //If false measurements are made at rate of constant SLOW_MODE_DELAY in msec
    float conversionDelay;

};

#endif