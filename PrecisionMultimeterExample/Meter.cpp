#include "Meter.h"
#include <ADS126X.h>
#include "definitions.h"
#include "Arduino.h"

ADS126X adc; // start the class

//class constuctor
Meter::Meter() {  }

//this function sets up the ADC settings, some setting are based on the measurement mode we are in
//input arguments are filter type, sample rate, and mode meter is in
void Meter::meterBegin(const PrecisionMeterSettings& meterConfig) {
  _meterConfig = meterConfig;
  conversionDelay = calculateADCConversionTime(); //based on filter, sample rate, and chop mode calculated measurement conversion time
  initializeVoltageRangeControlPins(_meterConfig.rng2V,_meterConfig.rng20V,_meterConfig.rng200V); //initilize pins for controlling voltage measurement ranges
  adc.begin(_meterConfig.cSelect); //start ADC library
  adc.setContinuousMode(); //set ADC for cont conversion mode
  setsUpADCVoltRef(_meterConfig.vRef); //turn on internal ref and select between int and ext
  adc.setFilter(_meterConfig.filterType); //set ADC1 filter type ADS126X_SINC4
  adc.setRate(_meterConfig.sampleRateSetting); //sets ADC sample rate ADS126X_RATE_2_5
  adc.clearResetBit(); //clear reset bit if it is set
  adc.startADC1(); // start conversion on ADC1
  switch (_meterConfig.meterMode) {
    case VOLTS_DC: //set pins for volt measuremewnt mode
      setVoltMeasurementRange(_meterConfig.measRange); //set default range of meter
      delay(50); //give current source and ref time to settle
      //this is where we need to do calibration
      break;
    case AMPS_DC:
      setADC1PGA(true, ADS126X_GAIN_8); //enable / disable PGA and set gain
      break;
    case RES_2W: 
      setVoltMeasurementRange(RANGE_2V); //make sure signals are routed through 2V range
      adc.setIDAC1Pin(ADS126X_IDAC_AINCOM); //sets pin for IDAC1
      adc.setIDAC1Mag(ADS126X_IDAC_MAG_3000); //current value of IDAC1 in uA
      setADC1PGA(false, ADS126X_GAIN_1); //enable / disable PGA and set gain
      break;
    case RES_4W:
      //turn on current source and ADC2
      setADC1PGA(false, ADS126X_GAIN_1); //enable / disable PGA and set gain
      break;
    case VOLTS_AC:
      //turn on current source and ADC2
      setADC1PGA(false, ADS126X_GAIN_1); //enable / disable PGA and set gain
      break;
    default:
      //do nothing
      setADC1PGA(false, ADS126X_GAIN_1); //enable / disable PGA and set gain
      break;
  }
  
  //adc.calibrateSelfOffsetADC1();
  if(_meterConfig.choppingEnabled && _meterConfig.iDACRotation) adc.setChopMode(ADS126X_CHOP_3); //11 enables both chop mode and IDAC rotation
  else if(_meterConfig.choppingEnabled)  adc.setChopMode(ADS126X_CHOP_1); //01 enable chop mode, IDAC is off
  else if(_meterConfig.iDACRotation)  adc.setChopMode(ADS126X_CHOP_2); //10 enable IDAC rotation, not chop mode
  else adc.setChopMode(ADS126X_CHOP_0); //00 disable chop mode and IDAC rotation
  
}

//This function sets initial state of pins used to control range switches
//input arguments are the three pins for controlling reed relays for setting ranges
void Meter::initializeVoltageRangeControlPins(uint8_t pin2VRange, uint8_t pin20VRange, uint8_t pin200VRange) {
  pinMode(pin2VRange,OUTPUT);
  pinMode(pin20VRange,OUTPUT);
  pinMode(pin200VRange,OUTPUT);
  digitalWrite(pin2VRange,LOW);
  digitalWrite(pin20VRange,LOW); 
  digitalWrite(pin200VRange,LOW); 
}

//adjust measurement voltage value to match range we are in
//input arguments are measured voltage and range
//returns scaled voltage value
float Meter::scaleVoltMeas4Range(float vVal, uint8_t range) {
  float rangeFactor;    
  if(range == RANGE_200mV) rangeFactor = 8; 
  else if(range == RANGE_2V) rangeFactor = 1.0; //set multiplication factor for volt calculation based on range we are in
  else if(range == RANGE_20V) rangeFactor = RATIO_20V_POS;
  else rangeFactor = RATIO_200V_POS; 
  return vVal/rangeFactor; 
}

//This function returns a measurement for the whatever measurement function was defined by the settings function.
//It also sets the overrange flag if an overrange conditon is detected
float Meter::makeMeterMeasurement() {
  int32_t aDCReading;
  float valMeas;
  uint8_t rTemp;
  delay(conversionDelay);
  switch (_meterConfig.meterMode) {
    case VOLTS_DC: //set pins for volt measuremewnt mode
        aDCReading = adc.readADC1(METER_PLUS,METER_MINUS); // read the voltage
        //aDCReading = adc.readADC1(ADS126X_AIN5,ADS126X_AIN6); // read the voltage
        //Serial.print("ADC reading: "); Serial.println(aDCReading); // send voltage through serial
        valMeas = getADCtoVoltage(aDCReading,ADC1_BITS,_meterConfig.vRef); //get voltage from ADC reading
        //Serial.print("Range when before range check "); Serial.println(_meterConfig.measRange); // send voltage through serial
        rTemp = setNewRange(_meterConfig.measRange, valMeas); 
        //Serial.print("Range after range check "); Serial.println(rTemp); // send voltage through serial
        if(rTemp == RANGE_OVER) { //if true we are in overrange condition
          _meterConfig.measRange = rTemp; //set value of range
          rangeState = OVER_RANGE; //set overrange flag
          //Serial.println("Overrange, remove voltage immediately from meter input or risk damage");
           Serial.println("overrange condition");
        }
        else if(rTemp == _meterConfig.measRange) { //if true measurement is within range
           valMeas = scaleVoltMeas4Range(valMeas,_meterConfig.measRange); // send voltage through serial
            //Serial.println("No range change");
        }
        else { //if true there was a range changed
         // Serial.print("Volt range change to: "); Serial.println(rTemp); 
         // Serial.print("Old volt range: "); Serial.println(PrecMeterConfig.measRange); 
          _meterConfig.measRange = rTemp; //set value of range
          rangeState = RANGE_CHANGING; //set flag that a auto range change is taken place
           Serial.println("there was a range change");
          discardFirstReadingDelayConTime(); //stsrt ADC, delay, and burn first reading
        }
      break;
    case AMPS_DC:
      aDCReading = adc.readADC1(AMPS_PLUS,AMPS_MINUS); // read the voltage
      valMeas = getADCtoVoltage(aDCReading,ADC1_BITS,VREF_INT); //get voltage from ADC reading
      valMeas = getCurrentFromVoltage(valMeas, AMPS_SHUNT);
      if(valMeas > MAX_CURRENT) { //over current condition tell user to turn it off
        rangeState = OVER_RANGE; //set overrange flag
      }
      break;
    case RES_2W: 
      aDCReading = adc.readADC1(METER_PLUS,METER_MINUS); // read the voltage
      valMeas = getADCtoVoltage(aDCReading,ADC1_BITS,_meterConfig.vRef); //get voltage from ADC reading
      valMeas = calculate2WireResistance(valMeas,0.003); //calculated resistance
      //Serial.print("Calculated resistance: "); Serial.println(valMeas,4); // send voltage through serial
      break;
    case RES_4W:
      //turn on current source and ADC2
      aDCReading = adc.readADC1(METER_PLUS,METER_MINUS); // read the voltage
      break;
    case VOLTS_AC:
      uint32_t* aDCMax; 
      uint32_t* aDCMin;
      getMaxAndMinADCValues(aDCMax,aDCMin,38400,250);
      valMeas = getADCtoVoltage(*aDCMax,ADC1_BITS,_meterConfig.vRef); //get voltage from ADC reading
      break;
    default:
      aDCReading = adc.readADC1(METER_PLUS,METER_MINUS); // read the voltage
      break;
  }

  return valMeas; //return calculated measurement as float
}

//This function gets an array of ADC measurements and returns the max ADC value and the min ADC value
//input arguments are pointers to the max and min ADC values, sample rate setting of the ADC, and number of samples to take
void Meter::getMaxAndMinADCValues(uint32_t* aMax, uint32_t* aMin, uint16_t sRate, uint8_t sCnt) {
  float sPeriod = (1.0 / (float)sRate)*1000000;

  uint32_t temp = adc.readADC1(METER_PLUS,METER_MINUS); // read the voltage
  *aMax = temp;
  *aMin = temp;
  for(uint16_t j=1; j<sCnt; j++) {
    delayMicroseconds((uint32_t)sPeriod); //delay to let acquisition complete
    temp = adc.readADC1(METER_PLUS,METER_MINUS); // read the voltage
    if(temp > *aMax) *aMax = temp; //set newest max value
    if(temp < *aMin) *aMin = temp; //set newest min value
  }

}

//This function starts ADC1 (stops it first), delays for conversion time, 
//and clears buffer by disgarding first reading
//Input argument is ADC conversion time
void Meter::discardFirstReadingDelayConTime() {
  adc.stopADC1(); //stop ADC 1
  adc.startADC1(); // start conversion on ADC1
  delay(conversionDelay); //delay conversion time
  if(_meterConfig.meterMode == VOLTS_DC || _meterConfig.meterMode == RES_2W) adc.readADC1(METER_PLUS,METER_MINUS); //burn first reading to clear buffer
  else if(_meterConfig.meterMode == AMPS_DC) adc.readADC1(AMPS_PLUS,AMPS_MINUS); //burn first reading to clear buffer
  //else would be 4w measurement
}

//returns time to complete measurement conversion time in milliseconds
float Meter::conversionTime() { return conversionDelay; }

//returns measurement mode or type we are in
uint8_t Meter::getMeasurementType() { return _meterConfig.meterMode; }

//This function returns true if there was an overrange condition detected
//This function also clears the overrange flag when it is called
uint8_t Meter::checkRangingState() {
  uint8_t temp = rangeState; 
  rangeState = NO_RANGE_CHANGE;
  return temp;
}


//converts ADC reading to voltage, voltage is returned as float
//input arguments are measured ADC value, ADC resolution, ADC ref voltage, volt measurement range
float Meter::getADCtoVoltage(int32_t aVal,float bits, uint8_t rVoltSetting) {
  float vRef;
  if(rVoltSetting == INT_2_5_VREF) vRef = VREF_INT; //use internal ref value
  else vRef = VREF_2048;
  return (((float)aVal / bits) * vRef);
}

//converts measured voltage value to current based on shunt resistance
//input arguments are measured voltage and shunt value
//returns measured current
float Meter::getCurrentFromVoltage(float vVal, float sVal) {
  return ((vVal / 8) / sVal); 
}

//calculates resistance based on current source setting and measured voltage
//input arguments are measured voltage and current source setting
float Meter::calculate2WireResistance(float measuredVoltage, float currentSource) {
  return measuredVoltage / currentSource; 
}

//enables or disables PGA and sets gain value
//inputs arguments are enable/disable and sets gain
void Meter::setADC1PGA(bool enable, uint8_t gain) {
  if(!enable) adc.bypassPGA(); //do not use PGA
  else {
    adc.enablePGA(); //use PGA
    adc.setGain(gain); //set gain of PGA
  }
}

//sets ADC voltage reference, with two options:
//Internal whihc is 2.5V
//external which is 2.048V optional hardware on precision meter
void Meter::setsUpADCVoltRef(uint8_t voltRef) {
  adc.enableInternalReference(); //turn on internal reference, needed for IDACs
  if(voltRef == INT_2_5_VREF) adc.setReference(ADS126X_REF_NEG_INT, ADS126X_REF_POS_INT); //use internal ref
  else adc.setReference(ADS126X_REF_NEG_AIN1, ADS126X_REF_POS_AIN0); //AIN1 tied to analog ground and AIN0 to 2.048V ref
}

//evaluates latest volt measurement and changes range if needed
//returns the current range if no change or the new range
uint8_t Meter::setNewRange(uint8_t cRange, float vMeas) {
  if(vMeas < 0.0) vMeas = -1.0*vMeas; //change negative value to positive for range checks

  if(vMeas > VOLT_RANGE_MAX) { //if true we are in over range
    if(cRange == RANGE_200V || cRange == RANGE_OVER) {
      cRange = RANGE_OVER; //set range to over range
    }
    else {
      cRange++; //up the range
      setVoltMeasurementRange(cRange); //set new higher range
    }
  }
  else if(vMeas < VOLT_RANGE_MIN && cRange != RANGE_200mV) {
    cRange--; //up the range
    setVoltMeasurementRange(cRange); //set new higher range
  }

  return cRange;
}

//used to change pins that set voltage measurement range usng reed relays
//input argument is range you want to change to or the new range
void Meter::setVoltMeasurementRange(uint8_t newRange) {
  Serial.print("range: "); Serial.println(newRange);
  Serial.print("2V: "); Serial.println(_meterConfig.rng2V);
  Serial.print("20V: "); Serial.println(_meterConfig.rng20V);
  Serial.print("200V: "); Serial.println(_meterConfig.rng200V);
  if(newRange == RANGE_200mV) {
     Serial.println("range 200mV");
    setADC1PGA(true,ADS126X_GAIN_8); //enable PGA and set gain to 8
    digitalWrite(_meterConfig.rng2V,HIGH);
    digitalWrite(_meterConfig.rng20V,LOW); 
    digitalWrite(_meterConfig.rng200V,LOW);  
  }
  else if(newRange == RANGE_2V) {
    Serial.println("range 2V");
    setADC1PGA(false,ADS126X_GAIN_1); //make sure PGA is off
    digitalWrite(_meterConfig.rng2V,HIGH);
    digitalWrite(_meterConfig.rng20V,LOW); 
    digitalWrite(_meterConfig.rng200V,LOW);  
  }
  else if(newRange == RANGE_20V) {
    Serial.println("range 20V");
    setADC1PGA(false,ADS126X_GAIN_1); //make sure PGA is on
    digitalWrite(_meterConfig.rng2V,LOW);
    digitalWrite(_meterConfig.rng20V,HIGH); 
    digitalWrite(_meterConfig.rng200V,LOW);  
  }
  else {
    Serial.println("range 200V");
    setADC1PGA(false,ADS126X_GAIN_1); //make sure PGA is on
    digitalWrite(_meterConfig.rng2V,LOW);
    digitalWrite(_meterConfig.rng20V,LOW); 
    digitalWrite(_meterConfig.rng200V,HIGH);  
  }
}


// Function to compute conversion and settling time in milliseconds of ADC1
//input arguments include filter type (reg), data rate (reg), is chop mode enabled
//returns conversion time in milliseconds 
float Meter::calculateADCConversionTime() {
  uint8_t settlingFactor;
  float sampleRate;

  // Identify settling factor for each filter
  switch (_meterConfig.filterType) {
    case ADS126X_FIR:
      settlingFactor = 3;
      break;
    case ADS126X_SINC1:
      settlingFactor = 1;
      break;
    case ADS126X_SINC2:
      settlingFactor = 2;
      break;
    case ADS126X_SINC3:
      settlingFactor = 3;
      break;
    case ADS126X_SINC4:
      settlingFactor = 4;
      break;
    default:
      settlingFactor = 4; //invalid filter type so set to highest settling factor
      break;
  }

   //get sample rate value
  switch (_meterConfig.sampleRateSetting) {
    case ADS126X_RATE_2_5:
      sampleRate = 2.5;
      break;
    case ADS126X_RATE_5:
      sampleRate = 5.0;
      break;
    case ADS126X_RATE_10:
      sampleRate = 10.0;
      break;
    case ADS126X_RATE_16_6:
      sampleRate = 16.6;
      break;
    case ADS126X_RATE_20:
      sampleRate = 20.0;
      break;
    case ADS126X_RATE_50:
      sampleRate = 50.0;
      break;
    case ADS126X_RATE_60:
      sampleRate = 60.0;
      break;
    case ADS126X_RATE_100:
      sampleRate = 100.0;
      break;
    case ADS126X_RATE_400:
      sampleRate = 400.0;
      break;
    case ADS126X_RATE_1200:
      sampleRate = 1200.0;
      break;
    case ADS126X_RATE_2400:
      sampleRate = 2400.0;
      break;
    case ADS126X_RATE_4800:
      sampleRate = 4800.0;
      break;
    case ADS126X_RATE_7200:
      sampleRate = 7200.0;
      break;
    case ADS126X_RATE_14400: //note that for this sample rate and above ADC automatically sets sinc5 filter regardless of the filter register settings
      sampleRate = 14400.0;
      settlingFactor = 6; //sinc5 filter
      break;
    case ADS126X_RATE_19200:
      sampleRate = 19200.0;
      settlingFactor = 6; //sinc5 filter
      break;
    case ADS126X_RATE_38400:
      sampleRate = 38400.0;
      settlingFactor = 6; //sinc5 filter
      break;
    default:
      sampleRate = 2.5; //invalid filter type so set to highest settling factor
      break;
  }

  float Tconv_ms = 1000.0 / sampleRate;  // Base conversion period in ms

  // Calculate settling time
  //there is a ~400usec delay at any filter setting beyond the conversion rate
  Tconv_ms = (Tconv_ms * settlingFactor) + 1; //add one because most filters have a ~400usec delay

  // Double conversion period if chopping mode is enabled
  if (_meterConfig.choppingEnabled) {
    Tconv_ms *= 2.0;  // Chopping mode A doubles conversion time
  } 
  //double conversion period if IDAC rotation mode is enabled
  if (_meterConfig.iDACRotation) {
    Tconv_ms *= 2.0;  // Chopping mode A doubles conversion time
  } 
  return Tconv_ms; //return delay time between measurements
}