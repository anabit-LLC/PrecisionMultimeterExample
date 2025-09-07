/*
 * definitions.h
 *
 * Created: 6/12/2025
 *  Author: nforcier
 * 
 * Purpose of this file is to define constants
 */ 

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

//ADC settings
#define ADC1_BITS (float)2147483647.0 //this is for 31 bits with the 32nd bit being signed
#define ADC2_BITS (float)8388608.0 //this is for 31 bits with the 32nd bit being signed
#define VREF_2048 (float)2.04777
#define VREF_INT (float)2.496 //internal ref voltage
//connector settings
#define METER_PLUS ADS126X_AIN8
#define METER_MINUS ADS126X_AIN9
#define SENSE_PLUS ADS126X_AIN5
#define SENSE_MINUS ADS126X_AIN6
#define AMPS_PLUS ADS126X_AIN3
#define AMPS_MINUS ADS126X_AIN2
//Range resistor constants
#define RATIO_20V_POS (float)0.1
#define RATIO_200V_POS (float)0.01
#define RATIO_20V_NEG (float)0.1
#define RATIO_200V_NEG (float)0.01
#define AMPS_SHUNT (float)0.1
#define MAX_CURRENT (float)2.1
#define VOLT_RANGE_MAX (float)2.03
#define VOLT_RANGE_MIN (float)0.2
//Used to define mode meter is in
#define VOLTS_DC 0
#define AMPS_DC 1
#define RES_2W 2
#define RES_4W 3
#define VOLTS_AC 4
//Volt ref
#define INT_2_5_VREF 0
#define EXT_2048_VREF 1
//Current range we are in
#define RANGE_200mV 0
#define RANGE_2V 1
#define RANGE_20V 2
#define RANGE_200V 3
#define RANGE_OVER 4
#define MEASUREMENT_DELAY 1500
//constants for state of autoranging
#define NO_RANGE_CHANGE 0
#define RANGE_CHANGING 1
#define OVER_RANGE 2

#endif /* DEFINITIONS_H_ */