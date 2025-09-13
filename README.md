# PrecisionMultimeterExample
This Arduino sketch is meant to work with Anabit's Precision Multimeter product which features a 32 bit ADC that delivers
Benchtop DMM performance with measurement resolutions of 6.5+ digits. The Precision Multimeter supports DCV, ACV, 2-wire resistance,
4-wire resistance, DCI, ACI.

Product link: https://anabit.co/products/precision-multimeter-open-source

This example sketch demonstrates how to make high accuracy and high resolution measurements with Anabit's Precision Multimeter.
This sketch can make DC voltage, resistance, and DC Current measurements. To use this sketch your arduino needs to support hardware 
SPI communication and will need three digital pins to control the input voltage range switches. 

Link to ADS126X.h library on Github: https://github.com/Molorius/ADS126X

Please report any issue with the sketch to the Anabit forum: https://anabit.co/community/forum/analog-to-digital-converters-adcs

Example code developed by Your Anabit LLC © 2025 Licensed under the Apache License, Version 2.0.
