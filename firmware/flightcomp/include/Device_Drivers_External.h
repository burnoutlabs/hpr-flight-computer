/**
 * @file Device_Drivers_External.h
 * @brief Hooks for sensors driven by an external library, in Device_Drivers_External.cpp.
 */
#pragma once

#include <Arduino.h>

bool beginExternalAccel();
void getExternalAccel();
bool beginExternalGyro();
void getExternalGyro();
bool beginExternalMag();
void getExternalMag();
bool beginExternalHighG();
void getExternalHighG();
bool beginExternalBaro();
void getExternalBaro();
void externalGnssPowerSave();
void externalGnssConfig();
bool beginExternalRadio();
bool setExternalRadioFreq(float freq);
bool setExternalRadioPwr(int8_t pwr);
bool sendExternalRadioDataPkt(uint8_t *data, uint8_t len);
void externalRadioSleep();
