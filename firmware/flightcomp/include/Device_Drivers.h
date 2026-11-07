/**
 * @file Device_Drivers.h
 * @brief Drivers for the sensors the flight computer supports directly, in Device_Drivers.cpp.
 */
#pragma once

#include <Arduino.h>

bool beginLsm303Accel();
bool beginLsm303Mag();
void getLsm303Accel();
void getLsm303Mag();
bool beginL3gd20h();
void getL3gd20h();
bool beginLsm9ds1AccelGyro();
bool beginLsm9ds1Mag();
void getLsm9ds1AccelGyro();
void getLsm9ds1Mag();
bool beginLsm6ds3tr();
void getLsm6ds3tr();
bool beginLsm6ds33();
void getLsm6ds33();
bool beginLsm6dsox();
void getLsm6dsox();
bool beginMpu6050();
void getMpu6050();
bool beginMpu9250AccelGyro();
void getMpu9250AccelGyro();
bool beginMpu9250Mag();
void getMpu9250Mag();
bool beginLis3mdl();
void getLis3mdl();
bool beginLps25h();
void getLps25h();
bool beginAdxl377();
void getAdxl377();
bool beginH3lis331dl();
void getH3lis331dl();
bool beginAds1115();
void getAds1115();
boolean beginMpl3115a2();
void getMpl3115a2();
float pressureToAltitude(float seaLevel, float atmospheric);
bool beginBmp180();
void getBmp180();
void initiateTemp();
void initiatePressure();
void getPressure();
boolean beginBmp280();
void getBmp280();
boolean beginBmp388();
void getBmp388();
void cmdMs56xx(byte cmd);
bool beginMs56xx();
void getMs56xx();
float convertTempMs56xx();
float convertPressMs56xx();
