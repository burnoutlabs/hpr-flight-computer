/**
 * @file Calibration.h
 * @brief Accelerometer, magnetometer, barometer and canard calibration routines in Calibration.cpp.
 */
#pragma once

#include <Arduino.h>

void accelCalibrate();
void magCalibrate();
void writeCalibration(int16_t inValue, byte eepromStart);
void setOrientation();
void exchangeAxes(char axisA, char axisB);
void flipAxis(char axis);
void readOrientation();
void baroCalibrate();
void setCanardTrim();
void wiggleServo(byte servo);
