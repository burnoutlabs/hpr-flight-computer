/**
 * @file Speed_Trig.h
 * @brief Fast table-based trigonometry in Speed_Trig.cpp.
 */
#pragma once

#include <Arduino.h>

// Angles are in tenths of a degree.
// speedTan() returns |tan|: it folds negative angles and angles past 90 degrees onto 0 to 90.
float speedTan(int rotn);
int speedArcTan(float myValue);
float speedSin(int degree);
int speedArcSin(float myValue);
float speedCos(int degree);
// 0 to 1800 (0 to 180 degrees)
int speedArcCos(float myValue);
int speedAtan2(float y, float x);
float cosSmallAngle(float rad);
float sinSmallAngle(float rad);
