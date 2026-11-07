/**
 * @file flightcomp.h
 * @brief The flight loop, and the beeper, settings and timer helpers in flightcomp.cpp.
 */
#pragma once

#include <Arduino.h>
#include <PWMServo.h>
#include <TinyGPS++.h>

// Set code version
const float CODE_VERSION = 4.72;

// GPS Setup (defined in flightcomp.cpp)
extern TinyGPSPlus gps;

// Servo Setup (defined in flightcomp.cpp)
// Servos 1-8: servos[0] and [1] are the yaw canards, [2] and [3] the pitch canards, [4] to [7]
// the action servos
extern PWMServo servos[8];

void loop(void);
void parseBeep(uint32_t value, uint8_t array[], uint8_t arrayLen);
void clearIrq();
void timerBuildPkt();
void timerSyncPkt();
