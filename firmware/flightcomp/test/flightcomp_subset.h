/**
 * @file flightcomp_subset.h
 * @brief Compiles the real globals.cpp, Speed_Trig.cpp and Rotation.cpp sources for the host.
 *
 * The shims in test/shim stand in for the Teensy core and libraries, and the source files
 * are included as they are, not copied, so they build here with their own headers from
 * include/. Include it from exactly one .cpp per test suite: it defines globals.
 *
 * Define __IMXRT1062__ before including it to select the Teensy 4.x (double) path of
 * getQuatRotn(); leave it undefined for the Teensy 3.x (float) path.
 */
#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <PWMServo.h>
#include <TinyGPS++.h>

#include "../src/globals.cpp"

// Objects that flightcomp.cpp defines and Rotation.cpp uses.
TinyGPSPlus gps;
PWMServo servos[8];

#include "../src/Speed_Trig.cpp"

#include "../src/Rotation.cpp"
