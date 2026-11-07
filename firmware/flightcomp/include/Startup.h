/**
 * @file Startup.h
 * @brief setup(), its initialization steps, and the pyro continuity check, in Startup.cpp.
 *
 * @details
 * The initialization steps are separate functions so that setup() and the in-flight recovery,
 * rapidReset() (InflightRecover.cpp), can bring the hardware up the same way.
 */
#pragma once

#include <Arduino.h>

void setup(void);
void checkPyroContinuity();

// Initialization steps, in the order setup() runs them
void readHardwareSettingsEeprom();
void setPinModes();
void beginGnssSerial();
void setAnalogResolution();
void beginSensors();
void scaleSensorGains();
void beginRadioLink();
void limitSettings();
void configurePyroChannels();
void configureRadio();
void attachServos();
void readCalibrationEeprom();
