/**
 * @file LCD.h
 * @brief The LCD screens for each flight phase, in LCD.cpp.
 */
#pragma once

#include <Arduino.h>
#include <SerLCD.h>

// LCD Setup (defined in LCD.cpp)
extern SerLCD lcd;

void startupLcd();
void preflightLcd();
void inflightLcd();
void postflightLcd();
void signalLostLcd();
void errorLcd();
void changeFreqLcd();
