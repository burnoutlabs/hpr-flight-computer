/**
 * @file SX127X_Driver.h
 * @brief Driver for the SX127X LoRa radio, in SX127X_Driver.cpp.
 */
#pragma once

#include <Arduino.h>

bool beginSx127x();
void sleepSx127x();
bool setPwrSx127x(int8_t pwr);
bool sendPktSx127x(uint8_t *data, uint8_t len);
bool config70cmSx127x();
bool setFreqSx127x(float freq);
bool setModeSx127x(uint8_t mode);
void clearFlagsSx127x();
