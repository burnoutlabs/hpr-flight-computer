/**
 * @file Telemetry.h
 * @brief Telemetry packet building, sending and frequency hopping, in Telemetry.cpp.
 */
#pragma once

#include <Arduino.h>

void beginTelemetry();
void buildTelemetryPkt();
void sendTelemetryPkt();
void hopTxFreq();
void syncPkt();
