/**
 * @file InflightRecover.h
 * @brief In-flight recovery after a power loss or reset, in InflightRecover.cpp.
 */
#pragma once

#include <Arduino.h>

/** Whether a flight whose last recorded event (a FlightCode value) is lastEvent can be resumed. */
bool isRecoverableEvent(uint8_t lastEvent);

/**
 * @brief Bring the hardware up and resume the flight from its last recorded event.
 * @return false if the rocket is standing still (on the ground): setup() then starts normally.
 */
bool rapidReset(uint8_t lastEvent);
