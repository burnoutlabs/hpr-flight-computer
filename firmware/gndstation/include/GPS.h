/**
 * @file GPS.h
 * @brief UBLOX GPS configuration and the distance calculation, in GPS.cpp.
 */
#pragma once

#include <Arduino.h>

void restoreGpsDefaults();
void configGps();
void gpsPowerSaveMode();
void calcChecksum(byte *checksumPayload, byte payloadSize);
void sendUbx(byte *ubxMsg, byte msgLength);
byte getUbxAck(byte *msgId);
void printHex(uint8_t *data, uint8_t length);
float calcGpsDist(float aLat, float aLon, float bLat, float bLon);
