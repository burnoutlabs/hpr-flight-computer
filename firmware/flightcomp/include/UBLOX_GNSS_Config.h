/**
 * @file UBLOX_GNSS_Config.h
 * @brief UBLOX GNSS receiver configuration, in UBLOX_GNSS_Config.cpp.
 */
#pragma once

#include <Arduino.h>

void ubloxRestoreDefaults();
void ubloxConfig();
void ubloxPowerSave();
void calcChecksum(byte *checksumPayload, byte payloadSize);
void sendUbx(byte *ubxMsg, byte msgLength);
byte getUbxAck(byte *msgId);
void printHex(uint8_t *data, uint8_t length);
