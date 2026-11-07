/**
 * @file Radio.h
 * @brief SX127X LoRa radio control, frequency hopping and SPI transfers, in Radio.cpp.
 */
#pragma once

#include <Arduino.h>

bool radioBegin();
bool setRadioPwr(uint8_t pwr);
bool radioRecvPkt(uint8_t *data);
float readRadioFreq();
bool setRadioFreq(float freq);
bool radioSetMode(uint8_t mode);
void hopFreq();
void syncPkt(byte rxPacket[]);
void dispPktInfo();
float getFreq(byte chnl);
uint8_t read8(byte reg);
bool write8(byte reg, byte data);
bool burstWrite(byte reg, uint8_t *data, byte len);
bool burstRead(byte reg, uint8_t *data, byte len);
