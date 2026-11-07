/**
 * @file SD_Card.h
 * @brief SD card settings, file creation and CSV writers, in SD_Card.cpp.
 */
#pragma once

#include <Arduino.h>

// Must be defined before SdFat.h so that SdFat declares SoftSpiDriver. Every file that uses
// SdFat includes it through this header, so they all see the same configuration.
#define SPI_DRIVER_SELECT 2
#include <SdFat.h>

const uint8_t SOFT_MISO_PIN = 2;
const uint8_t SOFT_MOSI_PIN = 1;
const uint8_t SOFT_SCK_PIN = 3;
const uint8_t SD_CS_PIN = 0;

// SdFat software SPI template (defined in SD_Card.cpp)
extern SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(0), &softSpi)

// SD card and files (defined in SD_Card.cpp)
extern SdFs SD;
extern FsFile sustainerFile;
extern FsFile boosterFile;
extern FsFile settingsFile;

void readRadioSettingsSd();
float parseNextVariable(boolean flag);
void createSustainerFile();
void createBoosterFile();
void writePreflightData();
void writeInflightData();
void writePostflightData();
void writeIntData(int dataValue);
void writeFloatData(float dataValue, byte decimals);
void writeTimeStamp(int dataValue);
void updateStrPosn();
