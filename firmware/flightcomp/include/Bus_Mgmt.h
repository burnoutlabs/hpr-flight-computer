/**
 * @file Bus_Mgmt.h
 * @brief Bus records and the generic I2C, SPI and serial routines in Bus_Mgmt.cpp.
 */
#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "board.h"

// Bus read buffer: burstRead() fills it (defined in Bus_Mgmt.cpp, alignas as in declarations.h)
alignas(4) extern byte rawData[14];

// Hardware Serial for GPS (defined in Bus_Mgmt.cpp)
extern HardwareSerial *gnssSerial;

//-----------------------------------------
// Bus Variables (defined in Bus_Mgmt.cpp)
//-----------------------------------------
// This struct contains all necessary bus info for each device
typedef struct
{
  char type;
  TwoWire *wire = &Wire;
  uint32_t i2cRate = 400000;
  uint8_t i2cAddress;
  SPIClass *spi = &SPI;
  SPISettings spiSet = SPISettings(10000000, MSBFIRST, SPI_MODE0);
  uint8_t cs;
  uint8_t writeMask = 0x00;
  uint8_t readMask = 0x00;
  uint8_t incMask = 0x00;
} Bus;
extern Bus accelBus;
extern Bus gyroBus;
extern Bus magBus;
extern Bus highGBus;
extern Bus baroBus;
extern Bus radioBus;
extern Bus sdBus;
extern Bus *activeBus;

void setGnssSerial();
void startI2c(Bus *myBus, uint8_t busNum);
bool testSensor(uint8_t address);
void startSpi(Bus *myBus, uint8_t busNum);
uint8_t read8(uint8_t reg);
void burstRead(uint8_t reg, uint8_t bytes);
void write8(uint8_t reg, uint8_t val);
void write16(uint8_t reg, uint16_t val);
void burstWrite(uint8_t reg, uint8_t *data, uint8_t len);
