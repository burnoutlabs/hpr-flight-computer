/**
 * @file SD_Card.h
 * @brief SD card routines in SD.cpp, and the flight-data writers the flight loop inlines.
 */
#pragma once

#include <Arduino.h>

#include "board.h"
#include "declarations.h"
#include "flightcomp.h"

// SD card line buffer and the write position in it (defined in SD.cpp, alignas as in
// declarations.h)
extern int strPosn;
alignas(4) extern char dataString[1024];

// name of the open flight data file (defined in SD.cpp)
extern char outputFileName[20];

void beginSD();
void restartSD();
void parseEEPROMsettingsSD();
void createNextFileSD();
void reOpenSD();
void syncSD();
void readFlightSettingsSD();
void writeSDfooterClose();
// defined in flightcomp.cpp, beside loop(), its only caller
void renameFileSD();
void writeIntData(int dataValue);
void writeULongData(unsigned long dataValue);
void writeLongData(long dataValue);
void writeFloatData2(float dataValue, uint8_t decimals);
void writeFloatData(float dataValue, uint8_t decimals);
float parseNextVariable(bool flag);
void updateGPSlogSD(char c);

// The flight loop writes a line of flight data on every pass, so writeSDflightData() and
// writeBoolData(), which it calls for every flag, are defined here, inline, for loop() in
// flightcomp.cpp to inline them. updateStrPosn() is always inline as well.

/**
 * @brief Converts a boolean to a char array and appends to the data string.
 *
 * @param dataBool Boolean value to be converted.
 */
inline void writeBoolData(bool dataBool)
{
  dataString[strPosn] = (dataBool) ? '1' : '0';
  strPosn++;
} // end writeBoolData

/**
 * @brief Updates the position within the data string.
 */
inline __attribute__((always_inline)) void updateStrPosn()
{
  while (dataString[strPosn] != '\0')
  {
    strPosn++;
  }
  dataString[strPosn] = cs;
  strPosn++;
} // end updateStrPosn

/**
 * @brief Writes flight data to the SD card in ASCII text format.
 */
inline __attribute__((always_inline)) void writeSDflightData()
{
  // Timestamp
  writeULongData(fltTime.timeCurrent);
  // Accel Data
  writeIntData(accel.x);
  writeIntData(accel.y);
  writeIntData(accel.z);
  // Gyro Data
  writeIntData(gyro.x);
  writeIntData(gyro.y);
  writeIntData(gyro.z);
  // HighG Accel Data
  writeIntData(highG.x);
  writeIntData(highG.y);
  writeIntData(highG.z);
  // debug output for write flags
  if (settings.testMode)
  {
    writeIntData(cyclesBtwn);
    writeBoolData(accel.newSamp);
    writeBoolData(gyro.newSamp);
    writeBoolData(highG.newSamp);
    writeBoolData(mag.newSamp);
    writeBoolData(baro.newSamp);
    writeBoolData(baro.newTemp);
    writeBoolData(SDradioTX);
    writeBoolData(gnss.SDwrite);
    writeBoolData(writeVolt);
    dataString[strPosn] = cs;
    strPosn++;
    writeULongData(sampleTime);
    writeULongData(writeTime);
    cyclesBtwn = 0;
  }
  // reset write flags
  accel.newSamp = gyro.newSamp = highG.newSamp = false;
  // Smoothed high-G data
  writeIntData((int16_t)highGSmooth);
  // Integrated Rotation Values
  writeLongData(rollZ);
  writeIntData(yawY);
  writeIntData(pitchX);
  writeIntData(offVert);
  // Integrated Speed and Altitude
  writeFloatData(accelVel, 2);
  writeFloatData(accelAlt, 2);
  // Sensor Fusion Speed and Altitude
  writeFloatData(fusionVel, 2);
  writeFloatData(fusionAlt, 2);
  // Flight Event Flags
  writeBoolData(events.liftoff);
  writeBoolData(events.boosterBurnout);
  writeBoolData(events.boosterBurnoutCheck);
  if (settings.fltProfile == '2')
  {
    writeBoolData(events.boosterSeparation);
    writeBoolData(events.sustainerFireCheck);
    writeBoolData(events.sustainerFire);
    writeBoolData(events.sustainerIgnition);
    writeBoolData(events.sustainerBurnout);
  }
  if (settings.fltProfile == 'A')
  {
    writeBoolData(events.airStart1Check);
    writeBoolData(events.airStart1Fire);
    writeBoolData(events.airStart1Ignition);
    writeBoolData(events.airStart1Burnout);
    writeBoolData(events.airStart2Check);
    writeBoolData(events.airStart2Fire);
    writeBoolData(events.airStart2Ignition);
    writeBoolData(events.airStart2Burnout);
  }
  writeBoolData(events.apogee);
  writeBoolData(events.apogeeFire);
  writeBoolData(events.mainDeploy);
  writeBoolData(events.touchdown);
  writeBoolData(events.timeOut);
  dataString[strPosn] = cs;
  strPosn++;
  writeIntData(radio.event);
  // Continuity Data
  dataString[strPosn] = 'C';
  strPosn++;
  for (uint8_t i = 0; i < 4; i++)
  {
    writeBoolData(pyros[i].contStatus);
  }
  dataString[strPosn] = cs;
  strPosn++;
  // Pyro Firing Data
  dataString[strPosn] = 'F';
  strPosn++;
  for (uint8_t i = 0; i < 4; i++)
  {
    writeBoolData(pyros[i].fireStatus);
  }
  dataString[strPosn] = cs;
  strPosn++;
  writeIntData(pins.firePin);
  // Barometer
  if (baro.newSamp)
  {
    writeFloatData2(baro.alt, 2);
    writeFloatData2(baro.smoothAlt, 2);
    writeFloatData(baro.vel, 2);
    writeFloatData(baro.pressure, 2);
    baro.newSamp = false;
  }
  else
  {
    for (byte i = 0; i < 4; i++)
    {
      dataString[strPosn] = cs;
      strPosn++;
    }
  }
  // Temperature
  if (baro.newTemp)
  {
    writeFloatData(baro.temperature, 2);
    baro.newTemp = false;
  }
  else
  {
    dataString[strPosn] = cs;
    strPosn++;
  }
  // Battery Voltage
  if (writeVolt)
  {
    writeFloatData(voltage, 2);
    writeVolt = false;
  }
  else
  {
    dataString[strPosn] = cs;
    strPosn++;
  }
  // Magnetometer Data
  if (mag.newSamp)
  {
    writeIntData(mag.x);
    writeIntData(mag.y);
    writeIntData(mag.z);
    mag.newSamp = false;
  }
  else
  {
    dataString[strPosn] = cs;
    strPosn++;
    dataString[strPosn] = cs;
    strPosn++;
    dataString[strPosn] = cs;
    strPosn++;
  }
  // GPS Data
  if (gnss.SDwrite)
  {
    writeFloatData(gnss.latitude, 6);
    writeFloatData(gnss.longitude, 6);
    writeFloatData((float)GPS.speed.mph(), 2);
    writeFloatData2(gnss.alt, 2);
    writeFloatData((float)GPS.course.deg(), 2);
    writeIntData(radio.satNum);
    gnss.SDwrite = false;
  }
  else
  {
    dataString[strPosn] = cs;
    strPosn++;
    dataString[strPosn] = cs;
    strPosn++;
    dataString[strPosn] = cs;
    strPosn++;
    dataString[strPosn] = cs;
    strPosn++;
    dataString[strPosn] = cs;
    strPosn++;
    dataString[strPosn] = cs;
    strPosn++;
  }
  // update the radio packet number
  if (SDradioTX)
  {
    writeIntData(radio.packetnum);
    SDradioTX = false;
  }
  else
  {
    dataString[strPosn] = cs;
    strPosn++;
  }
  // end of sample - carriage return, newline, and null value
  dataString[strPosn] = '\r';
  strPosn++;
  dataString[strPosn] = '\n';
  strPosn++;
  dataString[strPosn] = '\0';
  // write the string to file and capture the time it takes to write to the SD card
  if (settings.testMode)
  {
    writeStart = micros();
  }
  outputFile.write(dataString, strPosn);
  if (settings.testMode)
  {
    writeTime = micros() - writeStart;
    if (writeTime > maxWriteTime)
    {
      maxWriteTime = writeTime;
    }
    if (writeTime > writeThreshold)
    {
      writeThreshCount++;
    }
  }
  strPosn = 0;
} // end writeSDflightData
