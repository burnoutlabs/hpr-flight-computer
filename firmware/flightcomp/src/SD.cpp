/**
 * @file SD.cpp
 * @brief List of functions and routines for handling SD card operations.
 *
 * @details
 * This file contains various functions for interacting with SD cards in Teensy-based projects.
 * It provides functionalities to initialize, restart, parse, create, reopen, sync, and write data
 * to the SD card. Additional functions include helper routines for data conversion and parsing.
 *
 * @section functions Function List
 * - beginSd(): Starts the SD card.
 * - restartSd(): Restarts the SD card after a power loss.
 * - parseEepromSettingsSd(): If an EEPROM settings file exists, reads it and updates EEPROM.
 * - createNextFileSd(): Finds the last filename in the list and opens a new file, writes header.
 * - reopenSd(): Reopens the last file on the SD card.
 * - syncSd(): Syncs the SD card in case of a power loss or other event.
 * - readFlightSettingsSd(): Reads the user settings from the SD card.
 * - writeSdFlightData(): Primary routine to capture flight data in ASCII text on the SD card.
 * - writeSDfooter(): Writes the post-flight footer to the SD card once touchdown or timeout is detected.
 * - writeNMEA(): Writes the NMEA GPS data to a separate file.
 *
 * @section helper_functions Helper Function List
 * - writeIntData(): Converts an int to a char array.
 * - writeFloatData(): Converts floats to a char array, with a limit of 4 decimal places.
 * - writeFloatData2(): Converts floats to a char array with no decimal limit.
 * - writeLongData(): Converts long to a char array.
 * - writeUlongData(): Converts unsigned long to a char array.
 * - writeBoolData(): Converts a boolean to a char array.
 * - updateStrPosn(): Helper function to keep track of the position within the char array.
 * - parseNextVariable(): Reads data from the SD card and places it in the dataString char array.
 *
 * @section changelog Change Log
 * - 17 JUL 21: Initial breakout created.
 * - 04 AUG 21: Edited to account for variable gain settings.
 * - 15 AUG 21: Variable gain settings removed.
 * - 18 NOV 21: Added all flight settings to the footer.
 * - 03 JAN 22: Added hardware compatibility macros for Teensy 3.2, 4.0, 4.1, and moved in functions from the main file.
 * - 21 JUN 22: Minor bug fixes and tweaks.
 * - 27 NOV 22: Added an optional GPS output file containing NMEA strings.
 */

#include <Arduino.h>
#include <EEPROM.h>

#include "SD_Card.h"
#include "declarations.h"
#include "flightcomp.h"

// 28 FEB 26: adds rename capability for more unique filenames
//---------------------------------
// SD card files, declared in board.h with the library each board uses
#if defined(__MK66FX1M0__) || defined(__MK64FX512__) || defined(ARDUINO_TEENSY41)
// Teensy 3.5, 3.6, 4.0, 4.1

// SDIO Setup
SdFs SD;
FsFile outputFile;
FsFile settingsFile;
FsFile gpsFile;

#else
// Teensy 3.2 or 4.0

File outputFile;
File settingsFile;
File gpsFile;
#endif

// SD card line buffer and the write position in it, defined here with the writers so the
// compiler can tell them apart while it fills the buffer
int strPosn = 0;
alignas(4) char dataString[1024];

// GPS log variable
char gpsLogBuffer[1024];

// name of the open flight data file, for renameFileSd()
char outputFileName[20] = "";

// name of the last flight data file, for reopenSd()
char fileName[20] = "FLIGHT01.txt";

/**
 * @brief Initializes the SD card.
 */
void beginSd()
{

// Built-in SDIO on Teensy3.5 or 3.6 or Teensy4.1
#if defined(__MK66FX1M0__) || defined(__MK64FX512__) || defined(ARDUINO_TEENSY41)

  // Use SDFat library with Built-in SDIO
  if (!SD.begin(SdioConfig(FIFO_SDIO)))
  {
    Serial.println(F("SD card failed!"));
  }
  else
  {
    Serial.println(F("SD Card OK!"));
  }

// Teensy 3.2 or 4.0
#else

  // Use the SDFat library
  if (pins.sdCs != pins.nullFire)
  {
    if (!SD.begin(pins.sdCs))
    {
      Serial.println(F("SD card failed!"));
    }
    else
    {
      Serial.println(F("SD Card OK!"));
    }
  }

#endif
} // end beginSd

/**
 * @brief Restarts the SD card after a power loss.
 */
void restartSd()
{

// Teensy with built-in SDIO
#if defined(__MK66FX1M0__) || defined(__MK64FX512__) || defined(ARDUINO_TEENSY41)

  SD.begin(SdioConfig(FIFO_SDIO)); // SDFat V2.1 still not working

// Teensy 3.2 or 4.0
#else

  // if the SD Chip Select pin is set to the nullpin, then it must be a Teensy4.1 or Teensy4.0 using a builtin SDIO port
  if (pins.sdCs == pins.nullFire)
  {
    SD.begin();
  } // BUILTIN_SDCARD

  // if the SD Chip Select is not set to the nullpin, then it must be a Teensy3.2 or Teensy4.0 so use the chip select pin
  if (pins.sdCs != pins.nullFire)
  {
    SD.begin(pins.sdCs);
  }

#endif
} // end restartSd

/**
 * @brief Parses EEPROM settings from the SD card and updates EEPROM.
 */
void parseEepromSettingsSd()
{

  // If an EEPROM settings file exists, open it and copy the values into EEPROM
  byte kk;
  int8_t ii;
  if (SD.exists("EEPROMsettings.txt"))
  {
    Serial.println(F("EEPROM file found!  Writing initial EEPROM Settings..."));
    settingsFile = SD.open("EEPROMsettings.txt", FILE_READ);
    //-----------------------------------------------------------------
    // read the device ID codes
    //-----------------------------------------------------------------
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.accelId, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.magId, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.gyroId, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.highGId, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.baroId, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.radioId, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.gpsId, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.sdId, kk);
    //-----------------------------------------------------------------
    // read the device buses
    //-----------------------------------------------------------------
    parseNextVariable(false);
    ii = (uint8_t)dataString[1] - 48;
    Serial.print((char)dataString[0]);
    Serial.print(ii);
    Serial.print(F(", "));
    EEPROM.update(eeprom.accelBusType, (char)dataString[0]);
    EEPROM.update(eeprom.accelBusNum, ii); // accel bus data
    parseNextVariable(false);
    ii = (uint8_t)dataString[1] - 48;
    Serial.print((char)dataString[0]);
    Serial.print(ii);
    Serial.print(F(", "));
    EEPROM.update(eeprom.magBusType, (char)dataString[0]);
    EEPROM.update(eeprom.magBusNum, ii); // mag bus data
    parseNextVariable(false);
    ii = (uint8_t)dataString[1] - 48;
    Serial.print((char)dataString[0]);
    Serial.print(ii);
    Serial.print(F(", "));
    EEPROM.update(eeprom.gyroBusType, (char)dataString[0]);
    EEPROM.update(eeprom.gyroBusNum, ii); // gyro bus data
    parseNextVariable(false);
    ii = (uint8_t)dataString[1] - 48;
    Serial.print((char)dataString[0]);
    Serial.print(ii);
    Serial.print(F(", "));
    EEPROM.update(eeprom.highGBusType, (char)dataString[0]);
    EEPROM.update(eeprom.highGBusNum, ii); // highG bus data
    parseNextVariable(false);
    ii = (uint8_t)dataString[1] - 48;
    Serial.print((char)dataString[0]);
    Serial.print(ii);
    Serial.print(F(", "));
    EEPROM.update(eeprom.baroBusType, (char)dataString[0]);
    EEPROM.update(eeprom.baroBusNum, ii); // baro bus data
    parseNextVariable(false);
    ii = (uint8_t)dataString[1] - 48;
    Serial.print((char)dataString[0]);
    Serial.print(ii);
    Serial.print(F(", "));
    EEPROM.update(eeprom.radioBusType, (char)dataString[0]);
    EEPROM.update(eeprom.radioBusNum, ii); // radio bus data
    parseNextVariable(false);
    ii = (uint8_t)dataString[1] - 48;
    Serial.print((char)dataString[0]);
    Serial.print(ii);
    Serial.print(F(", "));
    EEPROM.update(eeprom.gpsBusType, (char)dataString[0]);
    EEPROM.update(eeprom.gpsBusNum, ii); // GPS bus data
    parseNextVariable(false);
    ii = (uint8_t)dataString[1] - 48;
    Serial.print((char)dataString[0]);
    Serial.print(ii);
    Serial.print(F(", "));
    EEPROM.update(eeprom.sdBusType, (char)dataString[0]);
    EEPROM.update(eeprom.sdBusNum, ii); // SD bus data
    //-----------------------------------------------------------------
    // read the device orientation to the mother board
    //-----------------------------------------------------------------
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    Serial.print((char)dataString[0]);
    Serial.print((char)dataString[1]);
    Serial.print(F(", "));
    EEPROM.update(eeprom.accel2BoardXSign, ii);
    EEPROM.update(eeprom.accel2BoardXAxis, (char)dataString[1]); // accelX orientation
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    Serial.print((char)dataString[0]);
    Serial.print((char)dataString[1]);
    Serial.print(F(", "));
    EEPROM.update(eeprom.accel2BoardYSign, ii);
    EEPROM.update(eeprom.accel2BoardYAxis, (char)dataString[1]); // accelY orientation
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    Serial.print((char)dataString[0]);
    Serial.print((char)dataString[1]);
    Serial.print(F(", "));
    EEPROM.update(eeprom.accel2BoardZSign, ii);
    EEPROM.update(eeprom.accel2BoardZAxis, (char)dataString[1]); // accelZ orientation
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    Serial.print((char)dataString[0]);
    Serial.print((char)dataString[1]);
    Serial.print(F(", "));
    EEPROM.update(eeprom.gyro2BoardXSign, ii);
    EEPROM.update(eeprom.gyro2BoardXAxis, (char)dataString[1]); // gyroX orientation
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    Serial.print((char)dataString[0]);
    Serial.print((char)dataString[1]);
    Serial.print(F(", "));
    EEPROM.update(eeprom.gyro2BoardYSign, ii);
    EEPROM.update(eeprom.gyro2BoardYAxis, (char)dataString[1]); // gyroY orientation
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    Serial.print((char)dataString[0]);
    Serial.print((char)dataString[1]);
    Serial.print(F(", "));
    EEPROM.update(eeprom.gyro2BoardZSign, ii);
    EEPROM.update(eeprom.gyro2BoardZAxis, (char)dataString[1]); // gyroZ orientation
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    Serial.print((char)dataString[0]);
    Serial.print((char)dataString[1]);
    Serial.print(F(", "));
    EEPROM.update(eeprom.highG2BoardXSign, ii);
    EEPROM.update(eeprom.highG2BoardXAxis, (char)dataString[1]); // highGX orientation
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    Serial.print((char)dataString[0]);
    Serial.print((char)dataString[1]);
    Serial.print(F(", "));
    EEPROM.update(eeprom.highG2BoardYSign, ii);
    EEPROM.update(eeprom.highG2BoardYAxis, (char)dataString[1]); // highGY orientation
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    Serial.print((char)dataString[0]);
    Serial.print((char)dataString[1]);
    Serial.print(F(", "));
    EEPROM.update(eeprom.highG2BoardZSign, ii);
    EEPROM.update(eeprom.highG2BoardZAxis, (char)dataString[1]); // highGZ orientation
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    Serial.print((char)dataString[0]);
    Serial.print((char)dataString[1]);
    Serial.print(F(", "));
    EEPROM.update(eeprom.mag2BoardXSign, ii);
    EEPROM.update(eeprom.mag2BoardXAxis, (char)dataString[1]); // magX orientation
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    Serial.print((char)dataString[0]);
    Serial.print((char)dataString[1]);
    Serial.print(F(", "));
    EEPROM.update(eeprom.mag2BoardYSign, ii);
    EEPROM.update(eeprom.mag2BoardYAxis, (char)dataString[1]); // magY orientation
    parseNextVariable(false);
    ii = (dataString[0] == '-') ? -1 : 1;
    Serial.print((char)dataString[0]);
    Serial.print((char)dataString[1]);
    Serial.print(F(", "));
    EEPROM.update(eeprom.mag2BoardZSign, ii);
    EEPROM.update(eeprom.mag2BoardZAxis, (char)dataString[1]); // magZ orientation
    //-----------------------------------------------------------------
    // read the GPIO control pins
    //-----------------------------------------------------------------
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.pyro1ContPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.pyro1FirePin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.pyro2ContPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.pyro2FirePin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.pyro3ContPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.pyro3FirePin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.pyro4ContPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.pyro4FirePin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.nullPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.beepPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.battReadPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.testModeGndPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.testModeReadPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.radioIrqPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.radioRstPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.radioEnPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.radioCsPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.accelCsPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.gyroCsPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.highGCsPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.magCsPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.baroCsPin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.sdCsPin, kk);
    //-----------------------------------------------------------------
    // read the servo control pins
    //-----------------------------------------------------------------
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.servo1Pin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.servo2Pin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.servo3Pin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.servo4Pin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.servo5Pin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.servo6Pin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.servo7Pin, kk);
    kk = (byte)parseNextVariable(true);
    Serial.print(kk);
    Serial.print(F(", "));
    EEPROM.update(eeprom.servo8Pin, kk);
    //-----------------------------------------------------------------
    // read the ham radio call sign
    //-----------------------------------------------------------------
    parseNextVariable(false);
    for (byte i = 0; i < 6; i++)
    {
      EEPROM.update(eeprom.callSign + i, (char)dataString[i]);
    }
    Serial.print(dataString);
    Serial.print(F(", "));
    //-----------------------------------------------------------------
    // read whether or not to put the unit into magnetometer calibration mode
    //-----------------------------------------------------------------
    magCalibrateMode = (byte)parseNextVariable(true); // Sets Magnetometer Calibration Mode, not stored in EEPROM
    //-----------------------------------------------------------------
    // read hardware ID into EEPROM
    //-----------------------------------------------------------------
    parseNextVariable(false);
    EEPROM.update(eeprom.hwVersion, (char)dataString[0]);
    Serial.print((char)dataString[0]);
    Serial.print(F(", "));
    EEPROM.update(eeprom.hwSubVersion, (char)dataString[1]);
    Serial.print((char)dataString[1]);
    Serial.print(F(", "));
    EEPROM.update(eeprom.hwUnitNum, (char)dataString[2]);
    Serial.print((char)dataString[2]);
    Serial.print(F("..."));
    settingsFile.close();
    SD.remove("EEPROMsettings.txt");
    Serial.println(F("Complete!"));
  }
} // end parseEepromSettingsSd

/**
 * @brief Creates a new file on the SD card with a unique name, writes a header.
 */
void createNextFileSd()
{

  char fileName[20] = "FLIGHT01.txt";

  if (settings.testMode)
  {
    Serial.print(F("Creating new SD card file: FLIGHT"));
  }
  fileNum = 0;
  while (SD.exists(fileName))
  {
    fileNum++;
    if (fileNum < 10)
    {
      itoa(fileNum, fileName + 7, 10);
    }
    else
    {
      itoa(fileNum, fileName + 6, 10);
    }
    fileName[8] = '.';
  }
  outputFile = SD.open(fileName, FILE_WRITE);
  strcpy(outputFileName, fileName);
  // Print header
  outputFile.print(F("timeStamp,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,highGx,highGy,highGz,"));
  if (settings.testMode)
  {
    outputFile.print(F("cyclesBtwn,writeFlags,sampleTime,writeTime,"));
  }
  outputFile.println(F("smoothHighGz,rollZ,yawY,pitchX,offVert,intVel,intAlt,fusionVel,fusionAlt,fltEvents,radioCode,pyroCont,pyroFire,pyroPin,baroAlt,altMoveAvg,baroVel,baroPress,baroTemp,battVolt,magX,magY,magZ,gnssLat,gnssLon,gnssSpeed,gnssAlt,gnssAngle,gnssSatellites,radioPacketNum"));
  outputFile.flush();

  // create GPS file
  if (settings.gpsLog)
  {
    fileName[0] = 'G';
    fileName[1] = 'P';
    fileName[2] = 'S';
    fileName[3] = 'l';
    fileName[4] = 'o';
    fileName[5] = 'g';
    fileName[8] = '.';
    fileName[9] = 'n';
    fileName[10] = 'm';
    fileName[11] = 'e';
    fileName[12] = 'a';
    fileName[13] = '\0';
    gpsFile = SD.open(fileName, FILE_WRITE);
    gpsFile.println("GPS Log");
  }

  if (settings.testMode)
  {
    if (fileNum < 10)
    {
      Serial.print('0');
      Serial.println(fileNum);
    }
    else
    {
      Serial.println(fileNum);
    }
  }
} // end createNextFileSd

/**
 * @brief Reopens the most recent file on the SD card.
 */
void reopenSd()
{
  fileNum = 1;
  while (SD.exists(fileName))
  {
    fileNum++;
    if (fileNum < 10)
    {
      itoa(fileNum, fileName + 7, 10);
    }
    else
    {
      itoa(fileNum, fileName + 6, 10);
    }
    fileName[8] = '.';
  }
  fileNum--;
  if (fileNum < 10)
  {
    itoa(fileNum, fileName + 7, 10);
  }
  else
  {
    itoa(fileNum, fileName + 6, 10);
  }
  fileName[8] = '.';
  // outputFile = SD.open(dataString, FILE_WRITE | O_AT_END);
  outputFile = SD.open(fileName, FILE_WRITE);
  strcpy(outputFileName, fileName);
} // end reopenSd

void syncSd()
{
  outputFile.flush();
  if (settings.gpsLog)
  {
    gpsFile.flush();
  }
} // end syncSd

void readFlightSettingsSd()
{
  if (settings.testMode)
  {
    Serial.println(F("Reading User Settings from SD Card"));
  }
  // Open the settings file
  settingsFile = SD.open("Settings.txt", FILE_READ);

  // Read in the user defined variables
  parseNextVariable(false);
  fileNum = 0;
  // Rocket name: what fits in rocketName with its terminator; the rest of a longer name is dropped
  while (dataString[fileNum] != '\0' && fileNum < sizeof(settings.rocketName) - 1)
  {
    settings.rocketName[fileNum] = dataString[fileNum];
    fileNum++;
  }
  settings.rocketName[fileNum] = '\0';
  fileNum = 0;
  parseNextVariable(false);
  settings.fltProfile = dataString[0];
  parseNextVariable(false);
  settings.units = dataString[0];
  if (settings.units == 'M')
  {
    unitConvert = 1.0F;
  }
  parseNextVariable(false);
  settings.reportStyle = dataString[0];
  settings.setupTime = (unsigned long)(parseNextVariable(true) * 1000UL);
  settings.mainDeployAlt = (float)(parseNextVariable(true) / unitConvert);
  settings.apogeeDelay = (unsigned long)(parseNextVariable(true) * 1000000UL);
  settings.rcdTime = (unsigned long)(parseNextVariable(true) * 1000000UL);
  settings.silentMode = (boolean)(parseNextVariable(true));
  settings.magSwitchEnable = (boolean)(parseNextVariable(true));
  settings.inflightRecover = (byte)(parseNextVariable(true));
  settings.gpsLog = (boolean)(parseNextVariable(true));
  if (settings.gpsLog)
  {
    Serial.println("GPS Log Active");
  }
  settings.fireTime = (unsigned long)(parseNextVariable(true) * 1000000UL);
  parseNextVariable(false);
  settings.pyro4Func = dataString[0];
  parseNextVariable(false);
  settings.pyro3Func = dataString[0];
  parseNextVariable(false);
  settings.pyro2Func = dataString[0];
  parseNextVariable(false);
  settings.pyro1Func = dataString[0];
  settings.txEnable = (boolean)parseNextVariable(true);
  settings.txPwr = (byte)(parseNextVariable(true));
  settings.txFreq = (float)(parseNextVariable(true));
  settings.fhss = (boolean)parseNextVariable(true);
  settings.boosterSeparationDelay = (unsigned long)(parseNextVariable(true) * 1000000UL);
  settings.sustainerFireDelay = (unsigned long)(parseNextVariable(true) * 1000000UL);
  parseNextVariable(false);
  settings.airStart1Event = dataString[0];
  settings.airStart1Delay = (unsigned long)(parseNextVariable(true) * 1000000UL);
  parseNextVariable(false);
  settings.airStart2Event = dataString[0];
  settings.airStart2Delay = (unsigned long)(parseNextVariable(true) * 1000000UL);
  settings.altThreshold = (int)(parseNextVariable(true) / unitConvert);
  settings.maxAngle = (int)parseNextVariable(true) * 10;
  settings.stableRotn = (boolean)parseNextVariable(true);
  settings.stableVert = (boolean)parseNextVariable(true);
  settings.flyBack = (boolean)parseNextVariable(true);
  settings.serialDebug = (byte)parseNextVariable(true);
  // close the settings file
  settingsFile.close();
} // end readFlightSettingsSd

/**
 * @brief Writes the footer information to the SD card after flight, then closes
 * the flight data file (and the GNSS log file, if enabled).
 */
void writeSdFooterClose()
{
  // Print the initial conditions
  outputFile.println(F("Max Baro Alt,Max GPS Alt,Max Speed,Max Gs,baseAlt,padTime,initial Y ang,initial X ang,accelX0,accelY0,accelZ0,highGz0,magX0,magY0,magZ0,gyroBiasX,gyroBiasY,gyroBiasZ,accelBiasX,accelBiasY,accelBiasZ,highGbiasX,highGbiasY,highGbiasZ,magBiasX,magBiasY,magBiasZ,baroPressureOffset,baroTempOffset"));
  writeUlongData((uint32_t)(baro.maxAlt * unitConvert));
  writeUlongData((uint32_t)(gnss.maxAlt * unitConvert));
  writeUlongData((uint32_t)(maxVelocity * unitConvert));
  writeFloatData(maxG / 9.80665, 2);
  writeFloatData(baro.baseAlt, 2);
  writeFloatData(((float)fltTime.padTime / (float)1000000), 2);
  writeFloatData(yawY0, 2);
  writeFloatData(pitchX0, 2);
  writeIntData(accel.x0);
  writeIntData(accel.y0);
  writeIntData(accel.z0);
  writeIntData(highG.z0);
  writeIntData(mag.x0);
  writeIntData(mag.y0);
  writeIntData(mag.z0);
  writeIntData(gyro.biasX);
  writeIntData(gyro.biasY);
  writeIntData(gyro.biasZ);
  writeIntData(accel.biasX);
  writeIntData(accel.biasY);
  writeIntData(accel.biasZ);
  writeIntData(highG.biasX);
  writeIntData(highG.biasY);
  writeIntData(highG.biasZ);
  writeIntData(mag.biasX);
  writeIntData(mag.biasY);
  writeIntData(mag.biasZ);
  writeFloatData(baro.pressOffset, 2);
  writeFloatData(baro.tempOffset, 1);
  // carriage return, newline, and null value
  dataString[strPosn] = '\r';
  strPosn++;
  dataString[strPosn] = '\n';
  strPosn++;
  dataString[strPosn] = '\0';
  outputFile.write(dataString, strPosn);

  // write out the launch time and locations
  strPosn = 0;
  outputFile.println(F("launch date, UTC time, launch altitude, launch latitude, launch longitude"));
  // Write out the GPS liftoff date
  outputFile.print(gnss.liftoff.day);
  outputFile.print("/");
  outputFile.print(gnss.liftoff.month);
  outputFile.print("/");
  outputFile.print(gnss.liftoff.year);
  outputFile.print(",");
  // Write out the GPS liftoff time
  outputFile.print(gnss.liftoff.hour);
  outputFile.print(":");
  outputFile.print(gnss.liftoff.minute);
  outputFile.print(":");
  outputFile.print((int)gnss.liftoff.second);
  outputFile.print(",");
  // Write out GPS launch location
  writeFloatData(gnss.baseAlt, 2);
  writeFloatData2(gnss.liftoff.latitude, 6);
  writeFloatData2(gnss.liftoff.longitude, 6);
  // end of sample - carriage return, newline, and null value
  dataString[strPosn] = '\r';
  strPosn++;
  dataString[strPosn] = '\n';
  strPosn++;
  dataString[strPosn] = '\0';
  outputFile.write(dataString, strPosn);

  // Write out the GPS landing location
  strPosn = 0;
  outputFile.println(F("landing date, UTC time, landing altitude, landing latitude, landing longitude"));
  // Write out the GPS landing date
  outputFile.print(gnss.liftoff.day);
  outputFile.print("/");
  outputFile.print(gnss.liftoff.month);
  outputFile.print("/");
  outputFile.print(gnss.liftoff.year);
  outputFile.print(",");
  // Write out the GPS landing time
  outputFile.print(gnss.touchdown.hour);
  outputFile.print(":");
  outputFile.print(gnss.touchdown.minute);
  outputFile.print(":");
  outputFile.print((int)gnss.touchdown.second);
  outputFile.print(",");
  writeFloatData(gnss.touchdown.alt, 2);
  writeFloatData2(gnss.touchdown.latitude, 6);
  writeFloatData2(gnss.touchdown.longitude, 6);
  // end of sample - carriage return, newline, and null value
  dataString[strPosn] = '\r';
  strPosn++;
  dataString[strPosn] = '\n';
  strPosn++;
  dataString[strPosn] = '\0';
  outputFile.write(dataString, strPosn);

  // write out the settings for the flight
  outputFile.print(F("Rocket Name, callsign, HWid, codeVersion, flightProfile, units, inflightRecover, pyro4func, pyro3func, pyro2func, pyro1func, apogeeDelay, mainDeployAlt, setupTime, rcdTime, fireTime, TXenable, TXpwr, TXfreq, FHSS, seaLevelPressure"));
  if (settings.fltProfile == '2')
  {
    outputFile.println("ignitionDelay, sepDelay, altThreshold, maxAng");
  }
  else if (settings.fltProfile == 'A')
  {
    outputFile.println("aistart1event, airstart1delay, airstart2event, airstart2delay, altThreshold, maxAng");
  }
  else
  {
    outputFile.println(' ');
  }
  outputFile.print(settings.rocketName);
  outputFile.print(CSV_SEPARATOR);
  outputFile.print(settings.callSign);
  outputFile.print(CSV_SEPARATOR);
  outputFile.print(settings.hwId);
  outputFile.print(CSV_SEPARATOR);
  outputFile.print(CODE_VERSION, 2);
  outputFile.print(CSV_SEPARATOR);
  outputFile.print((char)settings.fltProfile);
  outputFile.print(CSV_SEPARATOR);
  outputFile.print(settings.units);
  outputFile.print(CSV_SEPARATOR);
  outputFile.print(settings.inflightRecover);
  outputFile.print(CSV_SEPARATOR);
  outputFile.print(settings.pyro4Func);
  outputFile.print(CSV_SEPARATOR);
  outputFile.print(settings.pyro3Func);
  outputFile.print(CSV_SEPARATOR);
  outputFile.print(settings.pyro2Func);
  outputFile.print(CSV_SEPARATOR);
  outputFile.print(settings.pyro1Func);
  outputFile.print(CSV_SEPARATOR);
  strPosn = 0;
  writeFloatData(((float)settings.apogeeDelay) * MILLIONTH, 1);
  writeFloatData(((float)settings.mainDeployAlt) * unitConvert, 0);
  writeFloatData(((float)settings.setupTime) / 1000, 0); // issue
  writeFloatData(settings.rcdTime * MILLIONTH, 0);
  writeFloatData(settings.fireTime * MILLIONTH, 1);
  writeBoolData(settings.txEnable);
  dataString[strPosn] = CSV_SEPARATOR;
  strPosn++;
  writeIntData(settings.txPwr);
  writeFloatData(settings.txFreq, 3);
  writeBoolData(settings.fhss);
  dataString[strPosn] = CSV_SEPARATOR;
  strPosn++;
  writeFloatData(baro.seaLevelPressure, 2);
  if (settings.fltProfile == '2')
  {
    writeFloatData(settings.sustainerFireDelay * MILLIONTH, 1);
    writeFloatData(settings.boosterSeparationDelay * MILLIONTH, 1);
    writeIntData((int)(10 * int(settings.altThreshold * (unitConvert / 10))));
    writeIntData(settings.maxAngle / 10);
  }
  if (settings.fltProfile == 'A')
  {
    dataString[strPosn] = settings.airStart1Event;
    strPosn++;
    writeFloatData(settings.airStart1Delay * MILLIONTH, 1);
    dataString[strPosn] = settings.airStart2Event;
    strPosn++;
    writeFloatData(settings.airStart2Delay * MILLIONTH, 1);
    writeIntData((int)(10 * int(settings.altThreshold * (unitConvert / 10))));
    writeIntData(settings.maxAngle / 10);
  }
  // carriage return, newline, and null value
  dataString[strPosn] = '\r';
  strPosn++;
  dataString[strPosn] = '\n';
  strPosn++;
  dataString[strPosn] = '\0';
  outputFile.write(dataString, strPosn);
  strPosn = 0;

  // close the file
  outputFile.close();
  if (settings.gpsLog)
  {
    gpsFile.close();
  }
} // end SD footer and close

/**
 * @brief Converts an integer to a char array and appends to the data string.
 *
 * @param dataValue Integer value to be converted.
 */
void writeIntData(int dataValue)
{
  itoa(dataValue, dataString + strPosn, DECIMAL_BASE);
  while (dataString[strPosn] != '\0')
  {
    strPosn++;
  }
  dataString[strPosn] = CSV_SEPARATOR;
  strPosn++;
} // end writeIntData

/**
 * @brief Converts an unsigned long to a char array and appends to the data string.
 *
 * @param dataValue Unsigned long value to be converted.
 */
void writeUlongData(unsigned long dataValue)
{
  ultoa(dataValue, dataString + strPosn, DECIMAL_BASE);
  while (dataString[strPosn] != '\0')
  {
    strPosn++;
  }
  dataString[strPosn] = CSV_SEPARATOR;
  strPosn++;
} // end writeUlongData

/**
 * @brief Converts a long to a char array and appends to the data string.
 *
 * @param dataValue Long value to be converted.
 */
void writeLongData(long dataValue)
{
  ltoa(dataValue, dataString + strPosn, DECIMAL_BASE);
  while (dataString[strPosn] != '\0')
  {
    strPosn++;
  }
  dataString[strPosn] = CSV_SEPARATOR;
  strPosn++;
} // end writeLongData

/**
 * @brief Converts a float to a char array with a specified number of decimals.
 *
 * @param dataValue Float value to be converted.
 * @param decimals Number of decimal places.
 */
void writeFloatData2(float dataValue, uint8_t decimals)
{
  dtostrf(dataValue, 2, decimals, dataString + strPosn);
  updateStrPosn();
} // end writeFloatData2

/**
 * @brief Converts a float to a char array with a specified number of decimals.
 *
 * @param dataValue Float value to be converted.
 * @param decimals Number of decimal places.
 */
void writeFloatData(float dataValue, uint8_t decimals)
{
  int32_t fracInt;
  float partial;

  // sign portion
  if (dataValue < 0)
  {
    dataString[strPosn] = '-';
    strPosn++;
    dataValue *= -1;
  }

  // integer portion
  itoa((int16_t)dataValue, dataString + strPosn, DECIMAL_BASE);
  while (dataString[strPosn] != '\0')
  {
    strPosn++;
  }
  dataString[strPosn] = '.';
  strPosn++;

  // fractional portion
  partial = dataValue - (int16_t)(dataValue);
  fracInt = (int32_t)(partial * powf(10, decimals));
  if (fracInt == 0)
  {
    dataString[strPosn] = '0';
    strPosn++;
    dataString[strPosn] = CSV_SEPARATOR;
    strPosn++;
  }
  else
  {
    decimals--;
    while (fracInt < powf(10, decimals))
    {
      dataString[strPosn] = '0';
      strPosn++;
      decimals--;
    }
    ltoa(fracInt, dataString + strPosn, DECIMAL_BASE);
    while (dataString[strPosn] != '\0')
    {
      strPosn++;
    }
    dataString[strPosn] = ',';
    strPosn++;
  }
} // end writeFloatData

/**
 * @brief Parses the next variable from the SD card.
 *
 * @details Skips to the next '=', skips the one character after it, and copies the rest of the
 * line into dataString. At the end of the file (read() returns -1) it stops: a value the file
 * does not reach is empty, and reads as 0.
 *
 * @param flag Indicates whether to return the parsed variable.
 * @return Parsed float value if flag is true, otherwise null.
 */
float parseNextVariable(bool flag)
{
  uint8_t charPosn = 0;
  float dataValue;
  int c;
  charPosn = 0;
  c = '\0';
  while (c != '=' && c >= 0)
  {
    c = settingsFile.read();
  }
  if (c >= 0)
  {
    c = settingsFile.read();
  }
  if (c < 0)
  {
    // end of the file: no value
    dataString[0] = '\0';
  }
  while (c != '\n' && c != '\r' && c >= 0 && charPosn < sizeof(dataString))
  {
    c = settingsFile.read();
    if (c != '\n' && c != '\r' && c >= 0)
    {
      dataString[charPosn] = c;
    }
    else
    {
      dataString[charPosn] = '\0';
    }
    charPosn++;
  }
  if (flag)
  {
    dataValue = atof(dataString);
    return dataValue;
  }
  else
  {
    return '\0';
  }
} // end parseNextVariable

/**
 * @brief Updates the GPS log on the SD card.
 *
 * @param c Character to be added to the GPS log.
 */
void updateGpsLogSd(char c)
{

  static uint16_t logPosn = 0;

  if (logPosn < sizeof(gpsLogBuffer))
  {
    gpsLogBuffer[logPosn] = c;
    logPosn++;
  }
  else
  {
    gpsFile.write(gpsLogBuffer, logPosn);
    logPosn = 0;
    gpsLogBuffer[logPosn] = c;
  }
} // end updateGpsLogSd