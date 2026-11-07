/*!
 * \file UBLOX_GNSS_Config.cpp
 * \brief Configuration and power management routines for u-Blox GPS modules.
 *
 * This code provides a set of functions to configure, power-manage, and communicate
 * with u-Blox GPS modules. It includes a collection of helper functions for serial
 * communication, data conversion, and acknowledgment handling.
 */

#include <Arduino.h>

#include "UBLOX_GNSS_Config.h"
#include "declarations.h"
#include "Bus_Mgmt.h"

/*!
 * \brief Restore the u-Blox GNSS receiver to factory default settings.
 *
 * This function sends a command to the u-Blox GNSS module to restore factory
 * default settings. It is reached through the gnssRestoreDefaults() pointer when a
 * u-Blox receiver is configured. Debug output goes to the serial console in test mode.
 */
void ubloxRestoreDefaults()
{

  uint8_t gpsSetSuccess = 0;
  if (settings.testMode)
  {
    Serial.println("Configuring u-Blox GPS initial state...");
  }

  // Generate the configuration string for Factory Default Settings
  uint8_t setDefaults[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x17, 0x2F, 0xAE};

  // Restore Factory Defaults
  while (gpsSetSuccess < 3)
  {
    if (settings.testMode)
    {
      Serial.print("Restoring Factory Defaults... ");
    }
    sendUbx(&setDefaults[0], sizeof(setDefaults)); // Send UBX Packet
    gpsSetSuccess += getUbxAck(&setDefaults[2]);
  }
  if (gpsSetSuccess == 3 && settings.testMode)
  {
    Serial.println("Restore factory defaults failed!");
  }
}

/*!
 * \brief Configure the u-Blox GNSS receiver with custom settings.
 *
 * This function sends a series of UBX commands to update navigation settings,
 * data rate, GNSS constellations, NMEA sentence configurations, interference
 * thresholds, and baud rates. It is reached through the gnssConfig() pointer.
 *
 * The module version is taken from sensors.GNSS (1, 2, or 3); VTG (course over ground)
 * NMEA messages stay enabled only when settings.flyBack is set; debug output goes to
 * the serial console in test mode.
 */
void ubloxConfig()
{

  uint8_t gpsVersion = sensors.gnss;
  bool serialOutput = settings.testMode;
  bool vtgOption = settings.flyBack;

  byte gpsSetSuccess = 0;

  if (serialOutput)
  {
    Serial.println("Configuring u-Blox GPS ...");
  }

  // Generate the configuration string for Navigation Mode
  byte setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x1F};

  // Generate the configuration string for Data Rate
  // 4Hz nominal rate
  byte setDataRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96};

  // 5Hz Max data rate for NEO-M8N
  if (gpsVersion == 1)
  {
    setDataRate[6] = 0xC8;
    setDataRate[12] = 0xDE;
    setDataRate[13] = 0x6A;
  }

  // 10Hz Max data rate for NEO-M8Q, MAX-M8Q/W, SAM-M8Q
  if (gpsVersion == 2)
  {
    setDataRate[6] = 0x64;
    setDataRate[12] = 0x7A;
    setDataRate[13] = 0x12;
  }

  // 25Hz Max data rate for NEO-M9N
  if (gpsVersion == 3)
  {
    setDataRate[6] = 0x28;
    setDataRate[12] = 0x3E;
    setDataRate[13] = 0xAA;
  }

  // Faster Baud Rate for the higher update rates
  byte setBaudRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAF, 0x70};

  // Generate the configuration string for NMEA messages
  byte setGll[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
  byte setGsa[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
  byte setGsv[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
  byte setVtg[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
  byte setNmea41[] = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0x57};

  // Generate the configuration string for interference resistance settings
  byte setJam[] = {0xB5, 0x62, 0x06, 0x39, 0x08, 0x00, 0xF3, 0xAC, 0x62, 0xAD, 0x1E, 0x43, 0x00, 0x00, 0x56, 0x45};

  // set the constellations used
  // For M8, just add Galileo, from https://portal.u-blox.com/s/question/0D52p00008HKEEYCA5/ublox-gps-galileo-enabling-for-ubx-m8
  byte setSat[] = {0xB5, 0x62, 0x06, 0x3E, 0x0C, 0x00, 0x00, 0x00, 0x20, 0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x82, 0x56};

  // UBLOX Devices are numbered 1 through 3, Adafruit is 4
  if (gpsVersion < 4)
  {

    // Turn Off NMEA GSA Messages
    while (gpsSetSuccess < 3)
    {
      if (serialOutput)
      {
        Serial.print("Deactivating NMEA GSA Messages... ");
      }
      sendUbx(setGsa, sizeof(setGsa));
      gpsSetSuccess += getUbxAck(&setGsa[2]);
    }
    if (gpsSetSuccess == 3 && serialOutput)
    {
      Serial.println("NMEA GSA Message Deactivation Failed!");
    }
    gpsSetSuccess = 0;

    // Set Navigation Mode
    while (gpsSetSuccess < 3)
    {
      if (serialOutput)
      {
        Serial.print("Setting Navigation Mode... ");
      }
      sendUbx(&setNav[0], sizeof(setNav)); // Send UBX Packet
      gpsSetSuccess += getUbxAck(&setNav[2]);
    }
    if (gpsSetSuccess == 3 && serialOutput)
    {
      Serial.println("Navigation mode configuration failed!");
    }
    gpsSetSuccess = 0;

    // Turn Off NMEA GSV Messages
    while (gpsSetSuccess < 3)
    {
      if (serialOutput)
      {
        Serial.print("Deactivating NMEA GSV Messages... ");
      }
      sendUbx(setGsv, sizeof(setGsv));
      gpsSetSuccess += getUbxAck(&setGsv[2]);
    }
    if (gpsSetSuccess == 3 && serialOutput)
    {
      Serial.println("NMEA GSV Message Deactivation Failed!");
    }
    gpsSetSuccess = 0;

    // Set Data Update Rate
    while (gpsSetSuccess < 3)
    {
      if (serialOutput)
      {
        Serial.print("Setting Data Update Rate... ");
      }
      sendUbx(&setDataRate[0], sizeof(setDataRate)); // Send UBX Packet
      gpsSetSuccess += getUbxAck(&setDataRate[2]);
    }
    if (gpsSetSuccess == 3 && serialOutput)
    {
      Serial.println("Data update mode configuration failed!");
    }
    gpsSetSuccess = 0;

    // Turn on NMEA4.1 Messages for less than M9 versions
    while (gpsVersion < 3 && gpsSetSuccess < 3)
    {
      if (serialOutput)
      {
        Serial.print("Turning on NMEA 4.1 Messages... ");
      }
      sendUbx(setNmea41, sizeof(setNmea41));
      gpsSetSuccess += getUbxAck(&setNmea41[2]);
    }
    if (gpsSetSuccess == 3 && serialOutput)
    {
      Serial.println("NMEA 4.1 Message Activation Failed!");
    }
    gpsSetSuccess = 0;

    // Turn on Galileo for M8 versions since it is not enabled by default
    while (gpsVersion < 3 && gpsSetSuccess < 3)
    {
      if (serialOutput)
      {
        Serial.print("Setting Satellite Constellations... ");
      }
      sendUbx(setSat, sizeof(setSat));
      gpsSetSuccess += getUbxAck(&setSat[2]);
    }
    if (gpsSetSuccess == 3 && serialOutput)
    {
      Serial.println("Satellite Settings Failed!");
    }
    gpsSetSuccess = 0;

    // Turn Off NMEA VTG Messages
    while (!vtgOption && gpsSetSuccess < 3)
    {
      if (serialOutput)
      {
        Serial.print("Deactivating NMEA VTG Messages... ");
      }
      sendUbx(setVtg, sizeof(setVtg));
      gpsSetSuccess += getUbxAck(&setVtg[2]);
    }
    if (!vtgOption && gpsSetSuccess == 3 && serialOutput)
    {
      Serial.println("NMEA VTG Message Deactivation Failed!");
    }
    gpsSetSuccess = 0;

    // Set Interference Thresholds
    while (gpsSetSuccess < 3)
    {
      if (serialOutput)
      {
        Serial.print("Setting Interferece Thresholds... ");
      }
      sendUbx(setJam, sizeof(setJam));
      gpsSetSuccess += getUbxAck(&setJam[2]);
    }
    if (gpsSetSuccess == 3 && serialOutput)
    {
      Serial.println("Interference Settings Failed!");
    }
    gpsSetSuccess = 0;

    // Turn Off NMEA GLL Messages
    while (gpsSetSuccess < 3)
    {
      if (serialOutput)
      {
        Serial.print("Deactivating NMEA GLL Messages... ");
      }
      sendUbx(setGll, sizeof(setGll));
      gpsSetSuccess += getUbxAck(&setGll[2]);
    }
    if (gpsSetSuccess == 3 && serialOutput)
    {
      Serial.println("NMEA GLL Message Deactivation Failed!");
    }
    gpsSetSuccess = 0;

    // Increase Baud-Rate on M8Q for faster GPS updates
    while (gpsVersion == 2 && gpsSetSuccess < 3)
    {
      if (serialOutput)
      {
        Serial.print("Setting Ublox Baud Rate 38400... ");
      }
      sendUbx(setBaudRate, sizeof(setBaudRate));
      gpsSetSuccess += getUbxAck(&setBaudRate[2]);
    }
    if (gpsSetSuccess == 3 && serialOutput)
    {
      Serial.println("Ublox Baud Rate 38400 Failed!");
    }
    else if (gpsVersion == 2)
    {
      gnssSerial->end();
      gnssSerial->flush();
      gnssSerial->begin(38400);
    }
  }

} // end configGPS

/*!
 * \brief Enter GNSS Power Save Mode for reduced power consumption.
 *
 * This function configures the u-Blox GNSS receiver to enter power save mode, reducing
 * power consumption and updating less frequently. It is reached through the
 * gnssPowerSave() pointer; the module version is taken from sensors.GNSS, and debug
 * output goes to the serial console in test mode.
 */
void ubloxPowerSave()
{

  bool serialOutput = settings.testMode;
  uint8_t gpsVersion = sensors.gnss;

  // configure UBLOX chips
  if (gpsVersion < 4)
  {

    byte gpsSetSuccess = 0;
    if (serialOutput)
    {
      Serial.println("Configuring u-Blox GPS Power Save mode... ");
    }

    // Generate the configuration string to enable power save mode (RXM)
    byte enablePwrSv[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};

    // Generate configuration string to put reciever into power save mode (PM2)
    byte setPwr2[] = {0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x0E, 0x90, 0x40, 0x01, 0x20, 0x4E, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x87, 0x02, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0x67, 0x83};

    // Generate configuration string to put receiver into 1Hz update mode
    // byte setDataRate1Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39};//once per second
    byte setDataRate01Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x10, 0x27, 0x01, 0x00, 0x01, 0x00, 0x4D, 0xDD}; // once per 10 seconds

    // Enable Power Save Mode
    while (gpsSetSuccess < 3)
    {
      if (serialOutput)
      {
        Serial.print("Enabling Power Save Mode... ");
      }
      sendUbx(enablePwrSv, sizeof(enablePwrSv));
      gpsSetSuccess += getUbxAck(&enablePwrSv[2]);
    }
    if (gpsSetSuccess == 3 && serialOutput)
    {
      Serial.println("Enable Power Save Failed!");
    }
    gpsSetSuccess = 0;

    // Config Power Save Mode
    while (gpsSetSuccess < 3)
    {
      if (serialOutput)
      {
        Serial.print("Config Power Save Mode... ");
      }
      sendUbx(setPwr2, sizeof(setPwr2));
      gpsSetSuccess += getUbxAck(&setPwr2[2]);
    }
    if (gpsSetSuccess == 3 && serialOutput)
    {
      Serial.println("Commit PSM Message Failed!");
    }
    gpsSetSuccess = 0;

    // Set Data Update Rate
    while (gpsSetSuccess < 3)
    {
      if (serialOutput)
      {
        Serial.print("Setting Data Update Rate... ");
      }
      sendUbx(setDataRate01Hz, sizeof(setDataRate01Hz)); // Send UBX Packet
      gpsSetSuccess += getUbxAck(&setDataRate01Hz[2]);
    }
    if (gpsSetSuccess == 3 && serialOutput)
    {
      Serial.println("Data update mode configuration failed!");
    }
    gpsSetSuccess = 0;
  }
} // End PowerSave Mode

/*!
 * \brief Calculate checksum for a UBX message payload.
 *
 * This function calculates the checksum for a given UBX message payload.
 * It uses two checksum values (CK_A and CK_B) calculated by iterating over
 * the payload. The calculated checksums are then appended to the end of
 * the payload.
 *
 * \param checksumPayload Pointer to the start of the UBX message payload.
 * \param payloadSize Size of the payload in bytes.
 */
void calcChecksum(byte *checksumPayload, byte payloadSize)
{
  byte CK_A = 0, CK_B = 0;
  for (int i = 0; i < payloadSize; i++)
  {
    CK_A = CK_A + *checksumPayload;
    CK_B = CK_B + CK_A;
    checksumPayload++;
  }

  *checksumPayload = CK_A;
  checksumPayload++;
  *checksumPayload = CK_B;
}

/*!
 * \brief Send UBX command to the GPS module.
 *
 * This helper function sends a UBX command through the serial connection.
 *
 * \param ubxMsg Pointer to the UBX command message.
 * \param msgLength Length of the UBX command message.
 */
void sendUbx(byte *ubxMsg, byte msgLength)
{
  for (int i = 0; i < msgLength; i++)
  {
    gnssSerial->write(ubxMsg[i]);
    gnssSerial->flush();
  }

  gnssSerial->println();
  gnssSerial->flush();
}

/*!
 * \brief Get UBX acknowledgment for a given command.
 *
 * This function reads the acknowledgment message from the GPS module and checks
 * if it corresponds to the command that was sent.
 *
 * \param msgId Pointer to the expected acknowledgment ID.
 * \return Returns 10 for a successful acknowledgment, 1 for NAK, 5 for timeout.
 */
byte getUbxAck(byte *msgId)
{
  byte CK_A = 0, CK_B = 0;
  byte incomingChar;
  unsigned long ackWait = millis();
  byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int i = 0;
  while (1)
  {
    if (gnssSerial->available())
    {
      incomingChar = gnssSerial->read();
      if (incomingChar == ackPacket[i])
      {
        i++;
      }
      else if (i > 2)
      {
        ackPacket[i] = incomingChar;
        i++;
      }
    }
    if (i > 9)
      break;
    if ((millis() - ackWait) > 1500)
    {
      Serial.println("ACK Timeout");
      return 5;
    }
    if (i == 4 && ackPacket[3] == 0x00)
    {
      Serial.println("NAK Received");
      return 1;
    }
  }

  for (i = 2; i < 8; i++)
  {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }

  if (msgId[0] == ackPacket[6] && msgId[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9])
  {
    Serial.println("Success!");
    Serial.print("ACK Received! ");
    printHex(ackPacket, sizeof(ackPacket));
    return 10;
  }
  else
  {
    Serial.print("ACK Checksum Failure: ");
    printHex(ackPacket, sizeof(ackPacket));
    delay(1000);
    return 1;
  }
}

/*!
 * \brief Print data as hexadecimal representation.
 *
 * This function prints a given byte array in hexadecimal format.
 *
 * \param data Pointer to the byte array.
 * \param length Length of the byte array.
 */
void printHex(uint8_t *data, uint8_t length)
{ // prints 8-bit data in hex

  char tmp[length * 2 + 1];
  byte first;
  int j = 0;
  for (byte i = 0; i < length; i++)
  {

    first = (data[i] >> 4) | 48;
    if (first > 57)
      tmp[j] = first + (byte)7;
    else
      tmp[j] = first;
    j++;

    first = (data[i] & 0x0F) | 48;
    if (first > 57)
      tmp[j] = first + (byte)7;
    else
      tmp[j] = first;
    j++;
  }

  tmp[length * 2] = 0;
  for (byte i = 0, j = 0; i < sizeof(tmp); i++)
  {
    Serial.print(tmp[i]);
    if (j == 1)
    {
      Serial.print(" ");
      j = 0;
    }
    else
      j++;
  }
  Serial.println();
}