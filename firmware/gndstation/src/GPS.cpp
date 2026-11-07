/**
 * @brief List of functions and routines for GPS configuration and control.
 */

/**
 * @section List of Functions & Routines
 *
 * - restoreGpsDefaults(): Overrides command settings and restores factory settings.
 * - configGps(): Sends UBX commands to update navigation, data rate, GNSS constellations, reduce NMEA sentences, set interference thresholds, and adjust baud rate.
 * - gpsPowerSaveMode(): Reduces GPS power consumption and update rate after touchdown.
 * - sendUbx(): Helper function to send serial data.
 * - getUbxAck(): Function to receive the UBX acknowledgment for each command.
 * - printHex(): Helper function to convert binary data to hexadecimal.
 */

/**
 * @section Reference
 *
 * UBLOX configuration modified from [UBLOX Playground on Arduino](https://playground.arduino.cc/UBlox/GPS).
 */

#include <Arduino.h>

#include "declarations.h"
#include "GPS.h"

void restoreGpsDefaults()
{

  byte gpsSetSuccess = 0;
  Serial.println("Configuring u-Blox GPS initial state...");

  // Generate the configuration string for Factory Default Settings
  byte setDefaults[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x17, 0x2F, 0xAE};

  // Restore Factory Defaults
  while (gpsSetSuccess < 3)
  {
    Serial.print("Restoring Factory Defaults... ");
    sendUbx(&setDefaults[0], sizeof(setDefaults)); // Send UBX Packet
    gpsSetSuccess += getUbxAck(&setDefaults[2]);
  }
  if (gpsSetSuccess == 3)
  {
    Serial.println("Restore factory defaults failed!");
  }
}

void configGps()
{

  byte gpsSetSuccess = 0;

  Serial.println("Configuring u-Blox GPS ...");

  // Generate the configuration string for Navigation Mode
  byte setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x1F};

  // Generate the configuration string for Data Rate
  // 4Hz nominal rate
  byte setDataRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96};

  // 10Hz Max data rate for NEO-M8Q, MAX-M8Q/W, SAM-M8Q
  setDataRate[6] = 0x64;
  setDataRate[12] = 0x7A;
  setDataRate[13] = 0x12;

  // 25Hz Max data rate for NEO-M9N
  // setDataRate[6] = 0x33; setDataRate[12] = 0x49; setDataRate[13] = 0xEC;

  // Faster Baud Rate for the higher update rates
  byte setBaudRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAF, 0x70};

  // Generate the configuration string for NMEA messages
  byte setGll[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
  // byte setGsa[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
  byte setGsa[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};
  byte setGsv[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
  // byte setRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
  byte setVtg[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
  byte setNmea41[] = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0x57};

  // Generate the configuration string for interference resistance settings
  byte setJam[] = {0xB5, 0x62, 0x06, 0x39, 0x08, 0x00, 0xF3, 0xAC, 0x62, 0xAD, 0x1E, 0x43, 0x00, 0x00, 0x56, 0x45};
  // byte setJam[] = {0xB5, 0x62, 0x06, 0x39, 0x00, 0x00, 0xF3, 0xAC, 0x62, 0xAD, 0x1E, 0xD3, 0x00, 0x00, 0xE6, 0xF5};

  // add Galileo, from https://portal.u-blox.com/s/question/0D52p00008HKEEYCA5/ublox-gps-galileo-enabling-for-ubx-m8
  byte setSat[] = {0xB5, 0x62, 0x06, 0x3E, 0x0C, 0x00, 0x00, 0x00, 0x20, 0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x82, 0x56};

  // Turn Off NMEA GSA Messages
  while (gpsSetSuccess < 3)
  {
    Serial.print("Deactivating NMEA GSA Messages... ");
    sendUbx(setGsa, sizeof(setGsa));
    gpsSetSuccess += getUbxAck(&setGsa[2]);
  }
  if (gpsSetSuccess == 3)
  {
    Serial.println("NMEA GSA Message Deactivation Failed!");
  }
  gpsSetSuccess = 0;

  // Set Navigation Mode
  while (gpsSetSuccess < 3)
  {
    Serial.print("Setting Navigation Mode... ");
    sendUbx(&setNav[0], sizeof(setNav)); // Send UBX Packet
    gpsSetSuccess += getUbxAck(&setNav[2]);
  }
  if (gpsSetSuccess == 3)
  {
    Serial.println("Navigation mode configuration failed!");
  }
  gpsSetSuccess = 0;

  // Turn Off NMEA GSV Messages
  while (gpsSetSuccess < 3)
  {
    Serial.print("Deactivating NMEA GSV Messages... ");
    sendUbx(setGsv, sizeof(setGsv));
    gpsSetSuccess += getUbxAck(&setGsv[2]);
  }
  if (gpsSetSuccess == 3)
  {
    Serial.println("NMEA GSV Message Deactivation Failed!");
  }
  gpsSetSuccess = 0;

  // Set Data Update Rate
  //  while(gpsSetSuccess < 3) {
  //    Serial.print("Setting Data Update Rate... ");
  //    sendUbx(&setDataRate[0], sizeof(setDataRate));  //Send UBX Packet
  //    gpsSetSuccess += getUbxAck(&setDataRate[2]);}
  //  if (gpsSetSuccess == 3){Serial.println("Data update mode configuration failed!");}
  //  gpsSetSuccess = 0;

  // Turn on NMEA4.1 Messages
  while (gpsSetSuccess < 3)
  {
    Serial.print("Turning on NMEA 4.1 Messages... ");
    sendUbx(setNmea41, sizeof(setNmea41));
    gpsSetSuccess += getUbxAck(&setNmea41[2]);
  }
  if (gpsSetSuccess == 3)
  {
    Serial.println("NMEA 4.1 Message Activation Failed!");
  }
  gpsSetSuccess = 0;

  // Set Satellite Constellations
  while (gpsSetSuccess < 3)
  {
    Serial.print("Setting Satellite Constellations... ");
    sendUbx(setSat, sizeof(setSat));
    gpsSetSuccess += getUbxAck(&setSat[2]);
  }
  if (gpsSetSuccess == 3)
  {
    Serial.println("Satellite Settings Failed!");
  }
  gpsSetSuccess = 0;

  // Turn Off NMEA VTG Messages
  while (gpsSetSuccess < 3)
  {
    Serial.print("Deactivating NMEA VTG Messages... ");
    sendUbx(setVtg, sizeof(setVtg));
    gpsSetSuccess += getUbxAck(&setVtg[2]);
  }
  if (gpsSetSuccess == 3)
  {
    Serial.println("NMEA VTG Message Deactivation Failed!");
  }
  gpsSetSuccess = 0;

  // Set Interference Thresholds
  while (gpsSetSuccess < 3)
  {
    Serial.print("Setting Interferece Thresholds... ");
    sendUbx(setJam, sizeof(setJam));
    gpsSetSuccess += getUbxAck(&setJam[2]);
  }
  if (gpsSetSuccess == 3)
  {
    Serial.println("Interference Settings Failed!");
  }
  gpsSetSuccess = 0;

  // Turn Off NMEA GLL Messages
  while (gpsSetSuccess < 3)
  {
    Serial.print("Deactivating NMEA GLL Messages... ");
    sendUbx(setGll, sizeof(setGll));
    gpsSetSuccess += getUbxAck(&setGll[2]);
  }
  if (gpsSetSuccess == 3)
  {
    Serial.println("NMEA GLL Message Deactivation Failed!");
  }
  gpsSetSuccess = 0;

} // end configGps

void gpsPowerSaveMode()
{

  byte gpsSetSuccess = 0;
  Serial.println("Configuring u-Blox GPS Power Save mode... ");

  // Generate the configuration string to config GNSS (GLONASS Off)
  byte setGnss[] = {0xB5, 0x62, 0x6, 0x3E, 0x24, 0x0, 0x0, 0x0, 0x16, 0x4, 0x0, 0x8, 0xFF, 0x0, 0x1, 0x0, 0x0, 0x1, 0x1, 0x0, 0x3, 0x0, 0x1, 0x0, 0x0, 0x1, 0x5, 0x0, 0x3, 0x0, 0x1, 0x0, 0x0, 0x1, 0x6, 0x4, 0xFF, 0x0, 0x0, 0x0, 0x0, 0x1, 0xA5, 0x8E};

  // Generate the configuration string for Power Save mode with 5 minute updates
  byte setPwr[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x1F};

  // Generate configuration string to put reciever into power save mode
  byte setPwr2[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};

  // Generate configuration string to put receiver into 1Hz update mode
  byte setDataRate1Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39};  // once per second
  byte setDataRate01Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x10, 0x27, 0x01, 0x00, 0x01, 0x00, 0x4D, 0xDD}; // once per 10 seconds

  // Config GNSS
  while (gpsSetSuccess < 3)
  {
    Serial.print("Configuring GNSS for Power Save Mode (GLONASS OFF)... ");
    sendUbx(setGnss, sizeof(setGnss));
    gpsSetSuccess += getUbxAck(&setGnss[2]);
  }
  if (gpsSetSuccess == 3)
  {
    Serial.println("Config GNSS Message Failed!");
  }
  gpsSetSuccess = 0;

  // Config Power Save
  while (gpsSetSuccess < 3)
  {
    Serial.print("Configuring Power Save Mode... ");
    sendUbx(setPwr, sizeof(setPwr));
    gpsSetSuccess += getUbxAck(&setPwr[2]);
  }
  if (gpsSetSuccess == 3)
  {
    Serial.println("Config PSM Message Failed!");
  }
  gpsSetSuccess = 0;

  // Commit PSM
  while (gpsSetSuccess < 3)
  {
    Serial.print("Commit Power Save Mode... ");
    sendUbx(setPwr2, sizeof(setPwr2));
    gpsSetSuccess += getUbxAck(&setPwr2[2]);
  }
  if (gpsSetSuccess == 3)
  {
    Serial.println("Commit PSM Message Failed!");
  }
  gpsSetSuccess = 0;

  // Set Data Update Rate
  while (gpsSetSuccess < 3)
  {
    Serial.print("Setting Data Update Rate... ");
    sendUbx(setDataRate01Hz, sizeof(setDataRate01Hz)); // Send UBX Packet
    gpsSetSuccess += getUbxAck(&setDataRate01Hz[2]);
  }
  if (gpsSetSuccess == 3)
  {
    Serial.println("Data update mode configuration failed!");
  }
  gpsSetSuccess = 0;
}

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

void sendUbx(byte *ubxMsg, byte msgLength)
{
  for (int i = 0; i < msgLength; i++)
  {
    Serial2.write(ubxMsg[i]);
    Serial2.flush();
  }

  Serial2.println();
  Serial2.flush();
}

byte getUbxAck(byte *msgId)
{
  byte CK_A = 0, CK_B = 0;
  byte incomingChar;
  unsigned long ackWait = millis();
  byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int i = 0;
  while (1)
  {
    if (Serial2.available())
    {
      incomingChar = Serial2.read();
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

void printHex(uint8_t *data, uint8_t length) // prints 8-bit data in hex
{
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

float calcGpsDist(float aLat, float aLon, float bLat, float bLon)
{

  // calculate the difference between the points
  float diffLat = aLat - bLat;
  float diffLon = aLon - bLon;

  // convert latitude degrees to meters
  diffLat *= 111111.1;

  // https://www.sco.wisc.edu/2022/01/21/how-big-is-a-degree/#:~:text=Therefore%20we%20can%20easily%20compute,further%20subdivisions%20of%20a%20degree.&text=circumference%20of%2025%2C000%20miles.
  // convert longitude degrees to meters through a simple interpolation table
  if (fabs(diffLon) <= 15.0)
  {
    diffLon = fabs(diffLon) * 107325.1 + (15 - fabs(diffLon)) * 3786;
  }
  else if (fabs(diffLon) <= 30.0)
  {
    diffLon = fabs(diffLon) * 96225.0 + (30 - fabs(diffLon)) * 11100.1;
  }
  else if (fabs(diffLon) <= 45.0)
  {
    diffLon = fabs(diffLon) * 78567.4 + (45 - fabs(diffLon)) * 17657.6;
  }
  else if (fabs(diffLon) <= 60.0)
  {
    diffLon = fabs(diffLon) * 55555.6 + (60 - fabs(diffLon)) * 23011.8;
  }
  else if (fabs(diffLon) <= 75.0)
  {
    diffLon = fabs(diffLon) * 28757.7 + (75 - fabs(diffLon)) * 26797.9;
  }
  else
  {
    diffLon = (90 - fabs(diffLon)) * 28757.7;
  } // why one would launch rockets at the north pole is beyond me

  return sqrt(diffLat * diffLat + diffLon * diffLon);
}
