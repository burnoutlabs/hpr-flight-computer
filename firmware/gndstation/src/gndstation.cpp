/**
 * @brief File: gndstation.ino
 *
 * This file contains the setup and loop functions for the rocket's control system.
 *
 * Versions:
 * - V7 uses only a single microprocessor
 * - V8 uses an optimized data stream
 * - V9 overhauls the whole system
 * - V10 eliminates the RadioHead library
 * - V10_1 finalizes functionality to use two radios in tandem
 * - V10_2 fixes some bugs and adds a distance calculation to the LCD screen
 *
 * TO DO:
 * 0) Read user data from SD card
 * 1) Create BLE Service
 * 2) Pin correction for LCD - OK
 * 3) Radio 1 controls - OK
 * 4) Radio 2 controls - OK
 * 5) GPS Services - OK
 * 6) LSM9DS1 orientation
 * 7) LSM9DS1 magnetic
 * 8) Include Software SPI - OK
 * 9) Create user menu interface
 * 10) Delineate FHSS with a booster
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <Wire.h>

#include "declarations.h"
#include "gndstation.h"
#include "GPS.h"
#include "LCD.h"
#include "Parse_Packet.h"
#include "Radio.h"
#include "SD_Card.h"

// Hardware Serial for GPS
HardwareSerial *HWSERIAL;

// Radio control pins
#define RADIO1_RST 14 // ok
#define RADIO1_CS 15  // ok
#define RADIO1_IRQ 17 // ok
#define RADIO2_RST 16 // ok
#define RADIO2_CS 20  // ok
#define RADIO2_IRQ 22 // ok

// GPS Setup
TinyGPSPlus gps;

// Timer for the radio
IntervalTimer radioTimer;

void setup()
{

  // start SPI
  SPI.begin();

  // Primary Serial Port
  Serial.begin(9600);
  Serial2.begin(9600);

  // Read the battery voltage
  pinMode(A0, INPUT);
  battVolt = 0;
  for (byte i = 0; i < 5; i++)
  {
    battVolt += analogRead(A10);
    delay(10);
  }
  battVolt /= 5;

  // Set interrupts
  pinMode(RADIO1_IRQ, INPUT);
  attachInterrupt(digitalPinToInterrupt(RADIO1_IRQ), clearIrq1, RISING);
  pinMode(RADIO2_IRQ, INPUT);
  attachInterrupt(digitalPinToInterrupt(RADIO2_IRQ), clearIrq2, RISING);

  // Start SD card
  sdInit = SD.begin(SD_CONFIG);
  if (!sdInit)
  {
    Serial.println(F("SD card failed"));
  }
  else
  {
    Serial.println(F("SD Card OK!"));
  }
  // Read user settings
  readRadioSettingsSd();

  // set the radio pointers
  radio1.num = 1;
  radio2.num = 2;
  radio1.cs = RADIO1_CS;
  radio2.cs = RADIO2_CS;
  radio1.rst = RADIO1_RST;
  radio2.rst = RADIO2_RST;

  // set radio pins
  pinMode(radio1.cs, OUTPUT);
  pinMode(radio1.rst, OUTPUT);
  pinMode(radio2.cs, OUTPUT);
  pinMode(radio2.rst, OUTPUT);
  digitalWrite(radio1.cs, HIGH);
  digitalWrite(radio1.rst, HIGH);
  digitalWrite(radio2.cs, HIGH);
  digitalWrite(radio2.rst, HIGH);

  // Radio1 setup
  delay(500);
  activeRadio = &radio1;
  activeRadio->status = radioBegin();
  if (!activeRadio->status)
  {
    Serial.print("Radio ");
    Serial.print(activeRadio->num);
    Serial.println(" Failed");
  }
  else
  {
    Serial.print("Radio");
    Serial.print(activeRadio->num);
    Serial.println(" OK!");
  }
  // if the radio is active
  if (activeRadio->status)
  {
    // set frequency
    if (setRadioFreq(activeRadio->frq))
    {
      Serial.print("Radio");
      Serial.print(activeRadio->num);
      Serial.print(" on ");
      Serial.print(activeRadio->frq, 3);
      Serial.println(" MHz");
    }
    else
    {
      Serial.print("Radio");
      Serial.print(activeRadio->num);
      Serial.println(" Set Frequency Failed");
    }
    // set to recieve mode
    radioSetMode(RX_MODE);
  }

  // Radio2 setup
  delay(500);
  activeRadio = &radio2;
  activeRadio->status = radioBegin();
  if (!activeRadio->status)
  {
    Serial.print("Radio ");
    Serial.print(activeRadio->num);
    Serial.println(" Failed");
  }
  else
  {
    Serial.print("Radio");
    Serial.print(activeRadio->num);
    Serial.println(" OK!");
  }
  // if the radio is active
  if (activeRadio->status)
  {
    // set frequency
    if (setRadioFreq(activeRadio->frq))
    {
      Serial.print("Radio");
      Serial.print(activeRadio->num);
      Serial.print(" on ");
      Serial.print(activeRadio->frq, 3);
      Serial.println(" MHz");
    }
    else
    {
      Serial.print("Radio");
      Serial.print(activeRadio->num);
      Serial.println(" Set Frequency Failed");
    }
    // set to recieve mode
    radioSetMode(RX_MODE);
  }

  // set active radio based on user settings
  if (radio1.enable & !radio2.enable)
  {
    activeRadio = &radio1;
  }
  else if (!radio1.enable & radio2.enable)
  {
    activeRadio = &radio2;
  }
  else
  {
    activeRadio = &radio1;
  }

  /* EEPROM Allocations:
   Posn 00-19: Sustainer Name
   Posn 20-23: Latitude Float
   Posn 24-27: Longitude Float
   Posn    28: Radio1 Freq
   Posn 29-48: Booster Name
   Posn 49-52: Latitude Float
   Posn 53-56: Longitude Float
   Posn    57: Radio2 Freq
   */

  // read EEPROM for the rocket name
  for (byte i = 0; i < sizeof(rocketName); i++)
  {
    rocketName[i] = EEPROM.read(i);
  }
  // Read EEPROM for the sustainer last GPS coordinates
  for (byte i = 0; i < 4; i++)
  {
    radioUnion.unionByte[i] = EEPROM.read(20 + i);
  }
  lastGpsLat = radioUnion.gpsCoord;
  if (fabs(lastGpsLat) > 90)
  {
    lastGpsLat = 0.0;
  }
  for (byte i = 0; i < 4; i++)
  {
    radioUnion.unionByte[i] = EEPROM.read(24 + i);
  }
  lastGpsLon = radioUnion.gpsCoord;
  if (fabs(lastGpsLon) > 180)
  {
    lastGpsLon = 0.0;
  }

  // configure GPS
  configGps();

  // Display the startup screen
  Serial.println("starting LCD");
  startupLcd();

} // end setup

void loop()
{

  boolean msgRx = false;

  // Serial Print Debug
  if (micros() - timer > 1000000UL && micros() - timeLastNmea > 500000)
  {
    timer = micros();
    Serial.print(F("Waiting for Packet, frequency "));
    Serial.print(readRadioFreq(), 3);
    Serial.println(" MHz");
  }

  // check radios for recieved packet with interrupt
  if (radio1Pkt || radio2Pkt)
  {

    // select the radio that recieved a packet, if both do radio1 first
    if (radio1Pkt)
    {
      activeRadio = &radio1;
    }
    else
    {
      activeRadio = &radio2;
    }

    // set the timers
    lastRx = micros();
    activeRadio->lastRx = lastRx;
    timer = lastRx;

    // get the packet from the radio
    bool pktRx = radioRecvPkt(activeRadio->dataPacket);

    // get signal strength
    signalStrength = pktRssi;

    // if it is from our flight computer, parse it, log it, display it and follow the frequency hops
    if (pktRx)
    {
      processPacket();
    }

    if (activeRadio->num == 1)
    {
      radio1Pkt = false;
    }
    else
    {
      radio2Pkt = false;
    }
  } // end check radios

  // On FHSS, if signal was established on the ground but a packet was missed, then move to the next channel
  if (event == 0 && radio1.fhss && micros() - radio1.lastHopTime > 2000000UL && radio1.signalEst && radio1.chnlUsed < 3)
  {
    Serial.println("-------Radio 1 Missed Packet--------");
    activeRadio = &radio1;
    // signal to move to the synch freq because the necessary lead time on the ground is too long to try and catch the next packet
    activeRadio->chnlUsed = 3;
    hopFreq();
  }

  // On FHSS, if signal was established in-flight but a packet was missed, then move to the next channel
  else if (radio1.fhss && micros() - radio1.lastHopTime > 630000UL && radio1.signalEst && radio1.chnlUsed < 3)
  {
    Serial.println("-------Radio 1 Missed Packet--------");
    activeRadio = &radio1;
    hopFreq();
  }

  if (event == 0 && radio2.fhss && micros() - radio2.lastHopTime > 2000000UL && radio2.signalEst && radio2.chnlUsed < 3)
  {
    Serial.println("-------Radio 1 Missed Packet--------");
    activeRadio = &radio2;
    // signal to move to the synch freq because the necessary lead time on the ground is too long to try and catch the next packet
    activeRadio->chnlUsed = 3;
    hopFreq();
  }

  else if (radio2.fhss && micros() - radio2.lastHopTime > 630000UL && radio2.signalEst && radio2.chnlUsed < 3)
  {
    Serial.println("-------Radio 2 Missed Packet--------");
    activeRadio = &radio2;
    hopFreq();
  }

  // display lost signal status
  if (micros() - lastRx > lostSignalTime && signalEst)
  {

    if (debugSerial)
    {
      Serial.println(F("Signal Lost!"));
    }

    /* EEPROM Allocations:
    Posn 00-19: Sustainer Name
    Posn 20-23: Latitude Float
    Posn 24-27: Longitude Float
    Posn    28: Radio1 Freq
    Posn 29-48: Booster Name
    Posn 49-52: Latitude Float
    Posn 53-56: Longitude Float
    Posn    57: Radio2 Freq
    */

    // Write the last GPS coordinates to EEPROM for later display
    for (byte i = 0; i < sizeof(rocketName); i++)
    {
      EEPROM.update(i, rocketName[i]);
    }
    radioUnion.gpsCoord = lastGpsLat;
    for (byte i = 0; i < 4; i++)
    {
      EEPROM.update(i + 20, radioUnion.unionByte[i]);
    }
    radioUnion.gpsCoord = lastGpsLon;
    for (byte i = 0; i < 4; i++)
    {
      EEPROM.update(i + 24, radioUnion.unionByte[i]);
    }

    // sync the SD card
    if (sdInit && sustainerFileOpen)
    {
      sustainerFile.sync();
    }

    // display last coordinates on LCD
    signalLostLcd();

    signalEst = false;
  }

  // GPS
  while (Serial2.available() > 0)
  {
    if (!msgRx)
    {
      msgRx = true;
    }
    char c = Serial2.read();
    gps.encode(c);
    if (gpsDebug)
    {
      serialBuffer[serialPosn] = c;
      serialPosn++;
    }
  }
  if (gpsDebug && msgRx)
  {
    serialBuffer[serialPosn] = '\0';
    Serial.print(serialBuffer);
    msgRx = false;
    serialPosn = 0;
    timeLastNmea = micros();
  }

  // check LED
  if (ledLight && micros() - colorStart > 2000000)
  {
    byte bright = whiteIntensity;
    lcd.setBacklight(bright, bright, bright); // Set backlight to white
    ledLight = false;
  }

} // end loop

void clearIrq1()
{
  noInterrupts();
  radio1Pkt = true;
  interrupts();
}

void clearIrq2()
{
  noInterrupts();
  radio2Pkt = true;
  interrupts();
}
