/**
 * @file declarations.h
 * @brief Types and extern declarations of the globals shared by the ground station sources.
 *
 * @details
 * Holds the radio register modes, the settings and radio structs, and the packet, SD, GPS
 * and LCD variables. The variables are defined, with their initial values, in globals.cpp;
 * the hardware objects (SD card, LCD, GPS, timers) are defined in the file that drives them.
 */
#pragma once

#include <Arduino.h>

// REG_OP_MODE
enum
{
  SLEEP_MODE = 0x00,
  STANDBY_MODE = 0x01,
  TX_MODE = 0x03,
  RX_MODE = 0x05,
  CAD_MODE = 0x07,
  LORA_MODE = 0x80,
  LOW_FRQ_MODE = 0x08, // LowFrq mode enables registers below 860MHz
  WRITE_MASK = 0x80
};

//---------------------------
// User Settings
//---------------------------
struct UserSettings
{
  boolean radio1Enable = true;
  boolean radio2Enable = true;
  float radio1freq;
  float radio2freq;
  boolean radio1Fhss = false;
  boolean radio2Fhss = false;
  byte radio1Pwr = 13;
  byte radio2Pwr = 13;
  boolean displayMetric = false;
  byte debugSerial = 1;
  char callSign[7] = "KK4ELF";
  boolean enableLcd = false;
  byte lcdBrightness = 75;
  byte lcdContrast = 50;
  boolean lcdColorEvents = false;
  boolean bluetooth = true;
  boolean enableSd = true;
  boolean enableGps = true;
};
alignas(4) extern UserSettings settings;
//---------------------------
// Radio Control variables
//---------------------------
typedef struct
{
  float frq;
  byte num;
  boolean enable = true;
  boolean fhss = false;
  boolean status = false;
  byte txPwr = 13;
  byte dataPacket[256];
  boolean signalEst = false;
  uint32_t lastRx = 0UL;
  uint8_t cs;
  uint8_t rst;
  uint8_t hailChnl = 0;
  uint8_t chnl1;
  uint8_t nextChnl;
  int16_t nextHop = 0;
  uint8_t chnlUsed = 0;
  boolean syncFreq = true;
  uint32_t lastHopTime = 0UL;
} Radio;
extern Radio radio1;
extern Radio radio2;
extern Radio *activeRadio;
extern char serialBuffer[256];
extern int serialPosn;

extern byte dataPkt[80];
extern volatile boolean processIrq;
extern uint32_t timeLastPkt;
extern uint32_t debugTime;
extern int16_t pktRssi;
extern float radio1Freq;
extern float radio2Freq;
extern byte radio1Function;
extern byte radio2Function;
//---------------------------
// Data Packet variables
//---------------------------
extern uint8_t len;
alignas(4) extern char dataString[512];
extern boolean sdStatus;
extern char sustainerFileName[13];
extern char boosterFileName[14];
extern boolean sustainerFileOpen;
extern boolean boosterFileOpen;
extern boolean sustainerFileCreated;
extern boolean boosterFileCreated;
extern byte fileNum;
extern byte pktPosn;
extern int16_t lastPacketNum;
extern unsigned long timer;
extern boolean radio1status;
extern boolean radio2status;
extern boolean sendPacket;
extern boolean testMode;
extern unsigned long testStart;
extern unsigned long lastRx;
extern unsigned long debugStart;
extern unsigned long debugStop;
extern unsigned long colorStart;
extern int battVolt;
extern boolean sdInit;
extern boolean lcdInit;
extern boolean parseSustainer;
extern boolean parseBooster;
extern uint32_t timeLastNmea;
extern bool ledLight;
//---------------------------
// preflight output variables
//---------------------------
extern uint8_t event;
extern int strPosn;
extern boolean signalEst;
extern boolean dataProcessed;
extern boolean sustainerPreFlightWrite;
extern boolean boosterPreFlightWrite;
extern boolean sustainerPostFlightWrite;
extern boolean boosterPostFlightWrite;
extern boolean boosterFileReady;
extern unsigned long lostSignalTime;
//---------------------------
// preflight output variables
//---------------------------
alignas(4) extern char rocketName[20];
extern int baseAlt;
extern int baseGpsAlt;
extern byte contCode;
extern int satNum;
extern boolean fileOpen;
//---------------------------
// inflight output variables
//---------------------------
extern uint16_t sampleTime;
extern int16_t signalStrength;
extern int16_t velocity;
extern int16_t alt;
extern int16_t spin;
extern int16_t offVert;
extern int16_t accel;
extern int16_t packetNum;
extern boolean apogee;
extern unsigned long lastSustainerRx;
extern unsigned long lastBoosterRx;
//---------------------------
// Postflight output variables
//---------------------------
extern int maxAltitude;
extern int maxVelocity;
extern int maxG;
extern int maxGpsAlt;
// GPS output variables
extern byte gpsLock;
extern int gpsAlt;
extern int prevGpsAlt;
extern char charGpsLat;
extern float gpsLatitude;
extern char charGpsLon;
extern float gpsLongitude;
extern float lastGpsLat;
extern float lastGpsLon;
extern float prevGpsLat;
extern float prevGpsLon;
extern unsigned long lastGpsFix;
//---------------------------
// Radio Variables
//---------------------------
extern boolean gpsDebug;
extern boolean debugSerial;
extern float unitConvert;
extern volatile boolean radio1Pkt;
extern volatile boolean radio2Pkt;
union RadioUnion
{
  float gpsCoord;
  unsigned long radioTime;
  byte unionByte[4];
};
extern RadioUnion radioUnion;
union RadioInt
{
  int16_t unionInt;
  byte unionByte[2];
};
extern RadioInt radioInt;
extern byte radioFunction;
//---------------------------
// LCD Variables
//---------------------------
extern byte whiteIntensity;
extern byte redIntensity;
extern byte greenIntensity;
extern byte blueIntensity;
extern byte flightPhase;
//---------------------------
// Event Table
//---------------------------

extern const String EVENT_TABLE[33];

extern byte greenEvents[3];
extern byte redEvents[5];

//---------------------------
// Pyro Code Table
//---------------------------

extern const String PYRO_TABLE[10];

extern byte greenCont[5];
extern byte redCont[5];
