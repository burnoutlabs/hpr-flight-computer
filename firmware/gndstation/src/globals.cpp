/**
 * @file globals.cpp
 * @brief Definitions of the ground station's global variables.
 *
 * @details
 * declarations.h declares these extern for every source file; this is the one place they
 * are defined, with their initial values.
 */
#include <Arduino.h>

#include "declarations.h"

char serialBuffer[256];
int serialPosn = 0;

byte dataPkt[80];
volatile boolean processIrq = false;
uint32_t timeLastPkt = 0UL;
uint32_t debugTime;
int16_t pktRssi;
float radio1Freq = 433.500;
float radio2Freq = 902.300;

byte radio1Function = RX_MODE;
byte radio2Function = RX_MODE;

//---------------------------
// User Settings
//---------------------------
alignas(4) UserSettings settings;
//---------------------------
// Radio Control variables
//---------------------------
Radio radio1;
Radio radio2;
Radio *activeRadio;
//---------------------------
// Data Packet variables
//---------------------------
uint8_t len;
// dataString is defined in SD_Card.cpp
boolean sdStatus = false;
char sustainerFileName[13] = "FLIGHT01.txt";
char boosterFileName[14] = "BOOSTER01.txt";
boolean sustainerFileOpen = false;
boolean boosterFileOpen = false;
boolean sustainerFileCreated = false;
boolean boosterFileCreated = false;
byte fileNum = 1;
byte pktPosn = 0;
int16_t lastPacketNum = 0;
unsigned long timer = 0UL;
boolean radio1status = false;
boolean radio2status = false;
boolean sendPacket = true;
boolean testMode = false;
unsigned long testStart = 0UL;
unsigned long lastRx = 0UL;
unsigned long debugStart = 0UL;
unsigned long debugStop = 0UL;
unsigned long colorStart;
int battVolt;
boolean sdInit = false;
boolean lcdInit = true;
boolean parseSustainer = false;
boolean parseBooster = false;
uint32_t timeLastNmea = 0UL;
bool ledLight = false;
//---------------------------
// preflight output variables
//---------------------------
uint8_t event;
// strPosn is defined in SD_Card.cpp
boolean signalEst = false;
boolean dataProcessed = false;
boolean sustainerPreFlightWrite = true;
boolean boosterPreFlightWrite = true;
boolean sustainerPostFlightWrite = true;
boolean boosterPostFlightWrite = true;
boolean boosterFileReady = false;
unsigned long lostSignalTime = 2000000UL;
//---------------------------
// preflight output variables
//---------------------------
alignas(4) char rocketName[20] = "";
int baseAlt = 0;
int baseGpsAlt = 0;
byte contCode;
int satNum = 0;
boolean fileOpen = true;
//---------------------------
// inflight output variables
//---------------------------
uint16_t sampleTime;
int16_t signalStrength = 0;
int16_t velocity = 0;
int16_t alt = 0;
int16_t spin = 0;
int16_t offVert = 0;
int16_t accel = 0;
int16_t packetNum = 0;
boolean apogee = false;
unsigned long lastSustainerRx = 0UL;
unsigned long lastBoosterRx = 0UL;
//---------------------------
// Postflight output variables
//---------------------------
int maxAltitude = 0;
int maxVelocity = 0;
int maxG = 0;
int maxGpsAlt = 0;
// GPS output variables
byte gpsLock;
int gpsAlt = 0;
int prevGpsAlt;
char charGpsLat;
float gpsLatitude;
char charGpsLon;
float gpsLongitude;
float lastGpsLat;
float lastGpsLon;
float prevGpsLat = 0.0;
float prevGpsLon = 0.0;
unsigned long lastGpsFix = 0UL;
//---------------------------
// Radio Variables
//---------------------------
boolean gpsDebug = false;
boolean debugSerial = true;
float unitConvert = 3.28084F;
volatile boolean radio1Pkt = false;
volatile boolean radio2Pkt = false;
RadioUnion radioUnion;
RadioInt radioInt;
byte radioFunction;
//---------------------------
// LCD Variables
//---------------------------
byte whiteIntensity = 125;
byte redIntensity = 255;
byte greenIntensity = 255;
byte blueIntensity = 255;
byte flightPhase = 0;
//---------------------------
// Event Table
//---------------------------
const String EVENT_00 = "Preflight";
const String EVENT_01 = "Liftoff";
const String EVENT_02 = "Booster Burnout";
const String EVENT_03 = "Apogee Detected";
const String EVENT_04 = "Firing Apogee Pyro";
const String EVENT_05 = "Separation Detected!";
const String EVENT_06 = "Firing Mains";
const String EVENT_07 = "Under Chute";
const String EVENT_08 = "Ejecting Booster";
const String EVENT_09 = "Firing 2nd Stage";
const String EVENT_10 = "2nd Stage Ignition";
const String EVENT_11 = "2nd Stage Burnout";
const String EVENT_12 = "Firing Airstart1";
const String EVENT_13 = "Airstart1 Ignition";
const String EVENT_14 = "Airstart1 Burnout";
const String EVENT_15 = "Firing Airstart2";
const String EVENT_16 = "Airstart2 Ignition";
const String EVENT_17 = "Airstart2 Burnout";
const String EVENT_18 = "NoFire: Rotn Limit";
const String EVENT_19 = "NoFire: Alt Limit";
const String EVENT_20 = "NoFire: Rotn/Alt Lmt";
const String EVENT_21 = "Booster Apogee";
const String EVENT_22 = "Booster Apogee Fire";
const String EVENT_23 = "Booster Separation!";
const String EVENT_24 = "Booster Main Deploy";
const String EVENT_25 = "Booster Under Chute";
const String EVENT_26 = "Time Limit Exceeded";
const String EVENT_27 = "Touchdown!";
const String EVENT_28 = "Power Loss! Restart!";
const String EVENT_29 = "Booster Touchdown!";
const String EVENT_30 = "Booster Preflight";
const String EVENT_31 = "Booster Time Limit";
const String EVENT_32 = "Booster Pwr Restart";

const String EVENT_TABLE[] = {
    EVENT_00, EVENT_01, EVENT_02, EVENT_03, EVENT_04,
    EVENT_05, EVENT_06, EVENT_07, EVENT_08, EVENT_09,
    EVENT_10, EVENT_11, EVENT_12, EVENT_13, EVENT_14,
    EVENT_15, EVENT_16, EVENT_17, EVENT_18, EVENT_19,
    EVENT_20, EVENT_21, EVENT_22, EVENT_23, EVENT_24,
    EVENT_25, EVENT_26, EVENT_27, EVENT_28, EVENT_29,
    EVENT_30, EVENT_31, EVENT_32};

byte greenEvents[] = {5, 6, 7};
byte redEvents[] = {18, 19, 20, 26, 28};

//---------------------------
// Pyro Code Table
//---------------------------
const String CONT_0 = "No Pyros Detected!";
const String CONT_1 = "No Continuity Pyro 1";
const String CONT_2 = "No Continuity Pyro 2";
const String CONT_3 = "No Continuity Pyro 3";
const String CONT_4 = "No Continuity Pyro 4";
const String CONT_5 = "All 3 Pyros Detected";
const String CONT_6 = "All 4 Pyros Detected";
const String CONT_7 = "Pyro Apogee Only";
const String CONT_8 = "Pyro Mains Only";
const String CONT_9 = "Pyro Mains & Apogee";

const String PYRO_TABLE[] = {
    CONT_0, CONT_1, CONT_2, CONT_3, CONT_4,
    CONT_5, CONT_6, CONT_7, CONT_8, CONT_9};

byte greenCont[5] = {5, 6, 7, 8, 9};
byte redCont[5] = {0, 1, 2, 3, 4};
