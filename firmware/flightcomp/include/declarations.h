/**
 * @file declarations.h
 * @brief Types and extern declarations of the globals shared by the flight computer sources.
 *
 * @details
 * Holds the EEPROM address map, the settings, sensor, pin, and flight-state structs, the
 * radio register and packet variables, and the device function pointers that setPolyMorphs()
 * (Device_Mgmt.cpp) assigns at startup. The variables are defined, with their initial
 * values, in globals.cpp.
 */
#pragma once

#include <Arduino.h>

// alignas(4) on a byte-aligned array, struct or union: while every global was defined in the
// one translation unit, GCC gave these word alignment (its -O2 data alignment on ARM) and
// generated code that relies on it. Most files now see only the extern declaration, so the
// alignment is stated here, and on the definition, to keep that code the same.

// Header file for all major declared variables
//-----------------------------------------------
// Written by SparkyVT, TRA #12111, NAR #85720, L3
//-----------Change Log--------------------------
// 29 Nov 23: Broke out a separate header file to make reading the main file easier
//-----------------------------------------
// EEPROM Locations
//-----------------------------------------
struct EepromMap
{
  int maxFltAlt = 0;  // 00-05: maximum altitude of last flight
  int accelBiasX = 6; // 06-11: accel.bias(X,Y,Z)
  int accelBiasY = 8;
  int accelBiasZ = 10;
  int highGBiasX = 12; // 12-17: highG.bias(X,Y,Z)
  int highGBiasY = 14;
  int highGBiasZ = 16;
  int magBiasX = 18; // 18-23: mag.bias(X,Y,Z)
  int magBiasY = 20;
  int magBiasZ = 22;
  int accelXSign = 24; // 24-29: accelerometer to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
  int accelXPtr = 25;
  int accelYSign = 26;
  int accelYPtr = 27;
  int accelZSign = 28;
  int accelZPtr = 29;
  int gyroXSign = 30; // 30-35: gyro to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
  int gyroXPtr = 31;
  int gyroYSign = 32;
  int gyroYPtr = 33;
  int gyroZSign = 34;
  int gyroZPtr = 35;
  int magXSign = 36; // 36-41: magnetometer to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
  int magXPtr = 37;
  int magYSign = 38;
  int magYPtr = 39;
  int magZSign = 40;
  int magZPtr = 41;
  int highGXSign = 42; // 42-47: highG to world orientation (x-sign,x-pointer,y-sign,y-pointer,z-sign,z-pointer)
  int highGXPtr = 43;
  int highGYSign = 44;
  int highGYPtr = 45;
  int highGZSign = 46;
  int highGZPtr = 47;
  int accel2BoardXAxis = 48; // 48-53: accelerometer to main board orientation (x-sign,x-axis,y-sign,y-axis,z-sign,z-axis)
  int accel2BoardXSign = 49;
  int accel2BoardYAxis = 50;
  int accel2BoardYSign = 51;
  int accel2BoardZAxis = 52;
  int accel2BoardZSign = 53;
  int gyro2BoardXAxis = 54; // 54-59: gyro to main board orientation (x-sign,x-axis,y-sign,y-axis,z-sign,z-axis)
  int gyro2BoardXSign = 55;
  int gyro2BoardYAxis = 56;
  int gyro2BoardYSign = 57;
  int gyro2BoardZAxis = 58;
  int gyro2BoardZSign = 59;
  int mag2BoardXAxis = 60; // 60-65: magnetometer to main board orientation (x-sign,x-axis,y-sign,y-axis,z-sign,z-axis)
  int mag2BoardXSign = 61;
  int mag2BoardYAxis = 62;
  int mag2BoardYSign = 63;
  int mag2BoardZAxis = 64;
  int mag2BoardZSign = 65;
  int highG2BoardXAxis = 66; // 66-71: high-G accelerometer to main board orientation (x-sign,x-axis,y-sign,y-axis,z-sign,z-axis)
  int highG2BoardXSign = 67;
  int highG2BoardYAxis = 68;
  int highG2BoardYSign = 69;
  int highG2BoardZAxis = 70;
  int highG2BoardZSign = 71;
  int accelBusType = 72; // 72-93: device bus info
  int accelBusNum = 73;
  int gyroBusType = 74;
  int gyroBusNum = 75;
  int magBusType = 76;
  int magBusNum = 77;
  int highGBusType = 78;
  int highGBusNum = 79;
  int baroPressOffset = 80; // 80-83 : barometer pressure offset, stays in this position so I don't have to recalibrate all my flight computers
  int baroTempOffset = 84;  // 84-87  : barometer temperature offset, stays in this position so I don't have to recalibrate all my flight computers
  int baroBusType = 88;
  int baroBusNum = 89;
  int radioBusType = 90;
  int radioBusNum = 91;
  int sdBusType = 92;
  int sdBusNum = 93;
  int gpsBusType = 94;
  int gpsBusNum = 95;
  int pyro1ContPin = 96;     // 96     : Pyro 1 Continuity Pin
  int pyro1FirePin = 97;     // 97     : Pyro 1 Fire Pin
  int pyro2ContPin = 98;     // 98     : Pyro 2 Continuity Pin
  int pyro2FirePin = 99;     // 98     : Pyro 2 Fire Pin
  int pyro3ContPin = 100;    // 100   : Pyro 3 Continuity Pin
  int pyro3FirePin = 101;    // 101   : Pyro 3 Fire Pin
  int pyro4ContPin = 102;    // 102   : Pyro 4 Continuity Pin
  int pyro4FirePin = 103;    // 103   : Pyro 4 Fire Pin
  int nullPin = 104;         // 104        : Null Continuity / Fire Pin
  int beepPin = 105;         // 105        : Beeper Pin
  int battReadPin = 106;     // 106    : Battery Read Pin
  int testModeGndPin = 107;  // 107 : Test Mode Button Gnd Pin
  int testModeReadPin = 108; // 108   : Test Mode Read Pin
  int radioCsPin = 109;      // 109     : radio CS pin
  int radioIrqPin = 110;     // 110    : radio interrupt pin
  int radioRstPin = 111;     // 111    : radio reset pin
  int radioEnPin = 112;      // 112     : radio enable pin (optional w/ Adafruit breakout only)
  int accelCsPin = 113;
  int gyroCsPin = 114;
  int magCsPin = 115;
  int highGCsPin = 116;
  int sdCsPin = 117;
  int baroCsPin = 118;
  int servo1Pin = 119; // 119   : servo 1 control pin
  int servo2Pin = 120; // 120   : servo 2 control pin
  int servo3Pin = 121; // 121   : servo 3 control pin
  int servo4Pin = 122; // 122   : servo 4 control pin
  int servo5Pin = 123; // 123   : servo 5 control pin
  int servo6Pin = 124; // 124   : servo 6 control pin
  int servo7Pin = 125; // 125   : servo 7 control pin
  int servo8Pin = 126; // 126   : servo 8 control pin
  int accelId = 127;   // 127   : Sensor ID accel
  int gyroId = 128;    // 128   : Sensor ID gyro
  int highGId = 129;   // 129   : Sensor ID highG
  int baroId = 130;    // 130   : Sensor ID baro
  int radioId = 131;   // 131   : Sensor ID Radio
  int gpsId = 132;     // 132   : Sensor ID GPS
  int magId = 133;
  int sdId = 134;
  int hwVersion = 135;
  int hwSubVersion = 136;
  int hwUnitNum = 137;
  int callSign = 138; // 138-143: Ham Radio Callsign
  //-------------------------------------------------------------
  // 144-227: retired copy of the flight settings. Its fields overlapped, and nothing read it;
  // setup() stopped writing it, except inflightRecover (186), which setup() reads before the SD
  // card is started. The addresses stay reserved: boards in the field hold old values there.
  //-------------------------------------------------------------
  int rocketName = 144; // Maximum of 20 characters
  int fltProfile = 164;
  int units = 165;
  int reportStyle = 166;
  int setupTime = 167;         // 4
  int gTrigger = 171;          // 2
  int detectLiftoffTime = 173; // 4
  int mainDeployAlt = 177;     // 2
  int rcdTime = 179;           // 4
  int apogeeDelay = 183;
  int silentMode = 184;
  int magSwitchEnable = 185;
  int inflightRecover = 186;
  int gpsLogFile = 187;
  int fireTime = 188; // 4
  int pyro4Func = 192;
  int pyro3Func = 193;
  int pyro2Func = 194;
  int pyro1Func = 195;
  int txEnable = 196;
  int txPwr = 197;
  int txFreq = 198; // 4
  int fhss = 202;
  int boosterSeparationDelay = 203; // 4
  int sustainerFireDelay = 207;     // 4
  int airStart1Event = 211;
  int airStart1Delay = 212; // 4
  int airStart2Event = 216;
  int airStart2Delay = 217; // 2
  int altThreshold = 219;   // 2
  int maxAngle = 221;
  int stableRotn = 222;
  int stableVert = 223;
  int flyback = 224;
  int serialDebug = 225;
  int lastFile = 226; // 2
  int baseAlt = 228;  // 4
  int lastEvent = 232;
  //-------------------------------------------------------------
  // active stabilization parameters
  //-------------------------------------------------------------
  int servo1Trim = 233;
  int servo2Trim = 234;
  int servo3Trim = 235;
  int servo4Trim = 236;
  int servo5Trim = 237;
  int servo6Trim = 238;
  int servo7Trim = 239;
  int servo8Trim = 240;
  int kp = 241;           // 4
  int ki = 245;           // 4
  int kd = 249;           // 4
  int totalFlights = 253; // 2
  int gyroBiasX = 255;    // 2
  int gyroBiasY = 257;    // 2
  int gyroBiasZ = 259;    // 2
};
// Never written: const gives it internal linkage, so each address folds to a constant
const EepromMap eeprom{};
//-----------------------------------------
// Sensor device data
//-----------------------------------------
struct SensorConfig
{
  byte accel = 0;
  byte mag = 0;
  byte gyro = 0;
  byte highG = 0;
  byte baro = 0;
  byte radio = 0;
  byte gnss = 0;
  char accelBusType;
  byte accelBusNum;
  char gyroBusType;
  byte gyroBusNum;
  char magBusType;
  byte magBusNum;
  char baroBusType;
  byte baroBusNum;
  char highGBusType;
  byte highGBusNum;
  char radioBusType;
  byte radioBusNum;
  char gnssBusType;
  byte gnssBusNum;
  char sdBusType;
  byte sdBusNum;
  bool statusAccel = false;
  bool statusGyro = false;
  bool statusHighG = false;
  bool statusBaro = false;
  bool statusMag = false;
  bool statusRadio = false;
  bool pyroPwm = false;
};
alignas(4) extern SensorConfig sensors;
//-----------------------------------------
// User Settings
//-----------------------------------------
struct UserSettings
{
  bool testMode = false;
  bool calibrationMode = false;
  //--------flight settings----------------
  char rocketName[20] = ""; // at most 19 characters and the terminator
  char hwId[7] = "";
  char fltProfile = 'S';
  char units = 'S';
  char reportStyle = 'P';
  bool gpsFileEnable = false;
  unsigned long setupTime = 5000000UL;
  float mainDeployAlt = 153;             // Up to 458m for main deploy
  unsigned long rcdTime = 900000000UL;   // 15min
  unsigned long apogeeDelay = 1000000UL; // 1.0s apogee delay
  bool silentMode = false;               // true turns off beeper
  bool magSwitchEnable = false;
  byte inflightRecover = 0;
  bool gpsLog = false;
  byte serialDebug = 1;
  //--------pyro settings----------------
  unsigned long fireTime = 500000UL; // 0.5s
  char pyro4Func = 'M';
  char pyro3Func = 'A';
  char pyro2Func = 'N';
  char pyro1Func = 'N';
  //--------telemetry settings----------------
  char callSign[7] = "";
  bool txEnable = true; // false turns off radio transmissions
  byte txPwr = 13;
  float txFreq = 433.250;
  bool fhss = false;
  //--------2 stage settings----------------
  unsigned long boosterSeparationDelay = 500000UL; // 0.5s
  unsigned long sustainerFireDelay = 1000000UL;    // 1.0s
  //--------airstart settings----------------
  char airStart1Event = 'B';
  unsigned long airStart1Delay = 500000UL;
  char airStart2Event = 'B';
  unsigned long airStart2Delay = 500000UL;
  //-------safety thresholds----------------
  int altThreshold = 120; // 120m = 400ft
  int maxAngle = 45;      // degrees
  //-------active stabilization----------------
  bool stableRotn = false;
  bool stableVert = false;
  //----------flyback option--------------------
  bool flyBack = false;
};
extern UserSettings settings;
//-----------------------------------------
// Sensor variables
//-----------------------------------------
// rawData, the bus read buffer, is declared in Bus_Mgmt.h
typedef struct
{
  byte addr;
  int16_t adcMax;
  float gainX;
  float gainY;
  float gainZ;
  int16_t rawX;
  int16_t rawY;
  int16_t rawZ;
  int8_t dirX;
  int8_t dirY;
  int8_t dirZ;
  char orientX;
  char orientY;
  char orientZ;
  int16_t *ptrX;
  int16_t *ptrY;
  int16_t *ptrZ;
  int8_t *ptrXSign;
  int8_t *ptrYSign;
  int8_t *ptrZSign;
  int32_t sumX0 = 0L;
  int32_t sumY0 = 0L;
  int32_t sumZ0 = 0L;
  int16_t x0;
  int16_t y0;
  int16_t z0;
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t biasX;
  int16_t biasY;
  int16_t biasZ;
  uint32_t timeLastSamp = 0UL;
  uint32_t timeBtwnSamp = 10000UL;
  boolean newSamp = false;
} SensorData;
extern SensorData accel;
extern SensorData mag;
extern SensorData gyro;
extern SensorData highG;
//-----------------------------------------
// pyro variables
//-----------------------------------------
typedef struct
{
  char func;
  bool fireStatus = false;
  bool contStatus = false;
  unsigned long fireStart;
  uint8_t contPin;
  uint8_t firePin;
} PyroMap;
// the four pyro channels: pyros[0] is pyro 1
extern PyroMap pyros[4];
//-----------------------------------------
// flight events
//-----------------------------------------
struct EventList
{
  bool preLiftoff = true;
  bool inFlight = false;
  bool postFlight = false;
  bool liftoff = false;
  bool falseLiftoffCheck = true;
  bool boosterBurnout = false;
  bool boosterBurnoutCheck = false;
  bool boosterSeparation = false;
  bool sustainerFireCheck = false;
  bool sustainerFire = false;
  bool sustainerIgnition = false;
  bool sustainerBurnout = false;
  bool airStart1Check = false;
  bool airStart1Fire = false;
  bool airStart1Ignition = false;
  bool airStart1BurnoutCheck = false;
  bool airStart1Burnout = false;
  bool airStart2Check = false;
  bool airStart2Fire = false;
  bool airStart2Ignition = false;
  bool airStart2BurnoutCheck = false;
  bool airStart2Burnout = false;
  bool apogee = false;
  bool apogeeFire = false;
  bool apogeeSeparation = false;
  bool mainDeploy = false;
  bool touchdown = false;
  bool timeOut = false;
};
alignas(4) extern struct EventList events;
alignas(4) extern struct EventList resetEvents;
//-----------------------------------------
// Master timing variables
//-----------------------------------------
struct TimerList
{
  unsigned long liftoff = 0UL;
  unsigned long detectLiftoffTime = 500000UL; // 0.5s
  unsigned long boosterBurnout = 0UL;
  unsigned long boosterSeparation = 0UL;
  unsigned long sustainerFireCheck = 0UL;
  unsigned long sustainerFire = 0UL;
  unsigned long sustainerIgnition = 0UL;
  unsigned long sustainerBurnout = 0UL;
  unsigned long airStart1Check = 0UL;
  unsigned long airStart1Fire = 0UL;
  unsigned long airStart1Ignition = 0UL;
  unsigned long airStart1BurnoutCheck = 0UL;
  unsigned long airStart1Burnout = 0UL;
  unsigned long airStart2Check = 0UL;
  unsigned long airStart2Fire = 0UL;
  unsigned long airStart2Ignition = 0UL;
  unsigned long airStart2BurnoutCheck = 0UL;
  unsigned long airStart2Burnout = 0UL;
  unsigned long apogee = 0UL;
  unsigned long apogeeFire = 0UL;
  unsigned long apogeeSeparation = 0UL;
  unsigned long mainDeploy = 0UL;
  unsigned long touchdown = 0UL;
  unsigned long padTime = 0UL;
  unsigned long tmClock = 0UL;
  unsigned long tmClockPrev = 0UL;
  unsigned long timeCurrent = 0UL;
  unsigned long dt = 0UL;
};
extern struct TimerList fltTime;
extern struct TimerList resetFltTime;
//-----------------------------------------
// GPIO pin mapping
//-----------------------------------------
struct PinMap
{
  uint8_t nullCont;
  uint8_t nullFire;
  uint8_t beep;
  uint8_t testGnd;
  uint8_t testRead;
  uint8_t mag;
  uint8_t batt;
  uint8_t pyro1Cont;
  uint8_t pyro1Fire;
  uint8_t pyro2Cont;
  uint8_t pyro2Fire;
  uint8_t pyro3Cont;
  uint8_t pyro3Fire;
  uint8_t pyro4Cont;
  uint8_t pyro4Fire;
  uint8_t radioCs;
  uint8_t radioIrq;
  uint8_t radioRst;
  uint8_t radioEn;
  uint8_t servo1;
  uint8_t servo2;
  uint8_t servo3;
  uint8_t servo4;
  uint8_t servo5;
  uint8_t servo6;
  uint8_t servo7;
  uint8_t servo8;
  uint8_t accelCs;
  uint8_t gyroCs;
  uint8_t magCs;
  uint8_t highGCs;
  uint8_t baroCs;
  uint8_t sdCs;
  uint8_t firePin = 0;
};
alignas(4) extern PinMap pins;
//-----------------------------------------
// radio variables
//-----------------------------------------
alignas(4) extern uint8_t dataPacket[256];
extern bool liftoffSync;
extern unsigned long txStartTime;
extern bool implicitHdr;
extern bool syncFreq;
extern uint8_t pktPosn;
extern uint8_t pktSize;
extern uint16_t sampNum;
extern uint8_t packetSamples;
extern bool gpsTransmit;
extern bool txInProgress;
extern bool sdRadioTx;
extern uint8_t radioMode;
extern volatile boolean clearTx;
extern volatile boolean buildPkt;
extern volatile boolean syncFlag;
extern bool sendPkt;
struct PacketIntervals
{
  uint32_t preLiftoff = 1000000UL;
  uint32_t inflight = 200000UL;
  uint32_t postFlight = 10000000UL;
  uint32_t samplesPerPkt = 4UL;
};
extern PacketIntervals pktInterval;
struct RadioTelemetry
{
  int16_t packetNum = 0;
  uint8_t event = 0;
  uint16_t fltTime = 0;
  int16_t baseAlt = 0;
  int16_t alt = 0;
  int16_t accel = 0;
  int16_t vel = 0;
  int16_t roll = 0;
  int16_t offVert = 0;
  int16_t maxAlt = 0;
  int16_t maxVel = 0;
  int16_t maxGpsAlt = 0;
  int16_t maxG = 0;
  int16_t gpsAlt = 0;
  uint8_t satNum = 0;
  bool pktCallsign = false;
  uint32_t lastSampTime = 0UL;
};
extern RadioTelemetry radioTelemetry;
//-----------------------------------------
// continuity Booleans
//-----------------------------------------
struct ContinuityStatus
{
  bool apogee;
  bool main;
  bool boosterSep;
  bool upperStage;
  bool airStart1;
  bool airStart2;
  bool noFunc;
  bool error;
  uint8_t reportCode;
  uint8_t beepCode;
};
alignas(4) extern ContinuityStatus cont;
//-----------------------------------------
// Non-Event Booleans
//-----------------------------------------
extern bool rotnOk;
extern bool altOk;
extern bool beep;
extern bool pyroFire;
extern bool fileClose;
extern unsigned long boosterBurpTime;
extern float unitConvert;
//-----------------------------------------
// digital accelerometer variables
//-----------------------------------------
extern int accel1G;
extern float accelNow;
extern float maxG;
extern int gTrigger; // 2.5G trigger
//-----------------------------------------
// High-G accelerometer variables
//-----------------------------------------
extern int high1G;
extern bool filterFull;
extern long highGSum;
extern int highGFilter[30];
extern uint8_t sizeHighGFilter;
extern uint8_t filterPosn;
extern float highGSmooth;
extern float adcConvert;
extern uint16_t adcMidValue;
union CalUnion
{
  int16_t calValue;
  uint8_t calByte[2];
};
alignas(4) extern CalUnion calUnion;
//-----------------------------------------
// Altitude & Baro Sensor variables
//-----------------------------------------
struct BaroData
{
  float rawAlt = 0.0F;
  float alt = 0.0F;
  float baseAlt = 10.0F;
  float maxAlt = 0.0F;
  float smoothAlt = 0.0F;
  float vel = 0.0F;
  float maxVel = 0.0F;
  float pressure;
  float temperature;
  unsigned long timeLastSamp = 0UL;
  unsigned long timeBtwnSamp = 50000UL;
  bool newSamp = false;
  bool newTemp = false;
  float seaLevelPressure = 1013.25;
  float tempOffset;
  float pressOffset;
};
extern BaroData baro;
//-----------------------------------------
// Baro Reporting Variables
//-----------------------------------------
extern uint8_t baroTouchdown;
extern uint8_t touchdownTrigger;
extern float pressureAvg;
extern float pressureAvg5[5];
extern float pressureSum;
extern uint8_t pressurePosn;
//-----------------------------------------
// Magnetometer Variables
//-----------------------------------------
extern int16_t magTrigger;
extern uint8_t magCalibrateMode;
extern long magRoll;
extern int magOffVert;
extern int magPitch;
extern int magYaw;
//-----------------------------------------
// gyro & rotation variables
//-----------------------------------------
extern long gyroDeltaX;
extern long gyroDeltaY;
extern long gyroDeltaZ;
extern float yawY0;
extern float pitchX0;
extern int pitchX;
extern int yawY;
extern long rollZ;
const float MILLIONTH = 0.000001;
extern int offVert;
extern bool rotationFault;
//-----------------------------------------
// Rotation timing variables
//-----------------------------------------
extern unsigned long lastRotn; // time of the last call to update the rotation
extern unsigned long rotnRate; // 100 updates per second
//-----------------------------------------
// velocity calculation variables
//-----------------------------------------
extern float accelVel;
extern float accelAlt;
extern float maxVelocity;
extern float fusionVel;
extern float fusionAlt;
extern float thresholdVel;
extern float thresholdAlt;
extern uint32_t clearRailTime;
//-----------------------------------------
// beeper variables
//-----------------------------------------
extern uint8_t beepCounter;
extern uint8_t beepPosn;
extern uint32_t beepDelay;
extern uint8_t beepCode;
extern uint32_t beepLen;
extern uint32_t timeBeepStart;
extern uint32_t timeLastBeepEnd;
extern bool beepAlt;
extern bool beepVel;
extern bool beepCont;
extern bool beepAlarm;
extern bool livePyroAlarm;
const uint32_t SHORT_BEEP_DELAY = 100000UL;
const uint32_t LONG_BEEP_DELAY = 800000UL;
const uint32_t ALARM_BEEP_DELAY = 10000UL;
const uint32_t ALARM_BEEP_LEN = 10000UL;
const uint32_t MED_BEEP_LEN = 100000UL;
const uint32_t SHORT_BEEP_LEN = 10000UL;
//-----------------------------------------
// SD card writing variables
//-----------------------------------------
extern uint32_t writeStart;
extern uint32_t writeTime;
extern uint32_t maxWriteTime;
extern uint32_t writeThreshold;
extern uint32_t writeThreshCount;
extern bool syncApogee;
extern bool syncMains;
const byte DECIMAL_PLACES = 2;
const byte DECIMAL_BASE = 10;
// strPosn and dataString, the SD card line buffer, are declared in SD_Card.h
alignas(4) extern uint8_t maxAltDigits[6];
alignas(4) extern uint8_t maxVelDigits[4];
alignas(4) extern uint8_t voltageDigits[2];
extern uint8_t altDigits;
extern uint8_t velDigits;
extern uint8_t fileNum;
extern float voltage;
extern bool writeVolt;
extern unsigned long voltRate;
extern uint16_t voltReading;
extern unsigned long lastVolt;
extern bool reportMaxAlt; // true = report max altitude, false = report max velocity
extern uint8_t postFlightCode;
const char CSV_SEPARATOR = ',';
//-----------------------------------------
// GNSS Variables
//-----------------------------------------
struct EventData
{
  float latitude;
  float longitude;
  float alt = 0.0;
  int year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t centisecond;
};
struct GnssData
{
  float alt = 0.0;
  float maxAlt = 0.0;
  float baseAlt = 0.0;
  float vel = 0.0F;
  float latitude;
  float longitude;
  float avgAlt[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  float altBuff[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  uint32_t timeBuff[5] = {0UL, 0UL, 0UL, 0UL, 0UL};
  bool bufferFull = false;
  float altSum = 0.0;
  uint8_t bufferPosn = 0;
  uint8_t altPosn = 0;
  EventData liftoff;
  EventData apogee;
  EventData touchdown;
  uint8_t fix = 0;
  uint16_t fixCount = 0;
  unsigned long timeLastFix = 0UL;
  bool sdWrite = false;
  bool configDefaults = false;
  bool configFlight = false;
  bool configPwrSave = false;
  // GPS Global variables
  uint8_t serialPosn = 0;
  char serialBuffer[60] = "";
  bool msgRx = false;
};
extern GnssData gnss;
//-----------------------------------------
// EEPROM useful unions
//-----------------------------------------
union UlongUnion
{
  unsigned long val;
  byte bytes[4];
};
extern UlongUnion ulongUnion;
union IntUnion
{
  int val;
  byte bytes[2];
};
extern IntUnion intUnion;
union FloatUnion
{
  float val;
  byte bytes[4];
};
extern FloatUnion floatUnion;
//-----------------------------------------
// Active Stabilization
//-----------------------------------------
extern char userInput;
// servo trims in degrees, by servo: servoTrim[0] is servo 1
extern int8_t servoTrim[8];
const uint32_t CONTROL_INTERVAL = 50000UL;
extern uint32_t timeLastControl;
extern float kp;
extern float ki;
extern float kd;
//-----------------------------------------
// debug
//-----------------------------------------
extern boolean gpsEcho;
extern boolean radioDebug;
extern boolean txDataFile;
//-----------------------------------------
// loop debug and analysis variables
//-----------------------------------------
extern int cyclesBtwn;
extern uint32_t sampleTime;
extern uint32_t sampleStart;
extern uint32_t sampleTimeCheck;
extern uint32_t debugTimer;
//--------------------------------------------
// Radio event codes to eliminate magic numbers
//--------------------------------------------
enum FlightCode
{
  PREFLIGHT = 0,
  LIFTOFF = 1,
  BOOSTER_BURNOUT = 2,
  APOGEE = 3,
  FIRE_APOGEE = 4,
  SEPARATION = 5,
  FIRE_MAINS = 6,
  UNDER_CHUTE = 7,
  EJECT_BOOSTER = 8,
  FIRE_SUSTAINER = 9,
  SUSTAINER_IGNITION = 10,
  SUSTAINER_BURNOUT = 11,
  FIRE_AIRSTART1 = 12,
  AIRSTART1_IGNITION = 13,
  AIRSTART1_BURNOUT = 14,
  FIRE_AIRSTART2 = 15,
  AIRSTART2_IGNITION = 16,
  AIRSTART2_BURNOUT = 17,
  NO_FIRE_ROTN_LIMIT = 18,
  NO_FIRE_ALT_LIMIT = 19,
  NO_FIRE_ROTN_ALT_LIMIT = 20,
  BOOSTER_APOGEE = 21,
  FIRE_BOOSTER_APOGEE = 22,
  BOOSTER_SEPARATION = 23,
  FIRE_BOOSTER_MAINS = 24,
  BOOSTER_UNDER_CHUTE = 25,
  TIME_LIMIT = 26,
  TOUCHDOWN = 27,
  POWER_LOSS_RESTART = 28,
  BOOSTER_TOUCHDOWN = 29,
  BOOSTER_PREFLIGHT = 30,
  BOOSTER_TIME_LIMIT = 31,
  BOOSTER_POWER_LOSS = 32
};
//--------------------------------------------
// Radio operating modes (REG_OP_MODE)
//--------------------------------------------
enum RadioCode
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
extern uint8_t radioFunction;
//--------------------------------------------
// Function Pointers
//--------------------------------------------
/*!
 * @brief Initializes the configured accelerometer; returns false if it does not respond.
 * Set by setPolyMorphs() to the driver for the configured sensor type.
 */
extern bool (*beginAccel)();
/*!
 * @brief Initializes the configured gyroscope; returns false if it does not respond.
 * Set by setPolyMorphs() to the driver for the configured sensor type.
 */
extern bool (*beginGyro)();
/*!
 * @brief Initializes the configured magnetometer; returns false if it does not respond.
 * Set by setPolyMorphs() to the driver for the configured sensor type.
 */
extern bool (*beginMag)();
/*!
 * @brief Initializes the configured high-G accelerometer; returns false if it does not respond.
 * Set by setPolyMorphs() to the driver for the configured sensor type.
 */
extern bool (*beginHighG)();
/*!
 * @brief Initializes the configured barometer; returns false if it does not respond.
 * Set by setPolyMorphs() to the driver for the configured sensor type.
 */
extern bool (*beginBaro)();
extern bool (*beginRadio)();
/*!
 * @brief Reads raw data from the configured accelerometer.
 * Set by setPolyMorphs() to the driver for the configured sensor type.
 */
extern void (*getAccel)();
/*!
 * @brief Reads raw data from the configured gyroscope.
 * Set by setPolyMorphs() to the driver for the configured sensor type.
 */
extern void (*getGyro)();
/*!
 * @brief Reads raw data from the configured magnetometer.
 * Set by setPolyMorphs() to the driver for the configured sensor type.
 */
extern void (*getMag)();
/*!
 * @brief Reads raw data from the configured high-G accelerometer.
 * Set by setPolyMorphs() to the driver for the configured sensor type.
 */
extern void (*getHighG)();
/*!
 * @brief Reads raw data from the configured barometer.
 * Set by setPolyMorphs() to the driver for the configured sensor type.
 */
extern void (*getBaro)();
extern bool (*setRadioFreq)(float freq);
extern bool (*setRadioPwr)(int8_t pwr);
extern void (*radioSleep)();
extern bool (*radioSendPkt)(uint8_t *data, uint8_t len);
extern void (*gnssRestoreDefaults)();
extern void (*gnssConfig)();
extern void (*gnssPowerSave)();
