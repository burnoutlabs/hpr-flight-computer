/**
 * @file globals.cpp
 * @brief Definitions of the flight computer's global variables.
 *
 * @details
 * declarations.h declares these extern for every source file; this is the one place they
 * are defined, with their initial values.
 */
#include <Arduino.h>

#include "declarations.h"
//-----------------------------------------
// Sensor device data
//-----------------------------------------
alignas(4) SensorConfig sensors;
//-----------------------------------------
// User Settings
//-----------------------------------------
UserSettings settings;
//-----------------------------------------
// Sensor variables
//-----------------------------------------
// rawData is defined in Bus_Mgmt.cpp
SensorData accel;
SensorData mag;
SensorData gyro;
SensorData highG;
//-----------------------------------------
// pyro variables
//-----------------------------------------
PyroMap pyros[4];
//-----------------------------------------
// flight events
//-----------------------------------------
alignas(4) struct EventList events;
alignas(4) struct EventList resetEvents;
//-----------------------------------------
// Master timing variables
//-----------------------------------------
struct TimerList fltTime;
struct TimerList resetFltTime;
//-----------------------------------------
// GPIO pin mapping
//-----------------------------------------
alignas(4) PinMap pins;
//-----------------------------------------
// radio variables
//-----------------------------------------
alignas(4) uint8_t dataPacket[256];
bool liftoffSync = false;
unsigned long TXstartTime;
bool implicitHdr = false;
bool syncFreq = false;
uint8_t pktPosn = 0;
uint8_t pktSize = 0;
uint16_t sampNum = 0;
uint8_t packetSamples = 4;
bool gpsTransmit = false;
bool TX = false;
bool sdRadioTx = false;
uint8_t radioMode;
volatile boolean clearTX = false;
volatile boolean buildPkt = false;
volatile boolean syncFlag = false;
bool sendPkt = false;
PacketIntervals pktInterval;
RadioTelemetry radio;
//-----------------------------------------
// continuity Booleans
//-----------------------------------------
alignas(4) ContinuityStatus cont;
//-----------------------------------------
// Non-Event Booleans
//-----------------------------------------
bool rotnOk = true;
bool altOk = false;
bool beep = false;
bool pyroFire = false;
bool fileClose = false;
unsigned long boosterBurpTime = 100000UL;
float unitConvert = 3.2808F;
//-----------------------------------------
// digital accelerometer variables
//-----------------------------------------
int accel1G = 1366;
float accelNow;
float maxG = 0.0;
int gTrigger = 3415; // 2.5G trigger
//-----------------------------------------
// High-G accelerometer variables
//-----------------------------------------
int high1G = 63;
bool filterFull = false;
long highGSum = 0L;
int highGFilter[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t sizeHighGFilter = 30;
uint8_t filterPosn = 0;
float highGSmooth;
float adcConvert = 0.000015259;
uint16_t adcMidValue = 32768;
alignas(4) CalUnion calUnion;
//-----------------------------------------
// Altitude & Baro Sensor variables
//-----------------------------------------
BaroData baro;
//-----------------------------------------
// Baro Reporting Variables
//-----------------------------------------
uint8_t baroTouchdown = 0;
uint8_t touchdownTrigger = 5;
float pressureAvg = 0;
float pressureAvg5[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float pressureSum = 0.0;
uint8_t pressurePosn = 0;
//-----------------------------------------
// Magnetometer Variables
//-----------------------------------------
int16_t magTrigger = 0;
uint8_t magCalibrateMode = 0;
long magRoll = 0UL;
int magOffVert = 0;
int magPitch = 0;
int magYaw = 0;
//-----------------------------------------
// gyro & rotation variables
//-----------------------------------------
long gyroDeltaX = 0L;
long gyroDeltaY = 0L;
long gyroDeltaZ = 0L;
float yawY0;
float pitchX0;
int pitchX;
int yawY;
long rollZ = 0;
int offVert = 0;
bool rotationFault = false;
//-----------------------------------------
// Rotation timing variables
//-----------------------------------------
unsigned long lastRotn = 0UL;     // time of the last call to update the rotation
unsigned long rotnRate = 10000UL; // 100 updates per second
//-----------------------------------------
// velocity calculation variables
//-----------------------------------------
float accelVel = 0.0F;
float accelAlt = 0.0F;
float maxVelocity = 0.0F;
float fusionVel = 0.0F;
float fusionAlt = 0.0F;
float thresholdVel = 35.0F;
float thresholdAlt = 61.0F;
uint32_t clearRailTime = 250000UL;
//-----------------------------------------
// beeper variables
//-----------------------------------------
uint8_t beepCounter = 0;
uint8_t beepPosn = 0;
uint32_t beepDelay = 100000UL;
uint8_t beepCode = 0;
uint32_t beepLen = 100000UL;
uint32_t timeBeepStart = 0UL;
uint32_t timeLastBeepEnd = 0UL;
bool beepAlt = false;
bool beepVel = false;
bool beepCont = true;
bool beepAlarm = false;
bool livePyroAlarm = false;
//-----------------------------------------
// SD card writing variables
//-----------------------------------------
uint32_t writeStart;
uint32_t writeTime;
uint32_t maxWriteTime;
uint32_t writeThreshold = 10000UL;
uint32_t writeThreshCount = 0UL;
bool syncApogee = false;
bool syncMains = false;
// strPosn and dataString are defined in SD.cpp
alignas(4) uint8_t maxAltDigits[6] = {0, 0, 0, 0, 0, 0};
alignas(4) uint8_t maxVelDigits[4] = {0, 0, 0, 0};
alignas(4) uint8_t voltageDigits[2] = {0, 0};
uint8_t altDigits = 6;
uint8_t velDigits = 4;
uint8_t fileNum = 1;
float voltage = 0.0F;
bool writeVolt = false;
unsigned long voltRate = 33333UL;
uint16_t voltReading;
unsigned long lastVolt = 0UL;
bool reportMaxAlt = true; // true = report max altitude, false = report max velocity
uint8_t postFlightCode = 0;
//-----------------------------------------
// GNSS Variables
//-----------------------------------------
GnssData gnss;
//-----------------------------------------
// EEPROM useful unions
//-----------------------------------------
UlongUnion ulongUnion;
IntUnion intUnion;
FloatUnion floatUnion;
//-----------------------------------------
// Active Stabilization
//-----------------------------------------
char userInput;
int8_t servoTrim[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint32_t timeLastControl = 0UL;
float kp = 0.0F;
float ki = 0.0F;
float kd = 0.0F;
//-----------------------------------------
// debug
//-----------------------------------------
boolean gpsEcho = false;
boolean radioDebug = false;
boolean txDataFile = true;
//-----------------------------------------
// loop debug and analysis variables
//-----------------------------------------
int cyclesBtwn = 0;
uint32_t sampleTime = 0UL;
uint32_t sampleStart = 0UL;
uint32_t sampleTimeCheck = 0UL;
uint32_t debugTimer = 0UL;
//--------------------------------------------
// Radio operating modes (RegOpMode)
//--------------------------------------------
uint8_t radioFnctn = SleepMode;
//--------------------------------------------
// Function Pointers
//--------------------------------------------
bool (*beginAccel)();
bool (*beginGyro)();
bool (*beginMag)();
bool (*beginHighG)();
bool (*beginBaro)();
bool (*beginRadio)();
void (*getAccel)();
void (*getGyro)();
void (*getMag)();
void (*getHighG)();
void (*getBaro)();
bool (*setRadioFreq)(float freq);
bool (*setRadioPWR)(int8_t pwr);
void (*radioSleep)();
bool (*radioSendPkt)(uint8_t *data, uint8_t len);
void (*GNSSrestorDefaults)();
void (*GNSSconfig)();
void (*GNSSpowerSave)();
