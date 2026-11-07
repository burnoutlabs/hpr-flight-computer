/**
 * @file startup_harness.h
 * @brief Runs the real setup() (Startup.cpp) on the host, with every board call logged.
 *
 * @details
 * Include it from exactly one .cpp per test suite: through flight_harness.h it defines the
 * globals, and it defines the stand-ins below.
 *
 * What setup() calls is replaced, in three groups:
 * - the Teensy core (pinMode, digitalWrite, digitalRead, analogRead, delay, Serial, EEPROM,
 *   the radio timer, the GNSS serial port, the servos): logged into ioLog (flight_harness.h) in
 *   call order, and EEPROM writes also land in the shim EEPROM. Inputs (the test button, the
 *   continuity pins, the battery, serial input) come from the scenario;
 * - the device drivers that setPolyMorphs() points at: logged by name; the get functions fill
 *   the raw sensor fields with deterministic values from a per-sensor counter;
 * - the SD card, calibration, orientation and telemetry routines (SD.cpp, Calibration.cpp,
 *   Telemetry.cpp, Bus_Mgmt.cpp): logged by name. readFlightSettingsSd() copies the scenario's
 *   settings into the fields the real parser sets.
 * What setup() itself decides, and the real setPolyMorphs(), getQuatRotn() and getDcm2dRotn(),
 * run as they are.
 *
 * Reading the clock advances it (shimMicrosPerCall), so setup()'s polling loops end.
 */
#pragma once

#include <stdio.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <Arduino.h>
#include <EEPROM.h>
#include <PWMServo.h>
#include <TinyGPS++.h>

// The I/O log: flight_harness.h defines it, and its RecordingIo writes the flight step's I/O to it
inline void logIo(const std::string &entry);

//-----------------------------------------
// Teensy core stand-ins
//-----------------------------------------
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// Teensyduino's min() and max(): the type is the common type of the arguments
template <class A, class B>
constexpr auto min(A a, B b) -> decltype(a < b ? a : b)
{
  return a < b ? a : b;
}
template <class A, class B>
constexpr auto max(A a, B b) -> decltype(a > b ? a : b)
{
  return a > b ? a : b;
}

/** Formats a print argument with its kind, so a change of type shows in the log. */
inline std::string fmtArg(const char *text) { return std::string("s:") + text; }
inline std::string fmtArg(char *text) { return std::string("s:") + text; }
inline std::string fmtArg(char c) { return std::string("c:") + c; }
template <class T>
std::string fmtArg(T value)
{
  if constexpr (std::is_floating_point_v<T>)
  {
    char text[64];
    snprintf(text, sizeof(text), "f%zu:%.9g", sizeof(T), (double)value);
    return text;
  }
  else
  {
    static_assert(std::is_integral_v<T>, "print argument");
    return std::string(std::is_signed_v<T> ? "i" : "u") + std::to_string(sizeof(T)) + ":" + std::to_string((long long)value);
  }
}

/** The scripted inputs of one run. */
struct BoardInputs
{
  // digitalRead() of the test button: runs of (count, level); the last level then holds
  std::vector<std::pair<int, uint8_t>> testButton;
  uint8_t testReadPin = 255;
  int testReads = 0;
  // continuity pins that read HIGH
  std::vector<uint8_t> highPins;
  int analogValue = 0;
  std::string serialInput;
  size_t serialPosn = 0;
  // getMag() calls before the magnet is sensed (magnetic switch)
  int magDetectAfter = 1000000;
};
inline BoardInputs boardInputs;

inline void pinMode(uint8_t pin, uint8_t mode) { logIo("pinMode " + std::to_string(pin) + " " + std::to_string(mode)); }
inline void digitalWrite(uint8_t pin, uint8_t level) { logIo("pin " + std::to_string(pin) + " " + std::to_string(level)); }
inline uint8_t digitalRead(uint8_t pin)
{
  if (pin == boardInputs.testReadPin)
  {
    int index = boardInputs.testReads++;
    for (const auto &run : boardInputs.testButton)
    {
      if (index < run.first)
      {
        return run.second;
      }
      index -= run.first;
    }
    return boardInputs.testButton.empty() ? HIGH : boardInputs.testButton.back().second;
  }
  return std::find(boardInputs.highPins.begin(), boardInputs.highPins.end(), pin) != boardInputs.highPins.end() ? HIGH : LOW;
}
inline int analogRead(uint8_t pin)
{
  logIo("analogRead " + std::to_string(pin));
  return boardInputs.analogValue;
}
inline void analogReadResolution(unsigned int bits) { logIo("analogReadResolution " + std::to_string(bits)); }
inline void delay(uint32_t ms)
{
  logIo("delay " + std::to_string(ms));
  shimAdvanceMicros(ms * 1000UL);
}

struct IntervalTimer
{
  bool begin(void (*)(), uint32_t microseconds)
  {
    logIo("radioTimer.begin " + std::to_string(microseconds));
    return true;
  }
  void end() { logIo("radioTimer.end"); }
};

struct HardwareSerial
{
  void begin(uint32_t baud) { logIo("HWSERIAL.begin " + std::to_string(baud)); }
  int available() { return 0; }
  int read() { return -1; }
};

/** Serial, logged with each argument's kind. */
struct LogSerial
{
  void begin(unsigned long baud) { logIo("Serial.begin " + std::to_string(baud)); }
  template <class T>
  size_t print(T value)
  {
    logIo("print " + fmtArg(value));
    return 0;
  }
  template <class T>
  size_t print(T value, int digits)
  {
    logIo("print " + fmtArg(value) + " /" + std::to_string(digits));
    return 0;
  }
  size_t println()
  {
    logIo("println");
    return 0;
  }
  template <class T>
  size_t println(T value)
  {
    logIo("println " + fmtArg(value));
    return 0;
  }
  template <class T>
  size_t println(T value, int digits)
  {
    logIo("println " + fmtArg(value) + " /" + std::to_string(digits));
    return 0;
  }
  int available() { return (int)(boardInputs.serialInput.size() - boardInputs.serialPosn); }
  int read() { return available() > 0 ? (uint8_t)boardInputs.serialInput[boardInputs.serialPosn++] : -1; }
};
inline LogSerial logSerial;

/** EEPROM: reads from the shim, writes logged and then made. */
inline ShimEEPROM &shimEeprom = EEPROM;
struct LogEeprom
{
  uint8_t read(int address) { return shimEeprom.read(address); }
  void update(int address, uint8_t value)
  {
    logIo("eeprom " + std::to_string(address) + " " + std::to_string(value));
    shimEeprom.update(address, value);
  }
  void write(int address, uint8_t value)
  {
    logIo("eeprom.write " + std::to_string(address) + " " + std::to_string(value));
    shimEeprom.write(address, value);
  }
};
inline LogEeprom logEeprom;

/** A servo that logs attach() and write(). */
struct LogServo
{
  int pin = -1;
  int lastWrite = -1;
  uint8_t attach(int servoPin)
  {
    pin = servoPin;
    logIo("servo.attach " + std::to_string(servoPin));
    return 1;
  }
  void write(int angle)
  {
    lastWrite = angle;
    logIo("servo.write " + std::to_string(pin) + " " + std::to_string(angle));
  }
};

#define Serial logSerial
#define EEPROM logEeprom
#define PWMServo LogServo

//-----------------------------------------
// The sources under test
//-----------------------------------------
#include "flight_harness.h"

#include "flight_state.h"

#include "../src/Device_Mgmt.cpp"

#include "../src/Startup.cpp"

// Objects that flightcomp.cpp and Bus_Mgmt.cpp define
IntervalTimer radioTimer;
IntervalTimer syncTimer;
inline HardwareSerial gnssPort;
HardwareSerial *HWSERIAL = nullptr;
void timerBuildPkt() { buildPkt = true; }

// parseBeep() is defined in flightcomp.cpp beside loop(), which the host does not build. Its
// copy at commit edc2ebf, which no change since has touched, stands in for it.
#include "legacy_setup/parse_beep.inc"

//-----------------------------------------
// Device driver stand-ins (the targets of setPolyMorphs())
//-----------------------------------------
/** Per-sensor read counters: the source of the deterministic sensor values. */
struct SensorFeed
{
  uint32_t accel = 0;
  uint32_t gyro = 0;
  uint32_t mag = 0;
  uint32_t highG = 0;
  uint32_t baro = 0;
};
inline SensorFeed sensorFeed;

// A test that models what the sensors see (the in-flight recovery test) sets these; each then
// replaces the deterministic values below for its sensor
inline void (*accelFeedHook)() = nullptr;
inline void (*baroFeedHook)() = nullptr;

inline void feedAccelAndGyro()
{
  if (accelFeedHook != nullptr)
  {
    accelFeedHook();
    return;
  }
  const int32_t n = (int32_t)sensorFeed.accel++;
  accel.rawX = (int16_t)(40 + n % 7);
  accel.rawY = (int16_t)(-25 + n % 5);
  accel.rawZ = (int16_t)(2040 + n % 11);
  gyro.rawX = (int16_t)(3 + n % 3);
  gyro.rawY = (int16_t)(-2 + n % 4);
  gyro.rawZ = (int16_t)(1 + n % 2);
}
inline void feedGyro()
{
  const int32_t n = (int32_t)sensorFeed.gyro++;
  gyro.rawX = (int16_t)(5 + n % 3);
  gyro.rawY = (int16_t)(-4 + n % 4);
  gyro.rawZ = (int16_t)(2 + n % 2);
}
inline void feedMag()
{
  const int32_t n = (int32_t)sensorFeed.mag++;
  const int16_t big = (n >= boardInputs.magDetectAfter) ? 4000 : 0;
  mag.rawX = (int16_t)(150 + n % 9 + big);
  mag.rawY = (int16_t)(-90 + n % 4);
  mag.rawZ = (int16_t)(300 + n % 6);
  // The magnetic switch wait in setup() tests mag.x, y and z after getMag(), but the drivers
  // fill only the raw fields and nothing in that wait calls orientMag(), so on a board the wait
  // never ends. Filling the oriented fields here lets the scenario run through that code.
  mag.x = mag.rawX;
  mag.y = mag.rawY;
  mag.z = mag.rawZ;
}
inline void feedHighG()
{
  const int32_t n = (int32_t)sensorFeed.highG++;
  highG.rawX = (int16_t)(1 + n % 3);
  highG.rawY = (int16_t)(-1 + n % 2);
  highG.rawZ = (int16_t)(21 + n % 2);
}
inline void feedBaro()
{
  if (baroFeedHook != nullptr)
  {
    baroFeedHook();
    return;
  }
  const int32_t n = (int32_t)sensorFeed.baro++;
  baro.rawAlt = 1400.0F + 0.01F * (float)(n % 13);
  baro.pressure = 856.0F + 0.001F * (float)(n % 7);
  baro.temperature = 21.5F;
}

#define LOGGED_BEGIN(name) \
  bool name()              \
  {                        \
    logIo(#name);          \
    return true;           \
  }
#define LOGGED_GET(name, feed) \
  void name()                  \
  {                            \
    logIo(#name);              \
    feed();                    \
  }
#define LOGGED_VOID(name) \
  void name() { logIo(#name); }

// accelerometers (the combined ones also fill the gyro, as their drivers do)
LOGGED_BEGIN(beginLSM303_A)
LOGGED_GET(getLSM303_A, feedAccelAndGyro)
LOGGED_BEGIN(beginLSM9DS1_AG)
LOGGED_GET(getLSM9DS1_AG, feedAccelAndGyro)
LOGGED_BEGIN(beginLSM6DS33)
LOGGED_GET(getLSM6DS33, feedAccelAndGyro)
LOGGED_BEGIN(beginMPU6050)
LOGGED_GET(getMPU6050, feedAccelAndGyro)
LOGGED_BEGIN(beginMPU9250_AG)
LOGGED_GET(getMPU9250_AG, feedAccelAndGyro)
LOGGED_BEGIN(beginLSM6DSOX)
LOGGED_GET(getLSM6DSOX, feedAccelAndGyro)
LOGGED_BEGIN(beginLSM6DS3TR)
LOGGED_GET(getLSM6DS3TR, feedAccelAndGyro)
LOGGED_BEGIN(beginExternalAccel)
LOGGED_GET(getExternalAccel, feedAccelAndGyro)
// gyroscopes
LOGGED_BEGIN(beginL3GD20H)
LOGGED_GET(getL3GD20H, feedGyro)
LOGGED_BEGIN(beginExternalGyro)
LOGGED_GET(getExternalGyro, feedGyro)
// high-G accelerometers
LOGGED_BEGIN(beginADS1115)
LOGGED_GET(getADS1115, feedHighG)
LOGGED_BEGIN(beginH3LIS331DL)
LOGGED_GET(getH3LIS331DL, feedHighG)
LOGGED_BEGIN(beginADXL377)
LOGGED_GET(getADXL377, feedHighG)
LOGGED_BEGIN(beginExternalHighG)
LOGGED_GET(getExternalHighG, feedHighG)
// magnetometers
LOGGED_BEGIN(beginLSM303_M)
LOGGED_GET(getLSM303_M, feedMag)
LOGGED_BEGIN(beginLSM9DS1_M)
LOGGED_GET(getLSM9DS1_M, feedMag)
LOGGED_BEGIN(beginLIS3MDL)
LOGGED_GET(getLIS3MDL, feedMag)
LOGGED_BEGIN(beginMPU9250_M)
LOGGED_GET(getMPU9250_M, feedMag)
LOGGED_BEGIN(beginExternalMag)
LOGGED_GET(getExternalMag, feedMag)
// barometers
boolean beginBMP180()
{
  logIo("beginBMP180");
  return true;
}
LOGGED_GET(getBMP180, feedBaro)
boolean beginMPL3115A2()
{
  logIo("beginMPL3115A2");
  return true;
}
LOGGED_GET(getMPL3115A2, feedBaro)
boolean beginBMP280()
{
  logIo("beginBMP280");
  return true;
}
LOGGED_GET(getBMP280, feedBaro)
boolean beginBMP388()
{
  logIo("beginBMP388");
  return true;
}
LOGGED_GET(getBMP388, feedBaro)
LOGGED_BEGIN(beginMS56XX)
LOGGED_GET(getMS56XX, feedBaro)
LOGGED_BEGIN(beginLPS25H)
LOGGED_GET(getLPS25H, feedBaro)
LOGGED_BEGIN(beginExternalBaro)
LOGGED_GET(getExternalBaro, feedBaro)
// radios
LOGGED_BEGIN(beginSX127X)
LOGGED_BEGIN(beginRFD900)
LOGGED_BEGIN(beginExternalRadio)
bool setFreqSX127X(float freq)
{
  logIo("setFreqSX127X " + fmtArg(freq));
  return true;
}
bool setPwrSX127X(int8_t pwr)
{
  logIo("setPwrSX127X " + fmtArg(pwr));
  return true;
}
bool setExternalRadioFreq(float freq)
{
  logIo("setExternalRadioFreq " + fmtArg(freq));
  return true;
}
bool setExternalRadioPwr(int8_t pwr)
{
  logIo("setExternalRadioPwr " + fmtArg(pwr));
  return true;
}
LOGGED_VOID(sleepSX127X)
LOGGED_VOID(externalRadioSleep)
bool sendPktSX127X(uint8_t *, uint8_t)
{
  logIo("sendPktSX127X");
  return true;
}
bool sendPktRFD900(uint8_t *, uint8_t)
{
  logIo("sendPktRFD900");
  return true;
}
bool sendExternalRadioDataPkt(uint8_t *, uint8_t)
{
  logIo("sendExternalRadioDataPkt");
  return true;
}
// GNSS receivers
LOGGED_VOID(UBLOXrestorDefaults)
LOGGED_VOID(UBLOXconfig)
LOGGED_VOID(UBLOXpowerSave)
LOGGED_VOID(ExternalGNSSconfig)
LOGGED_VOID(externalGNSS_PSM)

//-----------------------------------------
// SD card, calibration, orientation, telemetry and bus stand-ins
//-----------------------------------------
/** The settings Settings.txt holds, which readFlightSettingsSd() copies in. */
inline UserSettings settingsFile_;

LOGGED_VOID(beginSd)
LOGGED_VOID(restartSd)
LOGGED_VOID(parseEepromSettingsSd)
LOGGED_VOID(createNextFileSd)
LOGGED_VOID(reopenSd)
LOGGED_VOID(beginTelemetry)
LOGGED_VOID(accelCalibrate)
LOGGED_VOID(magCalibrate)
LOGGED_VOID(baroCalibrate)
LOGGED_VOID(setCanardTrim)

/** readFlightSettingsSd(): the fields the real parser (SD.cpp) sets, in its order. */
void readFlightSettingsSd()
{
  logIo("readFlightSettingsSd");
  const UserSettings &f = settingsFile_;
  memcpy(settings.rocketName, f.rocketName, sizeof(settings.rocketName));
  settings.fltProfile = f.fltProfile;
  settings.units = f.units;
  if (settings.units == 'M')
  {
    unitConvert = 1.0F;
  }
  settings.reportStyle = f.reportStyle;
  settings.setupTime = f.setupTime;
  settings.mainDeployAlt = f.mainDeployAlt;
  settings.apogeeDelay = f.apogeeDelay;
  settings.rcdTime = f.rcdTime;
  settings.silentMode = f.silentMode;
  settings.magSwitchEnable = f.magSwitchEnable;
  settings.inflightRecover = f.inflightRecover;
  settings.gpsLog = f.gpsLog;
  settings.fireTime = f.fireTime;
  settings.pyro4Func = f.pyro4Func;
  settings.pyro3Func = f.pyro3Func;
  settings.pyro2Func = f.pyro2Func;
  settings.pyro1Func = f.pyro1Func;
  settings.txEnable = f.txEnable;
  settings.txPwr = f.txPwr;
  settings.txFreq = f.txFreq;
  settings.fhss = f.fhss;
  settings.boosterSeparationDelay = f.boosterSeparationDelay;
  settings.sustainerFireDelay = f.sustainerFireDelay;
  settings.airStart1Event = f.airStart1Event;
  settings.airStart1Delay = f.airStart1Delay;
  settings.airStart2Event = f.airStart2Event;
  settings.airStart2Delay = f.airStart2Delay;
  settings.altThreshold = f.altThreshold;
  settings.maxAngle = f.maxAngle;
  settings.stableRotn = f.stableRotn;
  settings.stableVert = f.stableVert;
  settings.flyBack = f.flyBack;
  settings.serialDebug = f.serialDebug;
}

/** readOrientation(): logged; each sensor's axes map straight onto the board's. */
void readOrientation()
{
  logIo("readOrientation");
  SensorData *all[] = {&accel, &gyro, &mag, &highG};
  for (SensorData *s : all)
  {
    s->ptrX = &s->rawX;
    s->ptrY = &s->rawY;
    s->ptrZ = &s->rawZ;
    s->ptrXSign = &s->dirX;
    s->ptrYSign = &s->dirY;
    s->ptrZSign = &s->dirZ;
  }
}

void setHWSERIAL()
{
  logIo("setHWSERIAL " + std::to_string(sensors.gnssBusNum));
  HWSERIAL = &gnssPort;
}

//-----------------------------------------
// Whole state
//-----------------------------------------
/** What setup() can change beyond the flight globals: EEPROM, the ports, servos, clock, inputs. */
#define STARTUP_EXTRA_OBJECTS(X) \
  X(shimEeprom.bytes)            \
  X(HWSERIAL)                    \
  X(servos)                      \
      X(shimMicrosNow) X(sensorFeed) X(boardInputs.testReads) X(boardInputs.serialPosn)

#define STARTUP_STATE_OBJECTS(X) FLIGHT_STATE_OBJECTS(X) STARTUP_EXTRA_OBJECTS(X)

inline std::vector<uint8_t> saveStartupState()
{
  std::vector<uint8_t> bytes;
#define SAVE_OBJECT(object)                           \
  {                                                   \
    const uint8_t *p = (const uint8_t *)&(object);    \
    bytes.insert(bytes.end(), p, p + sizeof(object)); \
  }
  STARTUP_STATE_OBJECTS(SAVE_OBJECT)
#undef SAVE_OBJECT
  return bytes;
}

inline void restoreStartupState(const std::vector<uint8_t> &bytes)
{
  size_t at = 0;
#define RESTORE_OBJECT(object)                                    \
  {                                                               \
    memcpy((void *)&(object), bytes.data() + at, sizeof(object)); \
    at += sizeof(object);                                         \
  }
  STARTUP_STATE_OBJECTS(RESTORE_OBJECT)
#undef RESTORE_OBJECT
}

/** The name of the first object whose bytes differ, or "" if none. */
inline std::string firstStartupDifference(const std::vector<uint8_t> &a, const std::vector<uint8_t> &b)
{
  size_t at = 0;
  std::string name;
#define COMPARE_OBJECT(object)                                                   \
  if (name.empty() && memcmp(a.data() + at, b.data() + at, sizeof(object)) != 0) \
  {                                                                              \
    name = #object;                                                              \
  }                                                                              \
  at += sizeof(object);
  STARTUP_STATE_OBJECTS(COMPARE_OBJECT)
#undef COMPARE_OBJECT
  return name;
}
