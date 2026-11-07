/**
 * @file flight_harness.h
 * @brief Runs the real flight step (FlightStep.h, FlightStep.cpp) on the host, one loop() pass at a time.
 *
 * @details
 * Include it from exactly one .cpp per test suite: it includes flightcomp_subset.h, which defines
 * the globals.
 *
 * A pass does what loop() does around the flight step, in the same order:
 * 1. the loop clock (fltTime.tmClock) is set and each sensor that is due is sampled, using
 *    loop()'s own scheduling arithmetic on the sensor's timeLastSamp and timeBtwnSamp;
 * 2. a new barometer sample goes through processBaroSamp();
 * 3. flightStep() runs and its actions are carried out through RecordingIo, which logs them;
 * 4. the parts of the rest of the in-flight pass that change what the next step sees: the SD
 *    write clears the new-sample flags, and touchdown or timeout closes the flight
 *    (events.liftoff = false).
 *
 * The sensor configuration is a board with an LSM6DSOX (16 g) accelerometer, an H3LIS331DL
 * (100 g) high-G accelerometer and an MS5611 barometer, with the gains, ranges and sample
 * intervals their drivers in Device_Drivers.cpp set, and the scaling setup() applies to them.
 * Sensor biases are zero, so the accelerometer reads 1 g at rest.
 */
#pragma once

#include <stdio.h>

#include <string>
#include <vector>

#include "flightcomp_subset.h"

#include "../src/FlightStep.cpp"

//-----------------------------------------
// Recorded I/O
//-----------------------------------------
inline std::vector<std::string> ioLog;

inline void logIo(const std::string &entry) { ioLog.push_back(entry); }

/** The I/O type performFlightActions() drives on the host: every call is logged. */
struct RecordingIo
{
  static void pinWrite(uint8_t pin, uint8_t level) { logIo("pin " + std::to_string(pin) + " " + std::to_string(level)); }
  static void eepromUpdate(int address, uint8_t value) { logIo("eeprom " + std::to_string(address) + " " + std::to_string(value)); }
  static void radioTimerEnd() { logIo("radioTimer.end"); }
  static void radioTimerBegin(uint32_t microseconds) { logIo("radioTimer.begin " + std::to_string(microseconds)); }
  static void syncSd() { logIo("syncSD"); }
  static void println(const char *text) { logIo(std::string("println ") + text); }
};

//-----------------------------------------
// Sensor configuration
//-----------------------------------------
// LSM6DSOX at 16 g (Device_Drivers.cpp beginLSM6DSOX)
constexpr float ACCEL_G_PER_COUNT = 0.000488F;
constexpr uint32_t ACCEL_SAMPLE_US = 602;
// H3LIS331DL at 100 g (beginH3LIS331DL)
constexpr float HIGHG_G_PER_COUNT = 0.049F;
constexpr uint32_t HIGHG_SAMPLE_US = 1000;
// MS5611 (beginMS56XX): one conversion per 9090 us, every 11th of them a temperature
constexpr uint32_t BARO_SAMPLE_US = 9090;
// Gyro rate on the LSM6DSOX path is read with the accelerometer
constexpr uint32_t GYRO_SAMPLE_US = 602;
constexpr float STANDARD_GRAVITY = 9.80665F;

/** Set the sensor scaling as the drivers and setup() do for this board. */
inline void configureSensors()
{
  accel.gainX = accel.gainY = accel.gainZ = ACCEL_G_PER_COUNT;
  accel.adcMax = (int16_t)(0.98 * 32768);
  accel.timeBtwnSamp = ACCEL_SAMPLE_US;
  gyro.gainX = gyro.gainY = gyro.gainZ = 0.07;
  gyro.adcMax = 32768;
  gyro.timeBtwnSamp = GYRO_SAMPLE_US;
  highG.gainX = highG.gainY = highG.gainZ = HIGHG_G_PER_COUNT;
  highG.timeBtwnSamp = HIGHG_SAMPLE_US;
  sensors.highG = 1;
  baro.timeBtwnSamp = BARO_SAMPLE_US;
  // setup(): set the accelerometer g level and convert the gains to m/s^2
  accel1G = (int16_t)(1.0 / accel.gainX);
  accel.gainX *= 9.80665;
  accel.gainY *= 9.80665;
  accel.gainZ *= 9.80665;
  gTrigger = 2.5 * accel1G;
  high1G = (int16_t)(1.0 / highG.gainX);
  highG.gainX *= 9.80665;
  highG.gainY *= 9.80665;
  highG.gainZ *= 9.80665;
}

/** Pyro outputs as setup() wires them: channel n fires on pin 20 + n. */
inline void configurePyros(char f1, char f2, char f3, char f4)
{
  settings.pyro1Func = pyros[0].func = f1;
  settings.pyro2Func = pyros[1].func = f2;
  settings.pyro3Func = pyros[2].func = f3;
  settings.pyro4Func = pyros[3].func = f4;
  pyros[0].firePin = 21;
  pyros[1].firePin = 22;
  pyros[2].firePin = 23;
  pyros[3].firePin = 24;
  pyros[0].contStatus = f1 != 'N';
  pyros[1].contStatus = f2 != 'N';
  pyros[2].contStatus = f3 != 'N';
  pyros[3].contStatus = f4 != 'N';
}

/** The testMode overrides setup() applies to the flight step's inputs. */
inline void applyTestModeOverrides()
{
  fltTime.detectLiftoffTime = 10000UL;
  settings.apogeeDelay = 1000000UL;
  settings.rcdTime = 15000000UL;
  if (settings.stableRotn || settings.stableVert)
  {
    settings.rcdTime = 30000000UL;
  }
  gTrigger = (int)(1.5 * accel1G);
  baro.maxAlt = 11101 / unitConvert;
  maxVelocity = 202 / unitConvert;
  pktInterval.postFlight = 1000000UL;
  thresholdVel = 15.5F;
  clearRailTime = 20000UL;
}

//-----------------------------------------
// What the sensors see
//-----------------------------------------
/** The physical inputs at one instant. */
struct Truth
{
  float specificForce; // axial specific force, m/s^2, up positive (+1 g at rest)
  float pressurePa;    // static pressure
  float gyroX;         // body rates, deg/s
  float gyroY;
  float gyroZ;
};

/** Turns the truth into sensor counts; the error terms let a test perturb the accelerometer. */
struct SensorModel
{
  float accelScale = 1.0F;  // multiplies the true specific force
  float accelBias = 0.0F;   // added to it, m/s^2
  float accelNoise = 0.0F;  // standard deviation of added white noise, m/s^2
  uint32_t noiseState = 12345U;

  // deterministic noise: a 32-bit LCG, summed for an approximately Gaussian value
  float gaussian()
  {
    float sum = 0.0F;
    for (int i = 0; i < 12; i++)
    {
      noiseState = noiseState * 1664525U + 1013904223U;
      sum += (float)(noiseState >> 8) / 16777216.0F;
    }
    return sum - 6.0F;
  }

  static int16_t clampCount(double counts)
  {
    long c = lround(counts);
    if (c > 32767)
    {
      c = 32767;
    }
    if (c < -32768)
    {
      c = -32768;
    }
    return (int16_t)c;
  }

  // measured specific force, m/s^2, sampled once per accelerometer sample
  float measuredForce(float force)
  {
    float f = force * accelScale + accelBias;
    if (accelNoise > 0.0F)
    {
      f += accelNoise * gaussian();
    }
    return f;
  }
  int16_t accelCounts(float measured) const { return clampCount(measured / (STANDARD_GRAVITY * ACCEL_G_PER_COUNT)); }
  int16_t highGCounts(float measured) const { return clampCount(measured / (STANDARD_GRAVITY * HIGHG_G_PER_COUNT)); }
  // gyro counts at 0.07 deg/s per count
  static int16_t gyroCounts(float degPerSec) { return clampCount(degPerSec / 0.07); }
};

/** Pressure to altitude, as the MS56XX driver computes baro.rawAlt (ConvertPressMS56XX). */
inline float baroAltitude(float pressurePa)
{
  float pressure = pressurePa * 0.01F;
  return 44330 * (1.0 - pow(pressure / (baro.seaLevelPressure), 0.1903));
}

//-----------------------------------------
// One loop() pass
//-----------------------------------------
/** Loop state the harness keeps between passes. */
struct LoopState
{
  uint8_t baroCounter = 0; // the MS56XX driver's pressure/temperature counter
  SensorModel model;
};

/**
 * @brief Steps 1 and 2 of a pass: sample what is due at time now, then process a new baro sample.
 */
inline void samplePass(LoopState &loopState, uint32_t now, const Truth &truth)
{
  fltTime.tmClock = sampleTimeCheck = now;
  if (sampleTimeCheck - accel.timeLastSamp > accel.timeBtwnSamp)
  {
    accel.z = loopState.model.accelCounts(loopState.model.measuredForce(truth.specificForce));
    accel.newSamp = true;
    accel.timeLastSamp += accel.timeBtwnSamp;
    while (sampleTimeCheck - accel.timeLastSamp > accel.timeBtwnSamp)
    {
      accel.timeLastSamp += accel.timeBtwnSamp;
    }
  }
  if (sampleTimeCheck - gyro.timeLastSamp > gyro.timeBtwnSamp)
  {
    gyro.x = SensorModel::gyroCounts(truth.gyroX);
    gyro.y = SensorModel::gyroCounts(truth.gyroY);
    gyro.z = SensorModel::gyroCounts(truth.gyroZ);
    gyro.newSamp = true;
    gyro.timeLastSamp += gyro.timeBtwnSamp;
    while (sampleTimeCheck - gyro.timeLastSamp > gyro.timeBtwnSamp)
    {
      gyro.timeLastSamp += gyro.timeBtwnSamp;
    }
  }
  if (sampleTimeCheck - highG.timeLastSamp > highG.timeBtwnSamp)
  {
    highG.z = loopState.model.highGCounts(truth.specificForce * loopState.model.accelScale + loopState.model.accelBias);
    highG.newSamp = true;
    highG.timeLastSamp += highG.timeBtwnSamp;
    while (sampleTimeCheck - highG.timeLastSamp > highG.timeBtwnSamp)
    {
      highG.timeLastSamp += highG.timeBtwnSamp;
    }
  }
  if (sampleTimeCheck - baro.timeLastSamp > baro.timeBtwnSamp)
  {
    if (loopState.baroCounter < 10)
    {
      baro.pressure = truth.pressurePa * 0.01F;
      baro.rawAlt = baroAltitude(truth.pressurePa);
      baro.newSamp = true;
      loopState.baroCounter++;
    }
    else
    {
      baro.newTemp = true;
      loopState.baroCounter = 0;
    }
    baro.timeLastSamp = sampleTimeCheck;
  }
  if (baro.newSamp)
  {
    processBaroSamp();
  }
}

/**
 * @brief Step 4 of a pass: what the rest of an in-flight pass changes for the next step.
 */
inline void finishPass(bool inFlight)
{
  if (!inFlight)
  {
    return;
  }
  // writeSDflightData() clears the new-sample flags
  accel.newSamp = gyro.newSamp = highG.newSamp = false;
  baro.newSamp = false;
  baro.newTemp = false;
  // touchdown or timeout closes the flight
  if (events.timeOut || events.touchdown)
  {
    fileClose = true;
    events.liftoff = false;
    livePyroAlarm = false;
  }
}

/** Run one whole pass with the flight step; returns its actions. */
inline FlightActions runPass(LoopState &loopState, uint32_t now, const Truth &truth)
{
  samplePass(loopState, now, truth);
  FlightActions actions = flightStep();
  performFlightActions<RecordingIo>(actions);
  finishPass(actions.inFlight);
  return actions;
}
