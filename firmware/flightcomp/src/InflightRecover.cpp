/**
 * @file InflightRecover.cpp
 * @brief Resumes a flight after a power loss or reset in flight.
 *
 * @details
 * With in-flight recovery on (settings.inflightRecover, 1 without pyros, 2 with them), the flight
 * step records each flight event in EEPROM (eeprom.lastEvent), and the pad altitude at liftoff
 * (eeprom.baseAlt). On a restart, setup() reads the last event before anything overwrites it and,
 * if it is one rapidReset() can resume from, calls rapidReset() instead of the normal startup.
 *
 * rapidReset() brings the hardware up with the same steps setup() uses (Startup.cpp), reads the
 * flight settings from Settings.txt as setup() does, and restores the flight events up to the
 * recorded one. Before it resumes, it samples the barometer and accelerometer until the
 * barometric velocity window is full (31 barometer samples): if the rocket is standing still
 * (barometric velocity under 5 m/s and 1 g on the accelerometer, within 0.2 g), it is on the
 * ground and the normal startup runs instead. Otherwise it reopens the last flight file, restarts
 * the velocity and altitude estimates from the barometer, and loop() carries on.
 *
 * What a restart restores:
 * - the events up to and including the recorded one, so a charge whose firing was recorded is
 *   not fired again, and the ones after it fire when their conditions are met;
 * - the pad altitude, so the main deploys at its set altitude above the pad;
 * - the hardware, settings, calibration and orientation, as setup() sets them.
 * What it cannot restore:
 * - time: event times restart at zero. If apogee was the last event, the apogee charge fires the
 *   apogee delay after the restart; the main fires no sooner than 1 s after it; the record time
 *   runs from the restart;
 * - the velocity and altitude integrated before the reset: they restart from the barometer;
 * - two-stage and airstart flights: they resume as single stage, so no stage or airstart charge
 *   fires after a restart;
 * - an event whose charge fired but whose record did not reach EEPROM: that charge fires again;
 * - a restart under the main (after Fire_Mains or Under_Chute) or after the booster's main: the
 *   normal startup runs, as before.
 */

#include <Arduino.h>
#include <EEPROM.h>

#include "InflightRecover.h"
#include "declarations.h"
#include "board.h"
#include "flightcomp.h"
#include "FlightStep.h"
#include "Calibration.h"
#include "Device_Mgmt.h"
#include "SD_Card.h"
#include "Startup.h"

// Barometer samples taken before resuming: enough to refill processBaroSamp()'s 30-sample
// velocity window
constexpr uint8_t RECOVERY_BARO_SAMPLES = 31;
// Standing still: barometric velocity below this, and the accelerometer within this of 1 g
constexpr float RECOVERY_STILL_VEL_MPS = 5.0F;
constexpr float RECOVERY_STILL_G = 0.2F;

bool isRecoverableEvent(uint8_t lastEvent)
{
  return lastEvent >= Liftoff && lastEvent < Fire_Booster_Mains && lastEvent != Fire_Mains && lastEvent != Under_Chute;
}

/**
 * @brief The flight events of a flight whose last recorded event is lastEvent.
 */
static eventList eventsAfter(uint8_t lastEvent)
{
  eventList e = resetEvents;
  e.preLiftoff = false;
  e.inFlight = true;
  e.liftoff = true;
  e.falseLiftoffCheck = false;
  switch (lastEvent)
  {
  case Liftoff:
    break;
  case Separation:
  case Booster_Separation:
    e.apogeeSeparation = true;
    e.apogeeFire = true;
    e.apogee = true;
    e.boosterBurnout = true;
    break;
  case Fire_Apogee:
  case Fire_Booster_Apogee:
    e.apogeeFire = true;
    e.apogee = true;
    e.boosterBurnout = true;
    break;
  case Apogee:
  case Booster_Apogee:
    e.apogee = true;
    e.boosterBurnout = true;
    break;
  default: // burnout, and the stage and airstart events, which all come after it
    e.boosterBurnout = true;
    break;
  }
  return e;
}

bool rapidReset(uint8_t lastEvent)
{
  // bring the hardware up as setup() does
  readHardwareSettingsEeprom();
  setPinModes();
  beginGnssSerial();
  setAnalogResolution();
  restartSD();
  readFlightSettingsSD();
  limitSettings();
  beginSensors();
  scaleSensorGains();
  beginRadioLink();
  configurePyroChannels();
  configureRadio();
  if (settings.stableRotn || settings.stableVert)
  {
    attachServos();
  }
  readCalibrationEeprom();
  readOrientation();

  // resume complex flights as single stage, and fire nothing unless the user asked for pyros
  if (settings.fltProfile == '2' || settings.fltProfile == 'A')
  {
    settings.fltProfile = 'S';
  }
  if (settings.inflightRecover != 2)
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      pyros[i].func = 'N';
    }
  }
  checkPyroContinuity();

  // the pad altitude stored at liftoff
  for (byte i = 0; i < 4; i++)
  {
    floatUnion.Byte[i] = (byte)EEPROM.read(eeprom.baseAlt + i);
  }
  baro.baseAlt = floatUnion.val;

  // the events up to the recorded one; processBaroSamp() keeps the pad altitude from here on
  events = eventsAfter(lastEvent);

  // sample until the barometric velocity window is full
  uint8_t baroSamples = 0;
  while (baroSamples < RECOVERY_BARO_SAMPLES)
  {
    if (micros() - accel.timeLastSamp > accel.timeBtwnSamp)
    {
      getAccel();
      orientAccel();
      accel.timeLastSamp = micros();
    }
    if (micros() - baro.timeLastSamp > baro.timeBtwnSamp)
    {
      getBaro();
      baro.timeLastSamp = micros();
    }
    if (baro.newSamp)
    {
      processBaroSamp();
      baro.newSamp = false;
      baroSamples++;
    }
  }
  const float accelG = sqrtf((float)accel.x * accel.x + (float)accel.y * accel.y + (float)accel.z * accel.z) / (float)g;

  // standing still: on the ground, so start normally
  if (fabsf(baro.Vel) < RECOVERY_STILL_VEL_MPS && fabsf(accelG - 1.0F) < RECOVERY_STILL_G)
  {
    events = resetEvents;
    return false;
  }

  // restart the velocity and altitude estimates from the barometer
  fusionVel = accelVel = baro.Vel;
  fusionAlt = accelAlt = baro.smoothAlt;

  reOpenSD();
  radio.event = (settings.fltProfile == 'B') ? Booster_Power_Loss : Power_Loss_Restart;
  if (settings.txEnable)
  {
    radioTimer.begin(timerBuildPkt, pktInterval.inflight / pktInterval.samplesPerPkt);
  }
  fltTime.tmClock = fltTime.tmClockPrev = micros();
  return true;
} // end rapidReset
