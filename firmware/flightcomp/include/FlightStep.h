/**
 * @file FlightStep.h
 * @brief The flight step: liftoff detection, velocity and altitude integration, and the event
 * logic, as one function that returns the I/O it wants done instead of doing it.
 *
 * @details
 * loop() calls flightStep() once per pass, after the sensors have been sampled and a new
 * barometer sample (if any) has gone through processBaroSamp(). flightStep() reads and updates
 * the flight globals in declarations.h, exactly as the same code did inside loop() and
 * checkEvents() before, but it does no I/O: every pin write, EEPROM write, radio timer change,
 * SD sync, GNSS time capture and debug print is appended to the FlightActions it returns, in
 * the order the old code did them. performFlightActions() then carries them out, in that order,
 * through an I/O type: the board's in flightcomp.cpp, a recording one in the host tests.
 *
 * Apogee is detected by either of two methods once booster burnout is confirmed and the
 * false-liftoff check has ended: barometric velocity (below 9000m above sea level, Mach-proofed
 * by the accelerometer velocity), or sensor fusion. Accelerometer-integrated velocity alone is
 * not used: replays of logged flights showed it firing before apogee with accelerometer errors of
 * 0.01 g or 1%, and up to 1.1 s early with a second flight accelerometer (#14).
 *
 * flightStep(), checkEvents() and their helpers run on every pass of the flight loop, so they
 * are defined here, always inline, for loop() to inline them.
 *
 * @remark Change log:
 * - 17 JUL 21: Initial breakout created
 */
#pragma once

#include <Arduino.h>

#include "declarations.h"
#include "flightcomp.h"
#include "Rotation.h"

//-----------------------------------------
// Named constants. Each keeps the literal (and so the type) the code used before it was named:
// an unsuffixed floating literal is double on Teensy 4.x and float on Teensy 3.x, which the
// build compiles with -fsingle-precision-constant.
//-----------------------------------------
// Sensor fusion velocity: the barometric velocity is blended in (1% per baro sample) only while
// it is below this speed, the vehicle is not accelerating, and it is below this altitude above
// the pad.
constexpr auto FUSION_BARO_VEL_MAX_MPS = 300;
constexpr auto FUSION_ACCEL_MAX_MPS2 = 0.2F;
constexpr auto FUSION_VEL_MAX_MPS = 300.0F;
constexpr auto FUSION_VEL_BARO_CEILING_M = 13000;
constexpr auto FUSION_VEL_KEEP = 0.99F;
constexpr auto FUSION_VEL_BARO_WEIGHT = 0.01F;
// Sensor fusion altitude: the barometric altitude is blended in (5% per baro sample) below this
// altitude above the pad.
constexpr auto FUSION_ALT_BARO_CEILING_M = 9000;
constexpr auto FUSION_ALT_KEEP = 0.95;
constexpr auto FUSION_ALT_BARO_WEIGHT = 0.05;
// Sustainer and airstart motor ignition: acceleration above this, in m/s^2
constexpr auto IGNITION_ACCEL_MPS2 = 10.0;
// A sustainer burnout counts only this long after its ignition
constexpr auto SUSTAINER_BURN_MIN_US = 100000UL;
// Apogee: barometric velocity below this (m/s), while the accelerometer velocity is below the
// Mach lock speed and the barometric altitude above sea level (not above the pad) is below the
// ceiling
constexpr auto BARO_APOGEE_VEL_MPS = -10;
constexpr auto BARO_APOGEE_MACH_LOCK_MPS = 70;
constexpr auto BARO_APOGEE_CEILING_M = 9000;
// Sync the SD card this long after the apogee charge fires, and after the main charge fires
constexpr auto SYNC_AFTER_APOGEE_US = 3000000UL;
constexpr auto SYNC_AFTER_MAIN_US = 3000000UL;
// Separation after apogee: accelerometer above this many g, within this long of the apogee charge
constexpr auto APOGEE_SEPARATION_G = 4;
constexpr auto APOGEE_SEPARATION_WINDOW_US = 2000000UL;
// The main charge fires no sooner than this after the apogee charge
constexpr auto MAIN_AFTER_APOGEE_MIN_US = 1000000UL;
// Main chute opening: acceleration above this (m/s^2), between these times after the main charge
constexpr auto CHUTE_CHECK_START_US = 50000UL;
constexpr auto CHUTE_CHECK_END_US = 3000000UL;
constexpr auto CHUTE_OPEN_ACCEL_MPS2 = 50.0;
// Touchdown: barometric altitude above the pad below this (m)
constexpr auto TOUCHDOWN_MAX_ALT_M = 46;

// Process a new barometer sample (FlightStep.cpp)
void processBaroSamp();

//-----------------------------------------
// Actions
//-----------------------------------------
/** I/O the flight step asks for. performFlightActions() says what each one does. */
enum class FlightAction : uint8_t
{
  PIN_HIGH,                // set pin arg high
  PIN_LOW,                 // set pin arg low
  EEPROM_LAST_EVENT,       // write arg (a FlightCode value) to eeprom.lastEvent
  EEPROM_BASE_ALT,         // write the 4 bytes of floatUnion (baro.baseAlt) to eeprom.baseAlt
  RADIO_INFLIGHT,          // restart the radio timer at the inflight sample rate, build a packet now
  RADIO_POSTFLIGHT,        // start the radio timer at the postflight packet rate
  RADIO_PRELIFTOFF,        // start the radio timer at the preflight packet rate
  SYNC_SD,                 // sync the SD card
  GNSS_LIFTOFF_TIME,       // copy the GNSS time into gnss.liftoff
  GNSS_TOUCHDOWN_TIME,     // copy the GNSS time into gnss.touchdown
  PRINT_SIMULATED_LIFTOFF, // test mode: report the liftoff
  PRINT_LIVE_PYRO_ALARM,   // report that the live-pyro alarm is on
  PRINT_FALSE_TRIGGER,     // test mode: report a false-liftoff reset
};

// The number of pushFlightAction() calls a flight step can reach: 6 in flightStep(), 44 in
// checkEvents(), 4 in each of the 8 uses of firePyros() and 8 in pulsePyro(). None is in a loop,
// so no step can push more. pushFlightAction() still stops at the limit rather than overrun.
constexpr uint8_t FLIGHT_ACTIONS_MAX = 90;

/** The I/O one flight step asks for, in the order it is to be done. */
struct FlightActions
{
  // events.liftoff was set when the in-flight part of the step ran, so the rest of the in-flight
  // pass (control, battery, SD write, file close) runs too. A false-liftoff reset clears
  // events.liftoff during the step; the pass still finishes as one in flight, as it always has.
  bool inFlight;
  uint8_t count;
  FlightAction kind[FLIGHT_ACTIONS_MAX];
  uint8_t arg[FLIGHT_ACTIONS_MAX];
};

void pushFlightAction(FlightActions &actions, FlightAction kind, uint8_t arg = 0);

/**
 * @brief Fire every pyro channel whose function is event: mark it firing and ask for its pin to
 * go high.
 */
void firePyros(char event, FlightActions &actions);

/**
 * @brief Pseudo-PWM for the pyro outputs: ask for each firing channel's pin to toggle.
 */
void pulsePyro(FlightActions &actions);

/**
 * @brief Check and manage rocket events during the flight.
 *
 * This function manages the rocket's key events, including timeouts, false liftoff checks,
 * booster burnout, stage separations, and parachute deployments. It also handles touch down
 * detection and pyrotechnic control. The I/O it needs is appended to actions.
 */
inline __attribute__((always_inline)) void checkEvents(FlightActions &actions)
{

  // Check for timeout
  if (!events.timeOut && !pyroFire && fltTime.timeCurrent > settings.rcdTime)
  {
    events.timeOut = true;
    events.postFlight = true;
    events.inFlight = false;
    radio.event = TIME_LIMIT;
    if (settings.fltProfile == 'B')
    {
      radio.event = BOOSTER_TIME_LIMIT;
    }
    if (settings.inflightRecover != 0)
    {
      pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
    }
    pushFlightAction(actions, FlightAction::RADIO_POSTFLIGHT);
    fltTime.touchdown = fltTime.timeCurrent;
    pushFlightAction(actions, FlightAction::GNSS_TOUCHDOWN_TIME);
  }

  // Check false trigger until:
  // the flight time has passed the minimum check-time
  // or the integrated speed is greater than the liftoff detection threshold
  // or the integrated altitude is greater than the liftoff detection threshold
  if (events.falseLiftoffCheck)
  {
    if (fltTime.timeCurrent > fltTime.detectLiftoffTime || accelVel > thresholdVel || accelAlt > thresholdAlt)
    {
      events.falseLiftoffCheck = false;
    }
    // if the acceleration drops below the trigger threshold
    // and the flight time is greater than 0.25 seconds but less than 0.5 seconds
    // and the integrated velocity inidicates the rocket will not go 100 feet
    // then reset flight variables and resume launch detect
    //--------------Developer's Note--------------------------------------------------------------------------
    // 100 feet is the minimum altitude at which the flight computer will attempt to deploy recovery devices
    // this increases the chance of a successful deployment if the motor CATOs after a short boost
    // 0.25 seconds is the typical time to depart the rail so that transient negative accelerations cannot reset the system
    if (accel.z < gTrigger && accelVel < thresholdVel && fltTime.timeCurrent > clearRailTime)
    {
      if (settings.testMode)
      {
        pushFlightAction(actions, FlightAction::PRINT_FALSE_TRIGGER);
      }
      // reset the key triggers
      events = resetEvents;
      fltTime.timeCurrent = 0UL;
      radio.fltTime = 0;
      radio.event = PREFLIGHT;
      if (settings.fltProfile == 'B')
      {
        radio.event = BOOSTER_PREFLIGHT;
      }
      pktPosn = 0;
      radio.packetnum = 0;
      sampNum = 0;
      pushFlightAction(actions, FlightAction::RADIO_PRELIFTOFF);
      // reset the high-g filter
      filterPosn = 0;
      highGSum = 0;
      for (byte i = 0; i < (sizeof(highGFilter) / sizeof(highGFilter[0])); i++)
      {
        highGFilter[i] = 0;
      }
      filterFull = false;
      // reset the acceleration based altitude and velocity
      accelVel = 0;
      accelAlt = 0;
      // sync the SD card
      pushFlightAction(actions, FlightAction::SYNC_SD);
    }
  } // end falseLiftoffCheck

  // check for booster burnout: if the z acceleration is negative
  if (!events.boosterBurnout && events.liftoff && accel.z <= 0)
  {
    events.boosterBurnout = true;
    radio.event = BOOSTER_BURNOUT;
    if (settings.inflightRecover != 0 && !settings.testMode)
    {
      pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
    }
    events.boosterBurnoutCheck = true;
    fltTime.boosterBurnout = fltTime.timeCurrent;
  }

  // check for booster motor burp for 0.1 second after burnout is detected
  if (events.boosterBurnoutCheck)
  {
    if (fltTime.timeCurrent - fltTime.boosterBurnout > boosterBurpTime)
    {
      events.boosterBurnoutCheck = false;
    }
    else if (events.boosterBurnout && !settings.testMode && accel.z > 0)
    {
      events.boosterBurnout = false;
      events.boosterBurnoutCheck = false;
      radio.event = LIFTOFF;
    }
  }

  // 2-Stage Flight Profile
  if (settings.fltProfile == '2')
  {

    // Fire separation charge if burnout is detected and time is past the separation delay
    if (!events.boosterSeparation && events.liftoff && events.boosterBurnout && !events.falseLiftoffCheck && fltTime.timeCurrent - fltTime.boosterBurnout > settings.boosterSeparationDelay)
    {
      events.boosterSeparation = true;
      fltTime.boosterSeparation = fltTime.timeCurrent;
      firePyros('B', actions);
      radio.event = EJECT_BOOSTER;
      if (settings.inflightRecover != 0 && !settings.testMode)
      {
        pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
      }
    }

    // Fire second stage
    if (!events.sustainerFireCheck && (!events.apogee || settings.testMode) && events.liftoff && events.boosterBurnout && events.boosterSeparation && !pyroFire && fltTime.timeCurrent - fltTime.boosterSeparation > settings.sustainerFireDelay)
    {
      events.sustainerFireCheck = true;
      postFlightCode = 1;
      fltTime.sustainerFireCheck = fltTime.timeCurrent;
      // Check for staging inhibit and fire if OK
      if (altOk && rotnOk)
      {
        events.sustainerFire = true;
        firePyros('I', actions);
        radio.event = FIRE_SUSTAINER;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
      else if (!rotnOk && !altOk)
      {
        postFlightCode = 4;
        radio.event = NO_FIRE_ROTN_ALT_LIMIT;
      }
      else if (!rotnOk)
      {
        postFlightCode = 3;
        radio.event = NO_FIRE_ROTN_LIMIT;
      }
      else if (!altOk)
      {
        postFlightCode = 2;
        radio.event = NO_FIRE_ALT_LIMIT;
      }
    }

    // Check for sustainer ignition
    if (!events.apogee && !events.sustainerIgnition && events.sustainerFire && accelNow > IGNITION_ACCEL_MPS2)
    {
      radio.event = SUSTAINER_IGNITION;
      if (settings.inflightRecover != 0 && !settings.testMode)
      {
        pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
      }
      events.sustainerIgnition = true;
      fltTime.sustainerIgnition = fltTime.timeCurrent;
    }

    // Check for sustainer burnout
    if (!events.apogee && !events.sustainerBurnout && events.sustainerIgnition && accelNow < 0.0 && fltTime.timeCurrent - fltTime.sustainerIgnition > SUSTAINER_BURN_MIN_US)
    {
      radio.event = SUSTAINER_BURNOUT;
      if (settings.inflightRecover != 0 && !settings.testMode)
      {
        pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
      }
      events.sustainerBurnout = true;
      fltTime.sustainerBurnout = fltTime.timeCurrent;
    }

  } // end 2-stage profile

  // Airstart Flight Profile
  if (settings.fltProfile == 'A')
  {

    // AirStart motor 1 if event 1 is main booster ignition
    if (settings.airStart1Event == 'I' && !events.airStart1Check && !events.falseLiftoffCheck && fltTime.timeCurrent > settings.airStart1Delay)
    {
      events.airStart1Check = true;
      fltTime.airStart1Check = fltTime.timeCurrent;
      postFlightCode = 1;
      // Check for inflight ignition inhibit and fire if OK
      if (altOk && rotnOk)
      {
        events.airStart1Fire = true;
        fltTime.airStart1Fire = fltTime.timeCurrent;
        firePyros('1', actions);
        radio.event = FIRE_AIRSTART1;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
      else if (!rotnOk && !altOk)
      {
        postFlightCode = 4;
        radio.event = NO_FIRE_ROTN_ALT_LIMIT;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
      else if (!rotnOk)
      {
        postFlightCode = NO_FIRE_ROTN_LIMIT;
        radio.event = 18;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
      else if (!altOk)
      {
        postFlightCode = 2;
        radio.event = NO_FIRE_ALT_LIMIT;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
    }

    // AirStart motor 1 if event 1 is main booster burnout
    if (settings.airStart1Event == 'B' && events.boosterBurnout && !events.airStart1Check && fltTime.timeCurrent > (fltTime.boosterBurnout + settings.airStart1Delay))
    {
      events.airStart1Check = true;
      fltTime.airStart1Check = fltTime.timeCurrent;
      postFlightCode = 1;
      // Check for inflight ignition inhibit and fire if OK
      if (altOk && rotnOk)
      {
        events.airStart1Fire = true;
        fltTime.airStart1Fire = fltTime.timeCurrent;
        firePyros('1', actions);
        radio.event = FIRE_AIRSTART1;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
      else if (!rotnOk && !altOk)
      {
        postFlightCode = 4;
        radio.event = NO_FIRE_ROTN_ALT_LIMIT;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
      else if (!rotnOk)
      {
        postFlightCode = 3;
        radio.event = NO_FIRE_ROTN_LIMIT;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
      else if (!altOk)
      {
        postFlightCode = 2;
        radio.event = NO_FIRE_ALT_LIMIT;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
    }

    // Look for AirStart 1 Ignition
    if (events.airStart1Fire && !events.apogee && !events.airStart1Ignition && accelNow > IGNITION_ACCEL_MPS2)
    {
      events.airStart1Ignition = true;
      fltTime.airStart1Ignition = fltTime.timeCurrent;
      radio.event = AIRSTART1_IGNITION;
      if (settings.inflightRecover != 0 && !settings.testMode)
      {
        pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
      }
    }

    // Look for AirStart 1 Burnout with a check for a motor burp
    if (!events.airStart1Burnout && events.airStart1Ignition)
    {
      if (!events.airStart1BurnoutCheck && !events.apogee && !events.airStart1Burnout && accelNow < 0.0)
      {
        events.airStart1BurnoutCheck = true;
        fltTime.airStart1Burnout = fltTime.timeCurrent;
      }
      if (events.airStart1BurnoutCheck && fltTime.timeCurrent < (fltTime.airStart1Burnout + boosterBurpTime) && accel.z > 0)
      {
        events.airStart1BurnoutCheck = false;
      }
      if (events.airStart1BurnoutCheck && fltTime.timeCurrent > (fltTime.airStart1Burnout + boosterBurpTime))
      {
        events.airStart1BurnoutCheck = false;
        events.airStart1Burnout = true;
        radio.event = AIRSTART1_BURNOUT;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
    }

    // AirStart motor 2 if event 2 is airstart1 motor ignition
    if (settings.airStart2Event == 'I' && !events.apogee && !events.airStart2Check && events.airStart1Ignition && fltTime.timeCurrent > fltTime.airStart1Ignition + settings.airStart2Delay)
    {
      events.airStart2Check = true;
      fltTime.airStart2Check = fltTime.timeCurrent;
      postFlightCode = 1;
      // Check for inflight ignition inhibit and fire if OK
      if (altOk && rotnOk)
      {
        events.airStart2Fire = true;
        fltTime.airStart2Fire = fltTime.timeCurrent;
        firePyros('2', actions);
        radio.event = FIRE_AIRSTART2;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
      else if (!rotnOk && !altOk)
      {
        postFlightCode = 4;
        radio.event = NO_FIRE_ROTN_ALT_LIMIT;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
      else if (!rotnOk)
      {
        postFlightCode = 3;
        radio.event = NO_FIRE_ROTN_LIMIT;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
      else if (!altOk)
      {
        postFlightCode = 2;
        radio.event = NO_FIRE_ALT_LIMIT;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
    }

    // Airstart motor 2 if event 2 is airstart1 motor burnout
    if (settings.airStart2Event == 'B' && !events.apogee && !events.airStart2Check && events.airStart1Burnout && fltTime.timeCurrent > (fltTime.airStart1Burnout + settings.airStart2Delay))
    {
      events.airStart2Check = true;
      fltTime.airStart2Check = fltTime.timeCurrent;
      postFlightCode = 1;
      // Check for inflight ignition inhibit and fire if OK
      if (altOk && rotnOk)
      {
        events.airStart2Fire = true;
        fltTime.airStart2Fire = fltTime.timeCurrent;
        firePyros('2', actions);
        radio.event = FIRE_AIRSTART2;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
      else if (!rotnOk && !altOk)
      {
        postFlightCode = 4;
        radio.event = NO_FIRE_ROTN_ALT_LIMIT;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
      else if (!rotnOk)
      {
        postFlightCode = 3;
        radio.event = NO_FIRE_ROTN_LIMIT;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
      else if (!altOk)
      {
        postFlightCode = 2;
        radio.event = NO_FIRE_ALT_LIMIT;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
    }

    // Look for AirStart 2 Ignition
    if (events.airStart2Fire && !events.apogee && !events.airStart2Ignition && accelNow > IGNITION_ACCEL_MPS2)
    {
      events.airStart2Ignition = true;
      fltTime.airStart2Ignition = fltTime.timeCurrent;
      radio.event = AIRSTART2_IGNITION;
      if (settings.inflightRecover != 0 && !settings.testMode)
      {
        pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
      }
    }

    // Look for AirStart 2 Burnout
    if (!events.airStart2Burnout && events.airStart2Ignition)
    {
      if (!events.airStart2BurnoutCheck && !events.apogee && !events.airStart2Burnout && accelNow < 0.0)
      {
        events.airStart2BurnoutCheck = true;
        fltTime.airStart2Burnout = fltTime.timeCurrent;
      }
      if (events.airStart2BurnoutCheck && fltTime.timeCurrent < (fltTime.airStart2Burnout + boosterBurpTime) && accel.z > 0)
      {
        events.airStart2BurnoutCheck = false;
      }
      if (events.airStart2BurnoutCheck && fltTime.timeCurrent > (fltTime.airStart2Burnout + boosterBurpTime))
      {
        events.airStart2BurnoutCheck = false;
        events.airStart2Burnout = true;
        radio.event = AIRSTART2_BURNOUT;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
        }
      }
    }
  } // End Airstart Flight Mode

  // Barometric based apogee detection: rocket must be below 9000m and barometric velocity < 0 and motor burnout is detected and accelometer velocity < 70 (needed for Mach proofing)
  bool baroApogee = (baro.vel < BARO_APOGEE_VEL_MPS && accelVel < BARO_APOGEE_MACH_LOCK_MPS && (baro.alt + baro.baseAlt) < BARO_APOGEE_CEILING_M) ? true : false;
  // Sensor fusion based apogee detection
  bool fusionApogee = (fusionVel < 0) ? true : false;
  // Check for apogee event
  if (!events.apogee && !pyroFire && events.boosterBurnout && !events.boosterBurnoutCheck && !events.falseLiftoffCheck && (baroApogee || fusionApogee))
  {
    events.apogee = true;
    fltTime.apogee = fltTime.timeCurrent;
    radio.event = APOGEE;
    if (settings.fltProfile == 'B')
    {
      radio.event = BOOSTER_APOGEE;
    }
    if (settings.inflightRecover != 0 && !settings.testMode)
    {
      pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
    }
  }

  // Fire apgogee charge if the current time > apogeeTime + apogeeDelay
  if (!events.apogeeFire && events.apogee && fltTime.timeCurrent - fltTime.apogee >= settings.apogeeDelay)
  {
    events.apogeeFire = true;
    fltTime.apogeeFire = fltTime.timeCurrent;
    firePyros('A', actions);
    radio.event = FIRE_APOGEE;
    if (settings.fltProfile == 'B')
    {
      radio.event = FIRE_BOOSTER_APOGEE;
    }
    if (settings.inflightRecover != 0 && !settings.testMode)
    {
      pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
    }
  }

  // Write the data to the card 3s after apogeeFire in case of crash or powerloss
  if (events.apogeeFire && !syncApogee && !settings.testMode && fltTime.timeCurrent - fltTime.apogeeFire >= SYNC_AFTER_APOGEE_US)
  {
    pushFlightAction(actions, FlightAction::SYNC_SD);
    syncApogee = true;
  }

  // Detect separation after apogee
  if (events.apogeeFire && !events.mainDeploy && accel.z > APOGEE_SEPARATION_G * accel1G && fltTime.timeCurrent - fltTime.apogeeFire <= APOGEE_SEPARATION_WINDOW_US)
  {
    events.apogeeSeparation = true;
    fltTime.apogeeSeparation = fltTime.timeCurrent;
    radio.event = SEPARATION;
    if (settings.fltProfile == 'B')
    {
      radio.event = BOOSTER_SEPARATION;
    }
    if (settings.inflightRecover != 0 && !settings.testMode)
    {
      pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
    }
  }

  // Fire main chute charge if the baro altitude is lower than the threshold and at least 1s has passed since apogee
  if (!events.mainDeploy && events.apogeeFire && baro.alt < settings.mainDeployAlt && fltTime.timeCurrent - fltTime.apogeeFire >= MAIN_AFTER_APOGEE_MIN_US)
  {
    events.mainDeploy = true;
    fltTime.mainDeploy = fltTime.timeCurrent;
    firePyros('M', actions);
    radio.event = FIRE_MAINS;
    if (settings.fltProfile == 'B')
    {
      radio.event = FIRE_BOOSTER_MAINS;
    }
    if (settings.inflightRecover != 0 && !settings.testMode)
    {
      pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
    }
  }

  // Detect deployment of the mains
  if (events.mainDeploy && fltTime.timeCurrent - fltTime.mainDeploy > CHUTE_CHECK_START_US && fltTime.timeCurrent - fltTime.mainDeploy < CHUTE_CHECK_END_US && accelNow > CHUTE_OPEN_ACCEL_MPS2)
  {
    radio.event = UNDER_CHUTE;
    if (settings.fltProfile == 'B')
    {
      radio.event = BOOSTER_UNDER_CHUTE;
    }
    if (settings.inflightRecover != 0 && !settings.testMode)
    {
      pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
    }
  }

  // Write the data to the card 3s after mainDeploy in case of crash or powerloss
  if (events.mainDeploy && !syncMains && !settings.testMode && fltTime.timeCurrent - fltTime.mainDeploy >= SYNC_AFTER_MAIN_US)
  {
    pushFlightAction(actions, FlightAction::SYNC_SD);
    syncMains = true;
  }

  // Turn off the pyros after the allotted time
  if (pyroFire)
  {
    // Check if pseudo-PWM is required
    if (sensors.pyroPwm)
    {
      pulsePyro(actions);
    }
    // Turn off the pyros
    bool anyFiring = false;
    for (uint8_t i = 0; i < 4; i++)
    {
      if (pyros[i].fireStatus && fltTime.timeCurrent - pyros[i].fireStart > settings.fireTime)
      {
        pushFlightAction(actions, FlightAction::PIN_LOW, pyros[i].firePin);
        pyros[i].fireStatus = false;
      }
      anyFiring = anyFiring || pyros[i].fireStatus;
    }
    if (!anyFiring)
    {
      pyroFire = false;
    }
  }

  // Check for touchdown
  if (!events.touchdown && events.mainDeploy && !pyroFire && !settings.testMode && baroTouchdown > touchdownTrigger && baro.alt < TOUCHDOWN_MAX_ALT_M)
  {
    events.touchdown = true;
    events.inFlight = false;
    events.postFlight = true;
    fltTime.touchdown = fltTime.timeCurrent;
    radio.event = TOUCHDOWN;
    if (settings.fltProfile == 'B')
    {
      radio.event = BOOSTER_TOUCHDOWN;
    }
    if (settings.inflightRecover != 0 && !settings.testMode)
    {
      pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
    }
    pushFlightAction(actions, FlightAction::GNSS_TOUCHDOWN_TIME);
  }

} // end check events

/**
 * @brief One pass of the flight logic: liftoff detection, then, in flight, the high-G filter,
 * velocity and altitude integration, the rotation update and the event logic.
 *
 * @return The I/O to do, in order, with performFlightActions().
 */
inline __attribute__((always_inline)) FlightActions flightStep()
{
  FlightActions actions;
  actions.inFlight = false;
  actions.count = 0;

  // detect liftoff
  if (!events.liftoff && accel.z > gTrigger && !events.touchdown && !events.timeOut)
  {
    if (settings.testMode)
    {
      pushFlightAction(actions, FlightAction::PRINT_SIMULATED_LIFTOFF);
    }
    // set liftoff clock times
    fltTime.padTime = fltTime.tmClockPrev = fltTime.tmClock;
    fltTime.liftoff = fltTime.tmClock;
    // update events
    events.preLiftoff = false;
    events.liftoff = true;
    events.inFlight = true;
    // set radio variables
    radio.event = LIFTOFF;
    radio.alt = 0;
    pushFlightAction(actions, FlightAction::RADIO_INFLIGHT);
    // set GNSS liftoff values
    pushFlightAction(actions, FlightAction::GNSS_LIFTOFF_TIME);
    gnss.maxAlt = 0.0F;
    // if live pyros are detected, turn on the alarm to alert the user that the system is active and may fire pyros
    if (pyros[0].contStatus || pyros[1].contStatus || pyros[2].contStatus || pyros[3].contStatus || settings.testMode)
    {
      livePyroAlarm = true;
      pushFlightAction(actions, FlightAction::PRINT_LIVE_PYRO_ALARM);
    }
    // store base alt in EEPROM for inflight recovery
    floatUnion.val = baro.baseAlt;
    if (settings.inflightRecover != 0)
    {
      pushFlightAction(actions, FlightAction::EEPROM_BASE_ALT);
      pushFlightAction(actions, FlightAction::EEPROM_LAST_EVENT, radio.event);
    }
  }

  if (events.liftoff)
  {
    actions.inFlight = true;

    // capture the cycles between samples
    if (settings.testMode)
    {
      cyclesBtwn++;
    }

    // set cycle timestamp
    fltTime.dt = fltTime.tmClock - fltTime.tmClockPrev;
    fltTime.timeCurrent += fltTime.dt;
    fltTime.tmClockPrev = fltTime.tmClock;

    // Update the moving average which greatly improves accuracy with minimal latency
    if (highG.newSamp)
    {
      highGSum -= highGFilter[filterPosn];
      highGFilter[filterPosn] = highG.z;
      highGSum += highGFilter[filterPosn];
      filterPosn++;
      if (filterPosn >= sizeHighGFilter)
      {
        filterPosn = 0;
        if (!filterFull)
        {
          filterFull = true;
        }
      }
      if (!filterFull)
      {
        highGSmooth = highGSum / filterPosn;
      }
      else
      {
        highGSmooth = highGSum / sizeHighGFilter;
      }
    }

    // Compute the current g-load. Use the high-G accelerometer if the IMU is pegged and a high-G accelerometer is present
    if (abs(accel.z) < accel.adcMax)
    {
      accelNow = (float)(accel.z - accel1G) * accel.gainZ;
    }
    else if (sensors.highG != 0)
    {
      accelNow = (float)(highG.z - high1G) * highG.gainZ;
    }
    else
    {
      accelNow = (float)(accel.z - accel1G) * accel.gainZ;
    }

    // Integrate velocity and altitude data prior to apogee
    if (!events.apogeeFire)
    {

      // Capture the max acceleration
      if (accelNow > maxG)
      {
        maxG = accelNow;
      }

      // calculate the new acceleration based velocity
      // this makes the apogee event mach immune
      accelVel += accelNow * (float)fltTime.dt * MILLIONTH;

      // calculate the new acceleration based altitude
      accelAlt += accelVel * (float)fltTime.dt * MILLIONTH;

      // calculate the new sensor fusion based velocity
      fusionVel += accelNow * (float)fltTime.dt * MILLIONTH;
      if (baro.newSamp && baro.vel < FUSION_BARO_VEL_MAX_MPS && baro.vel < baro.maxVel && accelNow < FUSION_ACCEL_MAX_MPS2 && fusionVel < FUSION_VEL_MAX_MPS && baro.alt < FUSION_VEL_BARO_CEILING_M)
      {
        fusionVel *= FUSION_VEL_KEEP;
        fusionVel += FUSION_VEL_BARO_WEIGHT * baro.vel;
      }
      radio.vel = (int16_t)fusionVel;

      // update maximum velocity if it exceeds the previous value
      if (fusionVel > maxVelocity)
      {
        maxVelocity = fusionVel;
      }

      // calculate the new sensor fusion based altitude
      fusionAlt += fusionVel * (float)fltTime.dt * MILLIONTH;
      if (baro.newSamp && baro.alt < FUSION_ALT_BARO_CEILING_M)
      {
        fusionAlt *= FUSION_ALT_KEEP;
        fusionAlt += FUSION_ALT_BARO_WEIGHT * baro.alt;
      }
      if (!altOk && (fusionAlt > settings.altThreshold || settings.testMode))
      {
        altOk = true;
      }
      radio.alt = (int16_t)fusionAlt;

    } // end if !apogee

    // caluclate the partial rotation in millionths of a degree, maintaining integer math for speed and retaining all possible precision
    gyroDeltaX += gyro.x * fltTime.dt;
    gyroDeltaY += gyro.y * fltTime.dt;
    gyroDeltaZ += gyro.z * fltTime.dt;

    // if active stablization is on, then use the more stable rotation algorithm (Rotation.cpp)
    if (settings.stableRotn || settings.stableVert)
    {
      getDcm2dRotn(gyroDeltaX, gyroDeltaY, gyroDeltaZ, gyro.gainZ);
      gyroDeltaX = gyroDeltaY = gyroDeltaZ = 0L;
    }

    // else update the quaternion rotation if we are not using active stabilization(Rotation.cpp)
    else if (fltTime.timeCurrent - lastRotn > rotnRate)
    {
      getQuatRotn(gyroDeltaX, gyroDeltaY, gyroDeltaZ, gyro.gainZ);
      gyroDeltaX = gyroDeltaY = gyroDeltaZ = 0L;
      lastRotn = fltTime.timeCurrent;
    }

    // run event logic
    checkEvents(actions);
  }

  return actions;
}

/**
 * @brief Carry out a flight step's actions, in order.
 *
 * @tparam Io Provides the board's I/O as static functions: pinWrite(pin, level),
 * eepromUpdate(address, value), radioTimerEnd(), radioTimerBegin(microseconds), syncSd() and
 * println(text). flightcomp.cpp passes the board's; the host tests pass a recording one.
 */
template <typename Io>
inline __attribute__((always_inline)) void performFlightActions(const FlightActions &actions)
{
  for (uint8_t i = 0; i < actions.count; i++)
  {
    const uint8_t arg = actions.arg[i];
    switch (actions.kind[i])
    {
    case FlightAction::PIN_HIGH:
      Io::pinWrite(arg, HIGH);
      break;
    case FlightAction::PIN_LOW:
      Io::pinWrite(arg, LOW);
      break;
    case FlightAction::EEPROM_LAST_EVENT:
      Io::eepromUpdate(eeprom.lastEvent, arg);
      break;
    case FlightAction::EEPROM_BASE_ALT:
      for (byte j = 0; j < 4; j++)
      {
        Io::eepromUpdate(eeprom.baseAlt + j, floatUnion.bytes[j]);
      }
      break;
    case FlightAction::RADIO_INFLIGHT:
      Io::radioTimerEnd();
      Io::radioTimerBegin(pktInterval.inflight / pktInterval.samplesPerPkt);
      noInterrupts();
      buildPkt = true;
      interrupts();
      break;
    case FlightAction::RADIO_POSTFLIGHT:
      Io::radioTimerBegin(pktInterval.postFlight);
      break;
    case FlightAction::RADIO_PRELIFTOFF:
      Io::radioTimerBegin(pktInterval.preLiftoff);
      break;
    case FlightAction::SYNC_SD:
      Io::syncSd();
      break;
    case FlightAction::GNSS_LIFTOFF_TIME:
      gnss.liftoff.hour = GPS.time.hour();
      gnss.liftoff.minute = GPS.time.minute();
      gnss.liftoff.second = GPS.time.second();
      gnss.liftoff.mili = GPS.time.centisecond();
      break;
    case FlightAction::GNSS_TOUCHDOWN_TIME:
      gnss.touchdown.hour = GPS.time.hour();
      gnss.touchdown.minute = GPS.time.minute();
      gnss.touchdown.second = GPS.time.second();
      gnss.touchdown.mili = GPS.time.centisecond();
      break;
    case FlightAction::PRINT_SIMULATED_LIFTOFF:
      Io::println(" ");
      Io::println("Simulated Liftoff Detected!");
      break;
    case FlightAction::PRINT_LIVE_PYRO_ALARM:
      Io::println("Live-pyro alarm activated");
      break;
    case FlightAction::PRINT_FALSE_TRIGGER:
      Io::println("False Trigger Reset");
      break;
    }
  }
}
