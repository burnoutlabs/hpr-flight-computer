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
  PinHigh,               // set pin arg high
  PinLow,                // set pin arg low
  EepromLastEvent,       // write arg (a flightCodes value) to eeprom.lastEvent
  EepromBaseAlt,         // write the 4 bytes of floatUnion (baro.baseAlt) to eeprom.baseAlt
  RadioInflight,         // restart the radio timer at the inflight sample rate, build a packet now
  RadioPostflight,       // start the radio timer at the postflight packet rate
  RadioPreliftoff,       // start the radio timer at the preflight packet rate
  SyncSd,                // sync the SD card
  GnssLiftoffTime,       // copy the GNSS time into gnss.liftoff
  GnssTouchdownTime,     // copy the GNSS time into gnss.touchdown
  PrintSimulatedLiftoff, // test mode: report the liftoff
  PrintLivePyroAlarm,    // report that the live-pyro alarm is on
  PrintFalseTrigger,     // test mode: report a false-liftoff reset
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
    radio.event = Time_Limit;
    if (settings.fltProfile == 'B')
    {
      radio.event = Booster_Time_Limit;
    }
    if (settings.inflightRecover != 0)
    {
      pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
    }
    pushFlightAction(actions, FlightAction::RadioPostflight);
    fltTime.touchdown = fltTime.timeCurrent;
    pushFlightAction(actions, FlightAction::GnssTouchdownTime);
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
        pushFlightAction(actions, FlightAction::PrintFalseTrigger);
      }
      // reset the key triggers
      events = resetEvents;
      fltTime.timeCurrent = 0UL;
      radio.fltTime = 0;
      radio.event = Preflight;
      if (settings.fltProfile == 'B')
      {
        radio.event = Booster_Preflight;
      }
      pktPosn = 0;
      radio.packetnum = 0;
      sampNum = 0;
      pushFlightAction(actions, FlightAction::RadioPreliftoff);
      // reset the high-g filter
      filterPosn = 0;
      highGsum = 0;
      for (byte i = 0; i < (sizeof(highGfilter) / sizeof(highGfilter[0])); i++)
      {
        highGfilter[i] = 0;
      }
      filterFull = false;
      // reset the acceleration based altitude and velocity
      accelVel = 0;
      accelAlt = 0;
      // sync the SD card
      pushFlightAction(actions, FlightAction::SyncSd);
    }
  } // end falseLiftoffCheck

  // check for booster burnout: if the z acceleration is negative
  if (!events.boosterBurnout && events.liftoff && accel.z <= 0)
  {
    events.boosterBurnout = true;
    radio.event = Booster_Burnout;
    if (settings.inflightRecover != 0 && !settings.testMode)
    {
      pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
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
      radio.event = Liftoff;
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
      radio.event = Eject_Booster;
      if (settings.inflightRecover != 0 && !settings.testMode)
      {
        pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
      }
    }

    // Fire second stage
    if (!events.sustainerFireCheck && (!events.apogee || settings.testMode) && events.liftoff && events.boosterBurnout && events.boosterSeparation && !pyroFire && fltTime.timeCurrent - fltTime.boosterSeparation > settings.sustainerFireDelay)
    {
      events.sustainerFireCheck = true;
      postFlightCode = 1;
      fltTime.sustainerFireCheck = fltTime.timeCurrent;
      // Check for staging inhibit and fire if OK
      if (altOK && rotnOK)
      {
        events.sustainerFire = true;
        firePyros('I', actions);
        radio.event = Fire_Sustainer;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
      else if (!rotnOK && !altOK)
      {
        postFlightCode = 4;
        radio.event = NoFire_RotnAlt_Limit;
      }
      else if (!rotnOK)
      {
        postFlightCode = 3;
        radio.event = NoFire_Rotn_Limit;
      }
      else if (!altOK)
      {
        postFlightCode = 2;
        radio.event = NoFire_Alt_Limit;
      }
    }

    // Check for sustainer ignition
    if (!events.apogee && !events.sustainerIgnition && events.sustainerFire && accelNow > IGNITION_ACCEL_MPS2)
    {
      radio.event = Sustainer_Ignition;
      if (settings.inflightRecover != 0 && !settings.testMode)
      {
        pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
      }
      events.sustainerIgnition = true;
      fltTime.sustainerIgnition = fltTime.timeCurrent;
    }

    // Check for sustainer burnout
    if (!events.apogee && !events.sustainerBurnout && events.sustainerIgnition && accelNow < 0.0 && fltTime.timeCurrent - fltTime.sustainerIgnition > SUSTAINER_BURN_MIN_US)
    {
      radio.event = Sustainer_Burnout;
      if (settings.inflightRecover != 0 && !settings.testMode)
      {
        pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
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
      if (altOK && rotnOK)
      {
        events.airStart1Fire = true;
        fltTime.airStart1Fire = fltTime.timeCurrent;
        firePyros('1', actions);
        radio.event = Fire_Airstart1;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
      else if (!rotnOK && !altOK)
      {
        postFlightCode = 4;
        radio.event = NoFire_RotnAlt_Limit;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
      else if (!rotnOK)
      {
        postFlightCode = NoFire_Rotn_Limit;
        radio.event = 18;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
      else if (!altOK)
      {
        postFlightCode = 2;
        radio.event = NoFire_Alt_Limit;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
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
      if (altOK && rotnOK)
      {
        events.airStart1Fire = true;
        fltTime.airStart1Fire = fltTime.timeCurrent;
        firePyros('1', actions);
        radio.event = Fire_Airstart1;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
      else if (!rotnOK && !altOK)
      {
        postFlightCode = 4;
        radio.event = NoFire_RotnAlt_Limit;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
      else if (!rotnOK)
      {
        postFlightCode = 3;
        radio.event = NoFire_Rotn_Limit;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
      else if (!altOK)
      {
        postFlightCode = 2;
        radio.event = NoFire_Alt_Limit;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
    }

    // Look for AirStart 1 Ignition
    if (events.airStart1Fire && !events.apogee && !events.airStart1Ignition && accelNow > IGNITION_ACCEL_MPS2)
    {
      events.airStart1Ignition = true;
      fltTime.airStart1Ignition = fltTime.timeCurrent;
      radio.event = Airstart1_Ignition;
      if (settings.inflightRecover != 0 && !settings.testMode)
      {
        pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
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
        radio.event = Airstart1_Burnout;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
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
      if (altOK && rotnOK)
      {
        events.airStart2Fire = true;
        fltTime.airStart2Fire = fltTime.timeCurrent;
        firePyros('2', actions);
        radio.event = Fire_Airstart2;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
      else if (!rotnOK && !altOK)
      {
        postFlightCode = 4;
        radio.event = NoFire_RotnAlt_Limit;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
      else if (!rotnOK)
      {
        postFlightCode = 3;
        radio.event = NoFire_Rotn_Limit;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
      else if (!altOK)
      {
        postFlightCode = 2;
        radio.event = NoFire_Alt_Limit;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
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
      if (altOK && rotnOK)
      {
        events.airStart2Fire = true;
        fltTime.airStart2Fire = fltTime.timeCurrent;
        firePyros('2', actions);
        radio.event = Fire_Airstart2;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
      else if (!rotnOK && !altOK)
      {
        postFlightCode = 4;
        radio.event = NoFire_RotnAlt_Limit;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
      else if (!rotnOK)
      {
        postFlightCode = 3;
        radio.event = NoFire_Rotn_Limit;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
      else if (!altOK)
      {
        postFlightCode = 2;
        radio.event = NoFire_Alt_Limit;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
    }

    // Look for AirStart 2 Ignition
    if (events.airStart2Fire && !events.apogee && !events.airStart2Ignition && accelNow > IGNITION_ACCEL_MPS2)
    {
      events.airStart2Ignition = true;
      fltTime.airStart2Ignition = fltTime.timeCurrent;
      radio.event = Airstart2_Ignition;
      if (settings.inflightRecover != 0 && !settings.testMode)
      {
        pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
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
        radio.event = Airstart2_Burnout;
        if (settings.inflightRecover != 0 && !settings.testMode)
        {
          pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
        }
      }
    }
  } // End Airstart Flight Mode

  // Barometric based apogee detection: rocket must be below 9000m and barometric velocity < 0 and motor burnout is detected and accelometer velocity < 70 (needed for Mach proofing)
  bool baroApogee = (baro.Vel < BARO_APOGEE_VEL_MPS && accelVel < BARO_APOGEE_MACH_LOCK_MPS && (baro.Alt + baro.baseAlt) < BARO_APOGEE_CEILING_M) ? true : false;
  // Sensor fusion based apogee detection
  bool fusionApogee = (fusionVel < 0) ? true : false;
  // Check for apogee event
  if (!events.apogee && !pyroFire && events.boosterBurnout && !events.boosterBurnoutCheck && !events.falseLiftoffCheck && (baroApogee || fusionApogee))
  {
    events.apogee = true;
    fltTime.apogee = fltTime.timeCurrent;
    radio.event = Apogee;
    if (settings.fltProfile == 'B')
    {
      radio.event = Booster_Apogee;
    }
    if (settings.inflightRecover != 0 && !settings.testMode)
    {
      pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
    }
  }

  // Fire apgogee charge if the current time > apogeeTime + apogeeDelay
  if (!events.apogeeFire && events.apogee && fltTime.timeCurrent - fltTime.apogee >= settings.apogeeDelay)
  {
    events.apogeeFire = true;
    fltTime.apogeeFire = fltTime.timeCurrent;
    firePyros('A', actions);
    radio.event = Fire_Apogee;
    if (settings.fltProfile == 'B')
    {
      radio.event = Fire_Booster_Apogee;
    }
    if (settings.inflightRecover != 0 && !settings.testMode)
    {
      pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
    }
  }

  // Write the data to the card 3s after apogeeFire in case of crash or powerloss
  if (events.apogeeFire && !syncApogee && !settings.testMode && fltTime.timeCurrent - fltTime.apogeeFire >= SYNC_AFTER_APOGEE_US)
  {
    pushFlightAction(actions, FlightAction::SyncSd);
    syncApogee = true;
  }

  // Detect separation after apogee
  if (events.apogeeFire && !events.mainDeploy && accel.z > APOGEE_SEPARATION_G * g && fltTime.timeCurrent - fltTime.apogeeFire <= APOGEE_SEPARATION_WINDOW_US)
  {
    events.apogeeSeparation = true;
    fltTime.apogeeSeparation = fltTime.timeCurrent;
    radio.event = Separation;
    if (settings.fltProfile == 'B')
    {
      radio.event = Booster_Separation;
    }
    if (settings.inflightRecover != 0 && !settings.testMode)
    {
      pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
    }
  }

  // Fire main chute charge if the baro altitude is lower than the threshold and at least 1s has passed since apogee
  if (!events.mainDeploy && events.apogeeFire && baro.Alt < settings.mainDeployAlt && fltTime.timeCurrent - fltTime.apogeeFire >= MAIN_AFTER_APOGEE_MIN_US)
  {
    events.mainDeploy = true;
    fltTime.mainDeploy = fltTime.timeCurrent;
    firePyros('M', actions);
    radio.event = Fire_Mains;
    if (settings.fltProfile == 'B')
    {
      radio.event = Fire_Booster_Mains;
    }
    if (settings.inflightRecover != 0 && !settings.testMode)
    {
      pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
    }
  }

  // Detect deployment of the mains
  if (events.mainDeploy && fltTime.timeCurrent - fltTime.mainDeploy > CHUTE_CHECK_START_US && fltTime.timeCurrent - fltTime.mainDeploy < CHUTE_CHECK_END_US && accelNow > CHUTE_OPEN_ACCEL_MPS2)
  {
    radio.event = Under_Chute;
    if (settings.fltProfile == 'B')
    {
      radio.event = Booster_Under_Chute;
    }
    if (settings.inflightRecover != 0 && !settings.testMode)
    {
      pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
    }
  }

  // Write the data to the card 3s after mainDeploy in case of crash or powerloss
  if (events.mainDeploy && !syncMains && !settings.testMode && fltTime.timeCurrent - fltTime.mainDeploy >= SYNC_AFTER_MAIN_US)
  {
    pushFlightAction(actions, FlightAction::SyncSd);
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
        pushFlightAction(actions, FlightAction::PinLow, pyros[i].firePin);
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
  if (!events.touchdown && events.mainDeploy && !pyroFire && !settings.testMode && baroTouchdown > touchdownTrigger && baro.Alt < TOUCHDOWN_MAX_ALT_M)
  {
    events.touchdown = true;
    events.inFlight = false;
    events.postFlight = true;
    fltTime.touchdown = fltTime.timeCurrent;
    radio.event = Touchdown;
    if (settings.fltProfile == 'B')
    {
      radio.event = Booster_Touchdown;
    }
    if (settings.inflightRecover != 0 && !settings.testMode)
    {
      pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
    }
    pushFlightAction(actions, FlightAction::GnssTouchdownTime);
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
      pushFlightAction(actions, FlightAction::PrintSimulatedLiftoff);
    }
    // set liftoff clock times
    fltTime.padTime = fltTime.tmClockPrev = fltTime.tmClock;
    fltTime.liftoff = fltTime.tmClock;
    // update events
    events.preLiftoff = false;
    events.liftoff = true;
    events.inFlight = true;
    // set radio variables
    radio.event = Liftoff;
    radio.alt = 0;
    pushFlightAction(actions, FlightAction::RadioInflight);
    // set GNSS liftoff values
    pushFlightAction(actions, FlightAction::GnssLiftoffTime);
    gnss.maxAlt = 0.0F;
    // if live pyros are detected, turn on the alarm to alert the user that the system is active and may fire pyros
    if (pyros[0].contStatus || pyros[1].contStatus || pyros[2].contStatus || pyros[3].contStatus || settings.testMode)
    {
      livePyroAlarm = true;
      pushFlightAction(actions, FlightAction::PrintLivePyroAlarm);
    }
    // store base alt in EEPROM for inflight recovery
    floatUnion.val = baro.baseAlt;
    if (settings.inflightRecover != 0)
    {
      pushFlightAction(actions, FlightAction::EepromBaseAlt);
      pushFlightAction(actions, FlightAction::EepromLastEvent, radio.event);
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
      highGsum -= highGfilter[filterPosn];
      highGfilter[filterPosn] = highG.z;
      highGsum += highGfilter[filterPosn];
      filterPosn++;
      if (filterPosn >= sizeHighGfilter)
      {
        filterPosn = 0;
        if (!filterFull)
        {
          filterFull = true;
        }
      }
      if (!filterFull)
      {
        highGsmooth = highGsum / filterPosn;
      }
      else
      {
        highGsmooth = highGsum / sizeHighGfilter;
      }
    }

    // Compute the current g-load. Use the high-G accelerometer if the IMU is pegged and a high-G accelerometer is present
    if (abs(accel.z) < accel.adcMax)
    {
      accelNow = (float)(accel.z - g) * accel.gainZ;
    }
    else if (sensors.highG != 0)
    {
      accelNow = (float)(highG.z - high1G) * highG.gainZ;
    }
    else
    {
      accelNow = (float)(accel.z - g) * accel.gainZ;
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
      accelVel += accelNow * (float)fltTime.dt * mlnth;

      // calculate the new acceleration based altitude
      accelAlt += accelVel * (float)fltTime.dt * mlnth;

      // calculate the new sensor fusion based velocity
      fusionVel += accelNow * (float)fltTime.dt * mlnth;
      if (baro.newSamp && baro.Vel < FUSION_BARO_VEL_MAX_MPS && baro.Vel < baro.maxVel && accelNow < FUSION_ACCEL_MAX_MPS2 && fusionVel < FUSION_VEL_MAX_MPS && baro.Alt < FUSION_VEL_BARO_CEILING_M)
      {
        fusionVel *= FUSION_VEL_KEEP;
        fusionVel += FUSION_VEL_BARO_WEIGHT * baro.Vel;
      }
      radio.vel = (int16_t)fusionVel;

      // update maximum velocity if it exceeds the previous value
      if (fusionVel > maxVelocity)
      {
        maxVelocity = fusionVel;
      }

      // calculate the new sensor fusion based altitude
      fusionAlt += fusionVel * (float)fltTime.dt * mlnth;
      if (baro.newSamp && baro.Alt < FUSION_ALT_BARO_CEILING_M)
      {
        fusionAlt *= FUSION_ALT_KEEP;
        fusionAlt += FUSION_ALT_BARO_WEIGHT * baro.Alt;
      }
      if (!altOK && (fusionAlt > settings.altThreshold || settings.testMode))
      {
        altOK = true;
      }
      radio.alt = (int16_t)fusionAlt;

    } // end if !apogee

    // caluclate the partial rotation in millionths of a degree, maintaining integer math for speed and retaining all possible precision
    dx += gyro.x * fltTime.dt;
    dy += gyro.y * fltTime.dt;
    dz += gyro.z * fltTime.dt;

    // if active stablization is on, then use the more stable rotation algorithm (Rotation.cpp)
    if (settings.stableRotn || settings.stableVert)
    {
      getDCM2DRotn(dx, dy, dz, gyro.gainZ);
      dx = dy = dz = 0L;
    }

    // else update the quaternion rotation if we are not using active stabilization(Rotation.cpp)
    else if (fltTime.timeCurrent - lastRotn > rotnRate)
    {
      getQuatRotn(dx, dy, dz, gyro.gainZ);
      dx = dy = dz = 0L;
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
    case FlightAction::PinHigh:
      Io::pinWrite(arg, HIGH);
      break;
    case FlightAction::PinLow:
      Io::pinWrite(arg, LOW);
      break;
    case FlightAction::EepromLastEvent:
      Io::eepromUpdate(eeprom.lastEvent, arg);
      break;
    case FlightAction::EepromBaseAlt:
      for (byte j = 0; j < 4; j++)
      {
        Io::eepromUpdate(eeprom.baseAlt + j, floatUnion.Byte[j]);
      }
      break;
    case FlightAction::RadioInflight:
      Io::radioTimerEnd();
      Io::radioTimerBegin(pktInterval.inflight / pktInterval.samplesPerPkt);
      noInterrupts();
      buildPkt = true;
      interrupts();
      break;
    case FlightAction::RadioPostflight:
      Io::radioTimerBegin(pktInterval.postFlight);
      break;
    case FlightAction::RadioPreliftoff:
      Io::radioTimerBegin(pktInterval.preLiftoff);
      break;
    case FlightAction::SyncSd:
      Io::syncSd();
      break;
    case FlightAction::GnssLiftoffTime:
      gnss.liftoff.hour = GPS.time.hour();
      gnss.liftoff.minute = GPS.time.minute();
      gnss.liftoff.second = GPS.time.second();
      gnss.liftoff.mili = GPS.time.centisecond();
      break;
    case FlightAction::GnssTouchdownTime:
      gnss.touchdown.hour = GPS.time.hour();
      gnss.touchdown.minute = GPS.time.minute();
      gnss.touchdown.second = GPS.time.second();
      gnss.touchdown.mili = GPS.time.centisecond();
      break;
    case FlightAction::PrintSimulatedLiftoff:
      Io::println(" ");
      Io::println("Simulated Liftoff Detected!");
      break;
    case FlightAction::PrintLivePyroAlarm:
      Io::println("Live-pyro alarm activated");
      break;
    case FlightAction::PrintFalseTrigger:
      Io::println("False Trigger Reset");
      break;
    }
  }
}
