/**
 * @file FlightStep.cpp
 * @brief Barometer sample processing, which feeds the flight step (FlightStep.h).
 *
 * @details
 * processBaroSamp() turns each new barometer altitude into the pad altitude (before liftoff and
 * after landing), the altitude above the pad, its maximum, the smoothed altitude, the barometric
 * velocity and the touchdown counter. loop() calls it whenever a barometer sample is new, before
 * flightStep(). It is defined here, not in flightcomp.cpp, so the host tests can build it.
 */
#include <Arduino.h>

#include "declarations.h"
#include "FlightStep.h"

void processBaroSamp()
{

  //----------------------------
  // process preliftoff variables
  //----------------------------
  static float sumBaseAlt = 0.0F;
  static uint8_t baseAltPosn = 0;
  static float baseAltBuff[30] = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                                  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                                  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  if (events.preLiftoff || events.touchdown || events.timeOut)
  {
    sumBaseAlt -= baseAltBuff[baseAltPosn];
    sumBaseAlt += baro.rawAlt;
    baseAltBuff[baseAltPosn] = baro.rawAlt;
    baseAltPosn++;
    const uint8_t sizeBaseAltBuff = sizeof(baseAltBuff) / sizeof(baseAltBuff[0]);
    if (baseAltPosn >= sizeBaseAltBuff)
    {
      baseAltPosn = 0;
    }
    baro.baseAlt = sumBaseAlt / sizeBaseAltBuff;
    radioTelemetry.baseAlt = (int16_t)baro.baseAlt;
    baro.newSamp = false;
  }

  //--------------------------
  // Update in-flight variables
  //--------------------------
  baro.alt = baro.rawAlt - baro.baseAlt;
  // update the maximum altitude reporting variable
  if (baro.alt > baro.maxAlt && !events.apogee)
  {
    baro.maxAlt = baro.alt;
  }

  //----------------------------
  // Smoothed barometric altitude
  //----------------------------
  static float rawAltSum = 0.0;
  static float rawAltBuff[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static byte rawAltPosn = 0;
  // Update the rolling sum
  rawAltSum -= rawAltBuff[rawAltPosn];
  rawAltBuff[rawAltPosn] = baro.alt;
  rawAltSum += rawAltBuff[rawAltPosn];
  // update the position counter
  rawAltPosn++;
  static const float sizeRawAltBuff = sizeof(rawAltBuff) / sizeof(rawAltBuff[0]);
  if (rawAltPosn >= (byte)sizeRawAltBuff)
  {
    rawAltPosn = 0;
  }
  // calculate the smoothed barometric altitude
  baro.smoothAlt = rawAltSum / sizeRawAltBuff;

  //---------------------------
  // barometric derived velocity
  //---------------------------
  static uint8_t altAvgPosn = 0;
  static int16_t baroVelPosn = 0;
  static float altAvgBuff[30] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static unsigned long baroTimeBuff[30] = {0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
                                           0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
                                           0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL};
  // if we are past apogee, use the maximum difference between samples in the buffer, otherwise set the difference to 10 samples
  if (events.apogee)
  {
    baroVelPosn = altAvgPosn;
  }
  else
  {
    baroVelPosn = altAvgPosn - 10;
    if (baroVelPosn < 0)
    {
      baroVelPosn = (int)((sizeof(altAvgBuff) / sizeof(altAvgBuff[0])) - (10 - altAvgPosn));
    }
  }
  // calculate the barometric derived velocity from the moving averages
  baro.vel = (baro.smoothAlt - altAvgBuff[baroVelPosn]) / ((float)(baro.timeLastSamp - baroTimeBuff[baroVelPosn]) * MILLIONTH);
  // update variables
  if (baro.vel > baro.maxVel)
  {
    baro.maxVel = baro.vel;
  }
  radioTelemetry.vel = baro.vel;
  radioTelemetry.alt = baro.smoothAlt;
  altAvgBuff[altAvgPosn] = baro.smoothAlt;
  baroTimeBuff[altAvgPosn] = baro.timeLastSamp;
  altAvgPosn++;
  if (altAvgPosn >= (byte)(sizeof(altAvgBuff) / sizeof(altAvgBuff[0])))
  {
    altAvgPosn = 0;
  }
  if (events.apogeeFire || settings.testMode)
  {
    fusionVel = baro.vel;
    fusionAlt = baro.smoothAlt;
  }

  //----------------------------
  // Barometric touchdown trigger
  //----------------------------
  (events.mainDeploy && fabs(baro.vel) < 1.0F) ? baroTouchdown++ : baroTouchdown = 0;
}

void pushFlightAction(FlightActions &actions, FlightAction kind, uint8_t arg)
{
  if (actions.count < FLIGHT_ACTIONS_MAX)
  {
    actions.kind[actions.count] = kind;
    actions.arg[actions.count] = arg;
    actions.count++;
  }
}

void firePyros(char event, FlightActions &actions)
{
  pyroFire = true;
  for (uint8_t i = 0; i < 4; i++)
  {
    if (pyros[i].func == event)
    {
      pins.firePin = pyros[i].firePin;
      pushFlightAction(actions, FlightAction::PIN_HIGH, pyros[i].firePin);
      pyros[i].fireStatus = true;
      pyros[i].fireStart = fltTime.timeCurrent;
    }
  }
}

void pulsePyro(FlightActions &actions)
{
  // whether each channel's pin is high in its pulse cycle
  static boolean pinHigh[4] = {false, false, false, false};

  for (uint8_t i = 0; i < 4; i++)
  {
    if (pyros[i].fireStatus)
    {
      if (pinHigh[i])
      {
        pushFlightAction(actions, FlightAction::PIN_LOW, pyros[i].firePin);
        pinHigh[i] = false;
      }
      else
      {
        pushFlightAction(actions, FlightAction::PIN_HIGH, pyros[i].firePin);
        pinHigh[i] = true;
      }
    }
  }
}
