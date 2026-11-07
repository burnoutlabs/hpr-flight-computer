/**
 * @file Rotation.cpp
 * @brief Functions and routines for rocket flight control and stabilization.
 *
 * @details
 * This file contains the following functions:
 * - `getQuatRotn()`: Calculates quaternion rotation at 100Hz using only gyroscope data.
 * - `magRotn()`: Computes roll and heading from the magnetometer.
 * - `getRotnDCM2D()`: Uses an optimized DCM code to calculate the 2D rotation for active stabilization.
 * - `setCanards()`: Controls active stabilization using canards.
 * - `setRtb()`: Implements control logic for a rocket's return-to-base feature.
 *
 * Change Log:
 * - 17 JUL 21: Initial file breakout created.
 * - 02 AUG 21: Revised PID control system, fixed bugs in DCM2D.
 * - 26 DEC 21: Initial coding for return system.
 * - 11 MAY 24: Updated for the Teensy 4.0/4.1 FPU at 64-bit precision.
 */

#include <Arduino.h>

#include "Rotation.h"
#include "declarations.h"
#include "flightcomp.h"
#include "Speed_Trig.h"

// getQuatRotn() state, at file scope rather than as function-local statics so that
// the host tests can reset it between cases. Rotation (quat) and roll-unwrapping
// holders; the types differ between the double and float paths as before.
#if defined(__IMXRT1062__)
static double quat[5] = {0.0, 1.0, 0.0, 0.0, 0.0};
static double prevRollZ = 0;
static double quatRollZ = 0;
static double fullRollZ = 0;
#else
static float quat[5] = {0.0, 1.0, 0.0, 0.0, 0.0};
static long prevRollZ = 0;
static long quatRollZ = 0;
static long fullRollZ = 0;
#endif

/**
 * @brief Calculate quaternion rotation at 100Hz using only gyroscope data.
 *
 * On Teensy 4.0/4.1 (`__IMXRT1062__`) the calculation runs in double precision on the
 * 64-bit FPU; on the other boards it runs in single precision (float).
 *
 * @param dx Gyroscope data along the X-axis.
 * @param dy Gyroscope data along the Y-axis.
 * @param dz Gyroscope data along the Z-axis.
 * @param gyroGain Gain factor for the gyroscope.
 */
#if defined(__IMXRT1062__)
void getQuatRotn(double dx, double dy, double dz, double gyroGain)
{

  // Local Vectors
  double quatDiff[5];
  double rotn1[4];
  double rotn2[4];
  double rotn3[4];
  double quatLen;
  double a, b, c, d, a2, b2, c2, d2, ab, ac, ad, bc, bd, cd;

  const double rotn2rad = gyroGain * DEG_TO_RAD / 1000000; // divide by 1000000 since dx,dy,dz are millionths of a degree

#else

void getQuatRotn(float dx, float dy, float dz, float gyroGain)
{
  // Local Vectors
  float quatDiff[5];
  float rotn1[4];
  float rotn2[4];
  float rotn3[4];
  float quatLen;
  float a, b, c, d, a2, b2, c2, d2, ab, ac, ad, bc, bd, cd;

  const float rotn2rad = gyroGain * DEG_TO_RAD / 1000000; // divide by 1000000 since dx,dy,dz are millionths of a degree

#endif

  // convert to radians
  dx *= rotn2rad;
  dy *= rotn2rad;
  dz *= rotn2rad;

  // Compute quaternion derivative
  quatDiff[1] = 0.5 * (-1 * dx * quat[2] - dy * quat[3] - dz * quat[4]);
  quatDiff[2] = 0.5 * (dx * quat[1] - dy * quat[4] + dz * quat[3]);
  quatDiff[3] = 0.5 * (dx * quat[4] + dy * quat[1] - dz * quat[2]);
  quatDiff[4] = 0.5 * (-1 * dx * quat[3] + dy * quat[2] + dz * quat[1]);

  // Update the quaternion
  quat[1] += quatDiff[1];
  quat[2] += quatDiff[2];
  quat[3] += quatDiff[3];
  quat[4] += quatDiff[4];

  // re-normalize
  quatLen = powf(quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3] + quat[4] * quat[4], -0.5);
  quat[1] *= quatLen;
  quat[2] *= quatLen;
  quat[3] *= quatLen;
  quat[4] *= quatLen;

  // compute the components of the rotation matrix
  a = quat[1];
  b = quat[2];
  c = quat[3];
  d = quat[4];
  a2 = a * a;
  b2 = b * b;
  c2 = c * c;
  d2 = d * d;
  ab = a * b;
  ac = a * c;
  ad = a * d;
  bc = b * c;
  bd = b * d;
  cd = c * d;

  // Compute rotation matrix
  rotn1[1] = a2 + b2 - c2 - d2;
  // rotn1[2] = 2 * (bc - ad);
  // rotn1[3] = 2 * (bd + ac);
  rotn2[1] = 2 * (bc + ad);
  // rotn2[2] = a2 - b2 + c2 - d2;
  // rotn2[3] = 2 * (cd - ab);
  rotn3[1] = 2 * (bd - ac);
  rotn3[2] = 2 * (cd + ab);
  rotn3[3] = a2 - b2 - c2 + d2;

  // compute 3D orientation
  pitchX = speedAtan2(rotn3[2], rotn3[3]); // this returns tenths of a degree, not whole degrees
  yawY = speedArcSin(-1 * rotn3[1]);       // this returns tenths of a degree, not whole degrees

  prevRollZ = quatRollZ;
  quatRollZ = speedAtan2(rotn2[1], rotn1[1]);
  if (quatRollZ - prevRollZ > 1800)
  {
    fullRollZ--;
  }
  else if (quatRollZ - prevRollZ < -1800)
  {
    fullRollZ++;
  }
  rollZ = (fullRollZ * 3600 + quatRollZ) * .1; // this is in whole degrees since its usually MUCH bigger than pitch and yaw

  // Compute angle off vertical
  float tanYaw = speedTan(yawY);
  float tanPitch = speedTan(pitchX);
  float hyp1 = tanYaw * tanYaw + tanPitch * tanPitch;
  float hyp2 = powf(hyp1, 0.5);
  offVert = speedArcTan(hyp2); // this returns tenths of a degree, not whole degrees

  // as long as the rocket never exceeded 45 degrees, set the rotnOk variable to whether or not the current off-vertical rotation is within the allowable cone
  rotnOk = (!rotationFault && offVert < settings.maxAngle) ? true : false;

  // if the rocket goes outside a 45 degree cone, then permanently shut-down all airstart events.  The rocket is probably skywriting or experienced a failure
  if (!rotationFault && offVert > 450)
  {
    rotationFault = true;
    rotnOk = false;
  }

} // end getQuatRotn

// to reduce trigonometric function calls, these are global variables since they are also used by setCanards()
float cosZ = 1.0F;
float sinZ = 0.0F;

// getDcm2dRotn() state, at file scope so that the host tests can reset it
static float rawX = 0.0F;
static float rawY = 0.0F;
static float rawZ = 0.0F;

/**
 * @brief Optimized DCM code to calculate the rotation on each cycle of the loop.
 * @param dx Gyroscope data along the X-axis.
 * @param dy Gyroscope data along the Y-axis.
 * @param dz Gyroscope data along the Z-axis.
 * @param gyroGain Gain factor for the gyroscope.
 */
void getDcm2dRotn(long dx, long dy, long dz, float gyroGain)
{
  // this is a simplified Euler method that ONLY works for the simple rotations experienced by rockets (i.e. approximately a 2D arc to apogee)
  // this method will NOT give accurate results for complex rotations (i.e. significant coning or tumbling), use quaternions instead for maximum accuracy
  // rigorous testing shows a typical difference of 2-3 degrees (maximum) between this method and quaternions
  // this simplified algorithm can be employed on modest processors since it uses minimal floating point operations and simplified trigonometry
  // it has been successfully deployed on the Arduino Uno and Adafruit Metro Mini

  // Calculate new Z angle from gyro data
  rawZ += dz;
  float radNewRoll = dz * gyroGain * DEG_TO_RAD * MILLIONTH; // convert to radians

  // compute sin(roll) and cos(roll)
  float sinNewRoll = sinSmallAngle(radNewRoll);
  float cosNewRoll = cosSmallAngle(radNewRoll);
  float prevCosZ = cosZ;
  float prevSinZ = sinZ;
  cosZ = prevCosZ * cosNewRoll - prevSinZ * sinNewRoll;
  sinZ = prevSinZ * cosNewRoll + prevCosZ * sinNewRoll;

  // Compute the new y and x angles
  rawX += (cosZ * dx - sinZ * dy);
  rawY += (cosZ * dy + sinZ * dx);

  // Compute Pitch, Roll, Yaw
  rollZ = (long)((rawZ * gyroGain) * MILLIONTH);
  pitchX = (int)((rawX * gyroGain) * MILLIONTH * 10); // we report the value in tenths of a degree, not whole degrees
  yawY = (int)((rawY * gyroGain) * MILLIONTH * 10);

  // Compute off-vertical
  float tanYaw = speedTan(yawY);
  tanYaw *= tanYaw;
  float tanPitch = speedTan(pitchX);
  tanPitch *= tanPitch;
  offVert = speedArcTan(powf(tanYaw + tanPitch, 0.5));

  // Check if the max angle is exceeded, shutdown staging if angle > 45 is detected
  if ((!events.apogee) || settings.testMode)
  {

    // check to see if the maximum angle has been exceeded
    if (!rotationFault && offVert > settings.maxAngle)
    {
      rotnOk = false;
    }
    if (!rotationFault && offVert > 450)
    {
      rotationFault = true;
      rotnOk = false;
    }
  }
} // end void

float rollError;
float pitchError;
float yawError;
float rollPosn;

/**
 * @brief Control code for active stabilization using canards.
 */
void setCanards()
{

  const float kpYawPitch = 0.03;
  const float kiYawPitch = 0.0;
  const float kdYawPitch = 0.0;
  const float kpRoll = 0.0015;
  const float kiRoll = 0.0;
  const float kdRoll = 0.0005;
  const float rollSetPoint = 0.0F;
  const float pitchSetPoint = 0.0F;
  const float yawSetPoint = 0.0F;
  uint32_t cdt;
  static uint32_t lastUpdate = 0UL;
  static float rollErrorInt = 0.0F;
  static float pitchErrorInt = 0.0F;
  static float yawErrorInt = 0.0F;
  float sineRollThrowServo1;
  float sineRollThrowServo2;
  float sineRollThrowServo3;
  float sineRollThrowServo4;
  float sineYawThrowServo1;
  float sineYawThrowServo2;
  float sinePitchThrowServo3;
  float sinePitchThrowServo4;

  float rollTorqueInput = 0.0;
  float pitchTorqueInput = 0.0;
  float yawTorqueInput = 0.0;

  const float rollTorqueArm = 0.046482;    // 1.83in = 0.0464m
  const float yawPitchTorqueArm = 0.46482; // 18in = 0.464m
  float maxTorqueInput = 4.0;

  // Update time stamps
  uint32_t timeNow = micros();
  cdt = timeNow - lastUpdate;
  lastUpdate = timeNow;

  // Determine current position error
  rollError = rollZ % 360 - rollSetPoint;
  pitchError = pitchX * 0.1 - pitchSetPoint; // pitch is reported in tenths of a degree
  yawError = yawY * 0.1 - yawSetPoint;       // yaw is reported in tenths of a degree

  // Calculate air density
  const float R = 287.05; // R value for dry air
  float airDensity = (baro.pressure * 100) / (R * (baro.temperature + 273.15));

  // Calculate the fin drag force
  const float Cd = 0.5;             // estimate of the fin Cd
  const float finArea = 0.00064516; // area of each canard in m^2
  float currentVel = fusionVel;
  if (fabs(currentVel) < 1.0F)
  {
    currentVel = 1.0F;
  }
  if (settings.testMode)
  {
    currentVel = 100.0F;
  }
  float dragForce = 0.5 * Cd * currentVel * currentVel * finArea * airDensity;

  //-----------------------
  // Servos 1 & 2 must be aligned to the same plane
  // Servos 1 & 2 must be aligned to the board Y-axis
  // Servos 3 & 4 must be aligned to the same plane
  // Servos 3 & 4 must be alinged to the board X-axis
  //-----------------------
  // Update roll correction
  //-----------------------
  if (settings.stableRotn)
  {

    // Update Integral Terms
    rollErrorInt += rollError * cdt * MILLIONTH;

    // Prevent Integral Wind-Up in bench-test mode
    if (settings.testMode)
    {
      rollErrorInt = 0;
    }

    // Update roll position correction
    rollTorqueInput = kpRoll * rollError + kiRoll * rollErrorInt + kdRoll * gyro.z * gyro.gainZ;
    if (fabs(rollTorqueInput) > maxTorqueInput)
    {
      rollTorqueInput = ((rollTorqueInput > 0) ? 1 : -1) * maxTorqueInput;
    }

    // Determine the required torque to correct for roll
    const float numRollFins = 4.0F;
    float rollTorqueServo1 = rollTorqueInput / numRollFins;
    float rollTorqueServo2 = rollTorqueInput / numRollFins;
    float rollTorqueServo3 = rollTorqueInput / numRollFins;
    float rollTorqueServo4 = rollTorqueInput / numRollFins;

    //---------------------------------------------
    // Calculate the sine of the required throw angle
    //---------------------------------------------
    // torque = controlForce * torqueArmDistance
    // controlForce = sin(canardThrow) * finDragForce
    // finDragForce = 0.5 * Cd * velocity^2 * finArea * airDensity
    // canardThrow = asin(controlForce / finDragForce)
    sineRollThrowServo1 = rollTorqueServo1 / (rollTorqueArm * dragForce);
    sineRollThrowServo2 = rollTorqueServo2 / (rollTorqueArm * dragForce);
    sineRollThrowServo3 = rollTorqueServo3 / (rollTorqueArm * dragForce);
    sineRollThrowServo4 = rollTorqueServo4 / (rollTorqueArm * dragForce);
  }

  else
  {
    sineRollThrowServo1 = sineRollThrowServo2 = sineRollThrowServo3 = sineRollThrowServo4 = 0.0F;
  }

  //--------------------------------------------------------------------------------------------------------------
  // Update off-vertical error terms, restrict input to only when the roll is close to the original starting point
  //--------------------------------------------------------------------------------------------------------------
  if (settings.stableVert && fabs(rollError) < 30)
  {

    // Update integral terms
    pitchErrorInt += pitchError * cdt * MILLIONTH;
    yawErrorInt += yawError * cdt * MILLIONTH;

    // Prevent Integral Wind-Up in bench-test mode
    if (settings.testMode)
    {
      pitchErrorInt = yawErrorInt = 0;
    }

    // Update pitch & yaw correction positions, sinZ & cosZ are the sine and cosine of the roll angle
    pitchTorqueInput = kpYawPitch * pitchError + kiYawPitch * pitchErrorInt + kdYawPitch * (cosZ * gyro.gainX * gyro.x + sinZ * gyro.gainY * gyro.y);
    yawTorqueInput = kpYawPitch * yawError + kiYawPitch * yawErrorInt + kdYawPitch * (cosZ * gyro.gainY * gyro.y - sinZ * gyro.gainX * gyro.x);
    if (fabs(pitchTorqueInput) > maxTorqueInput)
    {
      pitchTorqueInput = ((rollTorqueInput > 0) ? 1 : -1) * maxTorqueInput;
    }
    if (fabs(yawTorqueInput) > maxTorqueInput)
    {
      yawTorqueInput = ((rollTorqueInput > 0) ? 1 : -1) * maxTorqueInput;
    }

    // Determine the required torque to correct for pitch and yaw
    const float numYawPitchFins = 2.0F;
    float yawTorqueServo1 = (sinZ * pitchTorqueInput + cosZ * yawTorqueInput) / numYawPitchFins;
    float yawTorqueServo2 = (sinZ * pitchTorqueInput + cosZ * yawTorqueInput) / numYawPitchFins;
    float pitchTorqueServo3 = (sinZ * yawTorqueInput + cosZ * pitchTorqueInput) / numYawPitchFins;
    float pitchTorqueServo4 = (sinZ * yawTorqueInput + cosZ * pitchTorqueInput) / numYawPitchFins;

    //---------------------------------------------
    // Calculate the sine of the required throw angle
    //---------------------------------------------
    // torque = controlForce * torqueArmDistance
    // controlForce = sin(canardThrow)
    // finDragForce = 0.5 * Cd * velocity^2 * finArea * airDensity
    // canardThrow = asin(controlForce / finDragForce)
    sineYawThrowServo1 = (yawTorqueServo1 / yawPitchTorqueArm) / dragForce;
    sineYawThrowServo2 = (yawTorqueServo2 / yawPitchTorqueArm) / dragForce;
    sinePitchThrowServo3 = (pitchTorqueServo3 / yawPitchTorqueArm) / dragForce;
    sinePitchThrowServo4 = (pitchTorqueServo4 / yawPitchTorqueArm) / dragForce;
  }

  else
  {
    sineYawThrowServo1 = sineYawThrowServo2 = sinePitchThrowServo3 = sinePitchThrowServo4 = 0.0F;
  }

  // Determine the sine of the total canard throw
  float sineThrowServo1 = sineRollThrowServo1 + sineYawThrowServo1;
  float sineThrowServo2 = sineRollThrowServo2 - sineYawThrowServo2;
  float sineThrowServo3 = sineRollThrowServo3 + sinePitchThrowServo3;
  float sineThrowServo4 = sineRollThrowServo4 - sinePitchThrowServo4;

  // control input corrections are limited to 45 degrees
  const float sine45 = 0.70710678; // sqrt(2)/2
  if (fabs(sineThrowServo1) > sine45)
  {
    sineThrowServo1 = ((sineThrowServo1 > 0) ? 1 : -1) * sine45;
  }
  if (fabs(sineThrowServo2) > sine45)
  {
    sineThrowServo2 = ((sineThrowServo2 > 0) ? 1 : -1) * sine45;
  }
  if (fabs(sineThrowServo3) > sine45)
  {
    sineThrowServo3 = ((sineThrowServo3 > 0) ? 1 : -1) * sine45;
  }
  if (fabs(sineThrowServo4) > sine45)
  {
    sineThrowServo4 = ((sineThrowServo4 > 0) ? 1 : -1) * sine45;
  }

  // translate to degrees
  const float rad2deg = 180 / M_PI;
  float throwServo1 = 90 + asinf(sineThrowServo1) * rad2deg;
  float throwServo2 = 90 + asinf(sineThrowServo2) * rad2deg;
  float throwServo3 = 90 + asinf(sineThrowServo3) * rad2deg;
  float throwServo4 = 90 + asinf(sineThrowServo4) * rad2deg;

  // if we have hit apogee, then center the canards
  if (events.apogee)
  {
    throwServo1 = throwServo2 = throwServo3 = throwServo4 = 90;
  }

  // Set the fin positions
  // fin pointed in the positive-y rocket-frame
  servos[0].write(throwServo1 - servoTrim[0]);
  // fin pointed in the negative-y rocket-frame
  servos[1].write(throwServo2 - servoTrim[1]);
  // fin pointed in the positive-x rocket-frame
  servos[2].write(throwServo3 - servoTrim[2]);
  // fin pointed in the negative-x rocket-frame
  servos[3].write(throwServo4 - servoTrim[3]);

  // output debugging over serial
  const boolean serialPlotter = true;
  if (settings.testMode && !serialPlotter)
  {
    Serial.print("***Time: ");
    Serial.println(fltTime.timeCurrent * MILLIONTH, 2);
    Serial.print("Roll, Pitch, Yaw: ");
    Serial.print(rollZ);
    Serial.print(", ");
    Serial.print(pitchX * .1, 2);
    Serial.print(", ");
    Serial.println(yawY * .1, 2);
    Serial.print("Error from set point: ");
    Serial.print(rollError);
    Serial.print(", ");
    Serial.print(pitchError);
    Serial.print(", ");
    Serial.println(yawError);
    Serial.print("Roll Input: ");
    Serial.println(rollTorqueInput, 2);
    Serial.print("Yaw Input: ");
    Serial.println(yawTorqueInput, 2);
    Serial.print("Pitch Input: ");
    Serial.println(pitchTorqueInput, 2);
    Serial.print("DragForce: ");
    Serial.println(dragForce, 2);
    Serial.print("Servo1: ");
    Serial.println(throwServo1);
    Serial.print("Servo2: ");
    Serial.println(throwServo2);
    Serial.print("Servo3: ");
    Serial.println(throwServo3);
    Serial.print("Servo4: ");
    Serial.println(throwServo4);
  }

  if (settings.testMode && serialPlotter)
  {
    // Error from set point
    // Serial.print("RollError:");Serial.print(rollError);Serial.print("\t");
    // Serial.print("PitchError:");Serial.print(pitchError);Serial.print("\t");
    // Serial.print("YawError:");Serial.print(yawError);Serial.print("\t");
    Serial.print("RollInput:");
    Serial.print(rollTorqueInput);
    Serial.print("\t");
    Serial.print("PitchInput:");
    Serial.print(pitchTorqueInput);
    Serial.print("\t");
    Serial.print("YawInput:");
    Serial.println(yawTorqueInput);
  }
} // end setCanards()

/**
 * @brief Implements control logic for a rocket's return-to-base feature.
 *
 * @details
 * This function calculates the torque required for the fins to correct the rocket's bearing to return to a specified point.
 * It uses PID control for position correction and employs a predictive model for landing points, considering factors like
 * rate of descent, current altitude, and distance from the landing point. The function sets servo throws to correct for
 * roll and bearing.
 */
void setRtb()
{

  const float rollTorqueArm = 0.046482; // 1.83in = 0.0464m
  // const float bearingTorqueArm =  0.46482;//18in = 0.464m
  float maxTorqueInput = 4.0;
  const float kpLandPoint = 0.03;
  const float kiLandPoint = 0.0;
  const float kdLandPoint = 0.0;
  const float kpRoll = 0.0015;
  const float kiRoll = 0.0;
  const float kdRoll = 0.0005;
  uint32_t cdt;
  static uint32_t lastUpdate = 0UL;
  static float bearingErrorInt = 0.0F;
  static float prevLandPointError = 0.0F;
  static float landPointErrorInt = 0.0F;

  // Update time stamps
  uint32_t timeNow = micros();
  cdt = timeNow - lastUpdate;
  lastUpdate = timeNow;

  // Calculate air density
  const float R = 287.05; // R value for dry air
  float airDensity = (baro.pressure * 100) / (R * (baro.temperature + 273.15));

  // Calculate the fin drag force
  const float Cd = 0.5;             // estimate of the fin Cd
  const float finArea = 0.00064516; // area of each canard in m^2
  float currentVel = fusionVel;
  if (fabs(currentVel) < 1.0F)
  {
    currentVel = 1.0F;
  }
  if (settings.testMode)
  {
    currentVel = 100.0F;
  }
  float dragForce = 0.5 * Cd * currentVel * currentVel * finArea * airDensity;

  // calculate the bearing to the return point using GPS coordinates
  // A = current position
  // B = desired position
  // X = cos(latitudeB) * sin(longitudeB - longitudeA)
  // Y = cos(latitudeA) * sin(latitudeB) - sin(latitudeA) * cos(latitudeB) * cos(longitudeB - longitudeA)

  float x = speedCos(gnss.latitude) * speedSin(gnss.liftoff.longitude - gnss.longitude);
  float y = speedCos(gnss.latitude) * speedSin(gnss.liftoff.latitude) - speedSin(gnss.latitude) * speedCos(gnss.liftoff.latitude) * speedCos(gnss.liftoff.longitude - gnss.longitude);

  float bearingSetpoint = speedAtan2(x, y);
  float magBearing = speedAtan2(mag.y, mag.x);
  // float trackBearing = GPS.course.deg();
  float bearingError = bearingSetpoint - magBearing;

  // Update Integral Terms
  bearingErrorInt += bearingError * cdt * MILLIONTH;

  // Prevent Integral Wind-Up in bench-test mode
  if (settings.testMode)
  {
    bearingErrorInt = 0;
  }

  // set the roll to the bearing with fins 3 & 4
  // use torque generation model from above
  // Update roll position correction
  float bearingTorqueInput = kpRoll * bearingError + kiRoll * bearingErrorInt + kdRoll * gyro.z * gyro.gainZ;
  if (fabs(bearingTorqueInput) > maxTorqueInput)
  {
    bearingTorqueInput = ((bearingTorqueInput > 0) ? 1 : -1) * maxTorqueInput;
  }

  // Determine the required torque to correct bearing
  const float numRollFins = 2.0F;
  float bearingTorqueServo3 = bearingTorqueInput / numRollFins;
  float bearingTorqueServo4 = bearingTorqueInput / numRollFins;

  //---------------------------------------------
  // Calculate the sine of the required throw angle
  //---------------------------------------------
  // torque = controlForce * torqueArmDistance
  // controlForce = sin(canardThrow) * finDragForce
  // finDragForce = 0.5 * Cd * velocity^2 * finArea * airDensity
  // canardThrow = asin(controlForce / finDragForce)
  float sineThrowServo3 = bearingTorqueServo3 / (rollTorqueArm * dragForce);
  float sineThrowServo4 = bearingTorqueServo4 / (rollTorqueArm * dragForce);

  // control input corrections are limited to 45 degrees
  const float sine45 = 0.70710678; // sqrt(2)/2
  if (fabs(sineThrowServo3) > sine45)
  {
    sineThrowServo3 = ((sineThrowServo3 > 0) ? 1 : -1) * sine45;
  }
  if (fabs(sineThrowServo4) > sine45)
  {
    sineThrowServo4 = ((sineThrowServo4 > 0) ? 1 : -1) * sine45;
  }

  // translate to degrees, but reverse direction since we are now descending
  const float rad2deg = 180 / M_PI;
  float throwServo3 = 90 - asinf(sineThrowServo3) * rad2deg;
  float throwServo4 = 90 - asinf(sineThrowServo4) * rad2deg;

  // if the mains have deployed, center the fins
  if (events.mainDeploy)
  {
    throwServo3 = throwServo4 = 90;
  }

  // Set the fin positions
  // fin pointed in the positive-x rocket-frame
  servos[2].write(throwServo3 - servoTrim[2]);
  // fin pointed in the negative-x rocket-frame
  servos[3].write(throwServo4 - servoTrim[3]);

  //-------------------------------------------------------------------------------------------------------------------
  // aim the vehicle body twoards the return point with fins 1 & 2
  // need a predictive model based on: rate of descent, current altitude, current distance from landing point
  // rate of descent = baro vel or GPS derived descent rate
  // current altitude = baro alt
  // distance from landing point = GPS calcs
  // GPS calculated ground speed
  // landing point = time to landing x ground speed
  // landing point = (baro alt - deployAlt) / baro vel x ground speed
  // servo throw will be proportional to the difference between predicted landing point and desired landing point
  // integral = how long the difference between predicted and desired is held
  // proportional = difference between predicted and desired
  // derivative = GPS calculated ground speed
  //-------------------------------------------------------------------------------------------------------------------

  // if the difference between the required bearing and the actual bearing is > 20 degrees,
  // then zero fins 1 & 2 so that it doesn't make the bearing differential worse
  float throwServo1;
  float throwServo2;
  if (fabs(bearingError) > 20)
  {
    throwServo1 = throwServo2 = 0;
  }

  // else estimate the landing point
  // float timeToLand = (gnss.alt - settings.mainDeployAlt) / fabs(gnss.vel);//subtract out the main deployment altitude because we want it back at the launch position when the mains come out
  float distanceToLandPoint = calcGpsDist(gnss.liftoff.longitude, gnss.liftoff.latitude, gnss.longitude, gnss.latitude);
  float gpsGroundSpeed = gps.speed.mps();
  float landPointEst = distanceToLandPoint / gpsGroundSpeed;
  float landPointError = distanceToLandPoint - landPointEst;

  // Update Integral Terms
  landPointErrorInt += landPointError * cdt * MILLIONTH;

  // calculate the controlThrow
  throwServo1 = kpLandPoint * landPointError + kiLandPoint * landPointErrorInt + kdLandPoint * (landPointError - prevLandPointError);
  throwServo2 = -1 * throwServo1;
  prevLandPointError = landPointError;

  // if the mains have deployed, center the fins
  if (events.mainDeploy)
  {
    throwServo1 = throwServo2 = 90;
  }

  // Set the fin positions
  // fin pointed in the positive-y rocket-frame
  servos[0].write(throwServo1 - servoTrim[0]);
  // fin pointed in the negative-y rocket-frame
  servos[1].write(throwServo2 - servoTrim[1]);

} // end setRtb

/**
 * @brief Calculate the distance between two GPS points.
 *
 * @param aLon Longitude of the first point.
 * @param aLat Latitude of the first point.
 * @param bLon Longitude of the second point.
 * @param bLat Latitude of the second point.
 * @return The distance between the two points in meters.
 *
 * @details
 * This function calculates the distance between two GPS points, converting latitude and longitude degrees into meters.
 * It uses a simple interpolation table to convert longitude degrees to meters, considering different interpolation rates
 * depending on the range of the longitude difference.
 */
float calcGpsDist(float aLon, float aLat, float bLon, float bLat)
{

  // calculate the difference between the points
  float diffLat = aLat - bLat;
  float diffLon = aLon - bLon;
  // determine the maximum latitude for interpolation below
  float absAlat = fabs(aLat);
  float absBlat = fabs(bLat);
  float maxLat = fabs(aLat) > fabs(bLat) ? absAlat : absBlat;

  // convert latitude degrees to meters
  diffLat *= 111111.1;
  diffLon *= (111111.1 * speedCos((int)(maxLat * 10)));

  return sqrtf(diffLat * diffLat + diffLon * diffLon);
} // end calcGpsDist

/**
 * @brief Compute rotation based on the magnetometer readings.
 */
void magRotn()
{

  long dotProd;
  long valVect1;
  long valVect2;
  long valVect3;
  float lenVect1;
  float lenVect2;
  float lenVect3;

  // compute roll
  dotProd = mag.x0 * mag.x + mag.y0 * mag.y;
  valVect1 = mag.x0 * mag.x0 + mag.y0 * mag.y0;
  valVect2 = mag.x * mag.x + mag.y * mag.y;
  lenVect1 = sqrtf(valVect1);
  lenVect2 = sqrtf(valVect2);

  float cosMagRoll = dotProd / (lenVect1 * lenVect2);
  magRoll = speedArcCos(cosMagRoll);

  // compute off vertical
  dotProd = valVect2;
  valVect3 = valVect2 + mag.z * mag.z;
  lenVect3 = sqrtf(valVect3);

  float cosOffVert = dotProd / (lenVect2 * lenVect3);
  magOffVert = speedArcCos(cosOffVert);

  // reduce roll to -360 to 360
  long tempRoll = magRoll;
  while (tempRoll > 3600)
  {
    tempRoll -= 3600;
  }
  while (tempRoll < 3600)
  {
    tempRoll += 3600;
  }

  // compute pitch
  magPitch = magOffVert * speedCos(tempRoll);

  // compute yaw
  magYaw = magOffVert * speedSin(tempRoll);
} // end magRotn