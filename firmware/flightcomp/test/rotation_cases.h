/**
 * @file rotation_cases.h
 * @brief Rotation.cpp tests, shared by the Teensy 4.x (double) and 3.x (float) suites.
 *
 * Each suite's .cpp chooses the getQuatRotn() path, includes flightcomp_subset.h,
 * then this file. The cases drive getQuatRotn() and getDcm2dRotn() as the flight
 * loop does: once per 10 ms, with each axis's input the sum over that interval of
 * gyro reading (LSB) x sample time (us). At a gain of 0.07 deg/s/LSB (the value
 * several of the supported gyro drivers set), a constant 1000 LSB reading is
 * 70 deg/s, which is 0.7 deg per call.
 *
 * Expected angles come from an independent model: the exact rotation matrix of
 * the same sequence of body-axis rotations, read out with the angle conventions
 * Rotation.cpp documents (pitchX/yawY/offVert in tenths of a degree, rollZ in
 * whole degrees, unwrapped).
 */
#pragma once

#include <unity.h>

#include <math.h>
#include <stdio.h>

static const double kDegPerRad = 57.295779513082320876798154814105;
static const double kRadPerDeg = 0.017453292519943295769236907684886;

// Flight-loop input for a constant rate: 1000 LSB for 10000 us at 0.07 deg/s/LSB
static const float kGyroGain = 0.07f;
static const long kStep = 1000L * 10000L;
static const double kDegPerStep = 0.7;

//-----------------------------------------------------------------------------
// Reference model: exact rotation of the body frame, as a 3x3 matrix
//-----------------------------------------------------------------------------
struct Reference
{
  double m[3][3];
  double rollDeg;     // unwrapped roll, degrees
  double prevRollDeg; // last wrapped roll, degrees
};

static Reference ref;

static void refReset()
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      ref.m[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }
  ref.rollDeg = ref.prevRollDeg = 0.0;
}

// Rotate the body by angleDeg about its own axis (0 = X, 1 = Y, 2 = Z), in steps
// small enough that the roll unwrapping sees every half-turn.
static void refRotate(int axis, double angleDeg)
{
  int steps = (int)ceil(fabs(angleDeg) / 10.0);
  for (int s = 0; s < steps; s++)
  {
    double a = angleDeg / steps * kRadPerDeg;
    double c = cos(a), sn = sin(a);
    double r[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    int p = (axis + 1) % 3, q = (axis + 2) % 3;
    r[p][p] = c;
    r[p][q] = -sn;
    r[q][p] = sn;
    r[q][q] = c;
    double out[3][3];
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        out[i][j] = 0;
        for (int k = 0; k < 3; k++)
        {
          out[i][j] += ref.m[i][k] * r[k][j];
        }
      }
    }
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        ref.m[i][j] = out[i][j];
      }
    }
    double roll = atan2(ref.m[1][0], ref.m[0][0]) * kDegPerRad;
    double d = roll - ref.prevRollDeg;
    if (d > 180)
    {
      d -= 360;
    }
    else if (d < -180)
    {
      d += 360;
    }
    ref.rollDeg += d;
    ref.prevRollDeg = roll;
  }
}

static double refPitchDeg() { return atan2(ref.m[2][1], ref.m[2][2]) * kDegPerRad; }
static double refYawDeg() { return asin(-ref.m[2][0]) * kDegPerRad; }
// The angle between the body Z axis and the vertical
static double refOffVertDeg() { return acos(ref.m[2][2]) * kDegPerRad; }

//-----------------------------------------------------------------------------
// Firmware state
//-----------------------------------------------------------------------------
static void resetRotationState()
{
  // getQuatRotn() state, to the initial values in Rotation.cpp
  quat[0] = 0;
  quat[1] = 1;
  quat[2] = quat[3] = quat[4] = 0;
  prevRollZ = quatRollZ = fullRollZ = 0;
  // getDcm2dRotn() state
  rawX = rawY = rawZ = 0;
  cosZ = 1;
  sinZ = 0;
  // outputs and flags
  pitchX = yawY = offVert = 0;
  rollZ = 0;
  rotationFault = false;
  rotnOk = true;
  settings.maxAngle = 300; // 30 deg; the settings file is in degrees, stored x10
  settings.testMode = false;
  events.apogee = false;
}

void setUp()
{
  resetRotationState();
  refReset();
}
void tearDown() {}

// Rotate about one body axis for a number of 10 ms calls, on both the firmware
// and the reference.
static void quatRotate(int axis, int sign, int calls)
{
  for (int i = 0; i < calls; i++)
  {
    getQuatRotn(axis == 0 ? sign * kStep : 0, axis == 1 ? sign * kStep : 0, axis == 2 ? sign * kStep : 0, kGyroGain);
  }
  refRotate(axis, sign * calls * kDegPerStep);
}

static void dcmRotate(int axis, int sign, int calls)
{
  for (int i = 0; i < calls; i++)
  {
    getDcm2dRotn(axis == 0 ? sign * kStep : 0, axis == 1 ? sign * kStep : 0, axis == 2 ? sign * kStep : 0, kGyroGain);
  }
}

// pitchX, yawY and offVert are tenths of a degree and truncated, so allow
// 0.2 deg; rollZ is whole degrees, truncated, so allow 1 deg.
static void assertMatchesReference(const char *label)
{
  char msg[200];
  snprintf(msg, sizeof(msg), "%s: pitchX %d yawY %d offVert %d rollZ %ld; expected %.1f %.1f %.1f %.1f",
           label, pitchX, yawY, offVert, rollZ, refPitchDeg() * 10, refYawDeg() * 10, refOffVertDeg() * 10, ref.rollDeg);
  TEST_MESSAGE(msg);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(2.0f, refPitchDeg() * 10, pitchX, msg);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(2.0f, refYawDeg() * 10, yawY, msg);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(2.0f, refOffVertDeg() * 10, offVert, msg);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.0f, ref.rollDeg, rollZ, msg);
}

//-----------------------------------------------------------------------------
// getQuatRotn()
//-----------------------------------------------------------------------------
void test_quat_pitch_about_x()
{
  quatRotate(0, +1, 30); // +21 deg
  TEST_ASSERT_INT_WITHIN(2, 210, pitchX);
  TEST_ASSERT_INT_WITHIN(2, 210, offVert);
  TEST_ASSERT_INT_WITHIN(1, 0, yawY);
  assertMatchesReference("+X 21 deg");
}

void test_quat_negative_pitch_about_x()
{
  quatRotate(0, -1, 30);
  TEST_ASSERT_INT_WITHIN(2, -210, pitchX);
  TEST_ASSERT_INT_WITHIN(2, 210, offVert);
  assertMatchesReference("-X 21 deg");
}

void test_quat_yaw_about_y()
{
  quatRotate(1, +1, 30);
  TEST_ASSERT_INT_WITHIN(2, 210, yawY);
  TEST_ASSERT_INT_WITHIN(2, 210, offVert);
  TEST_ASSERT_INT_WITHIN(1, 0, pitchX);
  assertMatchesReference("+Y 21 deg");
}

void test_quat_negative_yaw_about_y()
{
  quatRotate(1, -1, 30);
  TEST_ASSERT_INT_WITHIN(2, -210, yawY);
  assertMatchesReference("-Y 21 deg");
}

void test_quat_roll_about_z()
{
  quatRotate(2, +1, 30);
  TEST_ASSERT_INT_WITHIN(1, 21, rollZ);
  TEST_ASSERT_INT_WITHIN(1, 0, offVert);
  assertMatchesReference("+Z 21 deg");
}

void test_quat_roll_unwraps_past_half_turns()
{
  quatRotate(2, +1, 1000); // +700 deg: wraps through +/-180 twice
  TEST_ASSERT_INT_WITHIN(1, 700, rollZ);
  assertMatchesReference("+Z 700 deg");
  resetRotationState();
  refReset();
  quatRotate(2, -1, 1000);
  TEST_ASSERT_INT_WITHIN(1, -700, rollZ);
  assertMatchesReference("-Z 700 deg");
}

void test_quat_roll_moves_tilt_from_pitch_to_yaw()
{
  // Tilt 20.3 deg about X, then roll 90.3 deg about the body Z: the tilt is
  // unchanged, and it now shows on the yaw axis.
  quatRotate(0, +1, 29);
  quatRotate(2, +1, 129);
  TEST_ASSERT_INT_WITHIN(2, 0, pitchX);
  TEST_ASSERT_INT_WITHIN(2, -203, yawY);
  TEST_ASSERT_INT_WITHIN(2, 203, offVert);
  TEST_ASSERT_INT_WITHIN(1, 90, rollZ);
  assertMatchesReference("X 20.3 then Z 90.3");
}

void test_quat_combined_tilt_about_x_then_y()
{
  // The angle off vertical is acos(cos 20.3 x cos 20.3) = 28.4 deg. #33: offVert was
  // atan(sqrt(tan^2 pitch + tan^2 yaw)), 27.6 deg here.
  quatRotate(0, +1, 29);
  quatRotate(1, +1, 29);
  TEST_ASSERT_INT_WITHIN(2, 284, offVert);
  assertMatchesReference("X 20.3 then Y 20.3");
}

void test_quat_stays_normalised_over_a_long_sequence()
{
  for (int i = 0; i < 20; i++)
  {
    quatRotate(i % 3, (i & 1) ? -1 : 1, 50 + 7 * i);
  }
  double norm = sqrt((double)quat[1] * quat[1] + (double)quat[2] * quat[2] + (double)quat[3] * quat[3] + (double)quat[4] * quat[4]);
  // the double path normalises in double precision (#33: it used powf, good to about 3e-8)
  const double tolerance = sizeof(quat[0]) == 8 ? 1e-12 : 1e-6;
  char msg[80];
  snprintf(msg, sizeof(msg), "norm - 1 = %.3g", norm - 1.0);
  TEST_MESSAGE(msg);
  TEST_ASSERT_TRUE_MESSAGE(fabs(norm - 1.0) <= tolerance, msg);
  assertMatchesReference("mixed 20-segment sequence");
}

void test_quat_rotation_fault_latches_past_45_deg()
{
  quatRotate(0, +1, 40); // 28 deg: inside the 30 deg cone
  TEST_ASSERT_TRUE(rotnOk);
  quatRotate(0, +1, 10); // 35 deg: outside the cone, no fault yet
  TEST_ASSERT_FALSE(rotnOk);
  TEST_ASSERT_FALSE(rotationFault);
  quatRotate(0, +1, 20); // 49 deg: past 45, faults
  TEST_ASSERT_TRUE(rotationFault);
  quatRotate(0, -1, 70); // back to vertical: the fault stays
  TEST_ASSERT_INT_WITHIN(2, 0, offVert);
  TEST_ASSERT_TRUE(rotationFault);
  TEST_ASSERT_FALSE(rotnOk);
}

//-----------------------------------------------------------------------------
// getDcm2dRotn(): accumulates tilt in the launch frame, so a roll does not move
// an existing tilt between pitch and yaw, and body rates are rotated by the roll
//-----------------------------------------------------------------------------
void test_dcm_single_axis()
{
  dcmRotate(0, +1, 30);
  TEST_ASSERT_INT_WITHIN(2, 210, pitchX);
  TEST_ASSERT_INT_WITHIN(1, 0, yawY);
  TEST_ASSERT_INT_WITHIN(2, 210, offVert);
  resetRotationState();
  dcmRotate(1, -1, 30);
  TEST_ASSERT_INT_WITHIN(2, -210, yawY);
  TEST_ASSERT_INT_WITHIN(1, 0, pitchX);
  TEST_ASSERT_INT_WITHIN(2, 210, offVert);
  resetRotationState();
  dcmRotate(2, +1, 1000);
  TEST_ASSERT_INT_WITHIN(1, 700, rollZ);
  TEST_ASSERT_INT_WITHIN(1, 0, offVert);
}

void test_dcm_roll_then_pitch_rate_lands_on_yaw()
{
  dcmRotate(2, +1, 129); // roll 90.3 deg
  dcmRotate(0, +1, 29);  // body X rate, now along the launch-frame Y
  TEST_ASSERT_INT_WITHIN(2, 0, pitchX);
  TEST_ASSERT_INT_WITHIN(2, 203, yawY);
  TEST_ASSERT_INT_WITHIN(2, 203, offVert);
  TEST_ASSERT_INT_WITHIN(1, 90, rollZ);
}

void test_dcm_combined_tilt_about_x_then_y()
{
  // 20.3 deg about X, then 20.3 deg about Y: acos(cos 20.3 x cos 20.3) = 28.4 deg off vertical
  dcmRotate(0, +1, 29);
  dcmRotate(1, +1, 29);
  TEST_ASSERT_INT_WITHIN(2, 203, pitchX);
  TEST_ASSERT_INT_WITHIN(2, 203, yawY);
  TEST_ASSERT_INT_WITHIN(2, 284, offVert);
}

void test_dcm_rotation_fault_past_45_deg()
{
  dcmRotate(0, +1, 50); // 35 deg
  TEST_ASSERT_FALSE(rotnOk);
  TEST_ASSERT_FALSE(rotationFault);
  dcmRotate(0, +1, 20); // 49 deg
  TEST_ASSERT_TRUE(rotationFault);
}

static int runRotationTests()
{
  UNITY_BEGIN();
  RUN_TEST(test_quat_pitch_about_x);
  RUN_TEST(test_quat_negative_pitch_about_x);
  RUN_TEST(test_quat_yaw_about_y);
  RUN_TEST(test_quat_negative_yaw_about_y);
  RUN_TEST(test_quat_roll_about_z);
  RUN_TEST(test_quat_roll_unwraps_past_half_turns);
  RUN_TEST(test_quat_roll_moves_tilt_from_pitch_to_yaw);
  RUN_TEST(test_quat_combined_tilt_about_x_then_y);
  RUN_TEST(test_quat_stays_normalised_over_a_long_sequence);
  RUN_TEST(test_quat_rotation_fault_latches_past_45_deg);
  RUN_TEST(test_dcm_single_axis);
  RUN_TEST(test_dcm_roll_then_pitch_rate_lands_on_yaw);
  RUN_TEST(test_dcm_combined_tilt_about_x_then_y);
  RUN_TEST(test_dcm_rotation_fault_past_45_deg);
  return UNITY_END();
}
