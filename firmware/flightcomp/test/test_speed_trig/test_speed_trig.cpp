/**
 * @file test_speed_trig.cpp
 * @brief Speed_Trig.cpp lookup functions against libm.
 *
 * Angles in and out of the speed* functions are in tenths of a degree. Each test
 * sweeps the function's domain, measures the worst error against libm (in double
 * precision), and asserts a bound just above what the implementation achieves.
 * The measured figure is printed with the test, so a change in precision shows.
 *
 * Measured on x86-64 (g++ 14), and the bound each test asserts:
 *   speedSin, speedCos      |err| 2.96e-8          bound 4e-8   (float table rounding)
 *   speedTan                rel 5.7e-8 vs |tan|    bound 1e-7
 *   speedArcTan             |err| 0.1000017 deg    bound 0.1001 deg
 *   speedArcSin (-1 < x < 1) |err| 0.100011 deg    bound 0.1001 deg
 *   speedAtan2              |err| 0.1000017 deg    bound 0.1001 deg
 *   sinSmallAngle (|x| <= pi)       1.15e-7        bound 1.5e-7
 *   cosSmallAngle (|x| <= 1 deg)    3.4e-8         bound 4e-8
 *   cosSmallAngle (|x| <= 20 deg)   6.2e-4         bound 6.5e-4
 *   cosSmallAngle (|x| <= pi)       4.03e-2        bound 4.1e-2
 *
 * Behaviour these tests describe rather than endorse (left unchanged here):
 * - speedTan() returns |tan|: it folds negative angles and angles past 90 deg
 *   onto 0..90. Its callers square the result, so the sign is not used.
 * - speedArcSin(-1.0) returns +900, not -900; it is excluded from the sweep.
 * - speedArcCos() searches the decreasing COS_TABLE[] table as if it were increasing,
 *   so it does not return arccos (speedArcCos(1.0) is 900, speedArcCos(0.5) is 0).
 *   It has no test here; magRotn() is its only caller.
 * - cosSmallAngle() takes its small-angle branch, 1 - x^2/2, for |x| < COS_TABLE[7]
 *   (0.99993 rad), so the Maclaurin branch below it is unreachable and the
 *   error grows to 4e-2 near 1 rad. The bounds above are what it does today.
 */
#include <unity.h>

#include <stdio.h>

#include "flightcomp_subset.h"

static const double kDegToRad = 0.017453292519943295769236907684886;
static const double kRadToDeg = 57.295779513082320876798154814105;

// Fails with the measured error in the message when it exceeds the bound, and
// reports the measured error either way.
static void assertWithin(const char *what, double measured, double bound)
{
  char msg[160];
  snprintf(msg, sizeof(msg), "%s: worst error %.6g, bound %.6g", what, measured, bound);
  TEST_MESSAGE(msg);
  TEST_ASSERT_TRUE_MESSAGE(measured <= bound, msg);
}

void setUp() { settings.testMode = false; }
void tearDown() {}

void test_speedSin_matches_sin_over_two_turns()
{
  double worst = 0;
  for (int t = -3600; t <= 3600; t++)
  {
    worst = fmax(worst, fabs(speedSin(t) - sin(t * 0.1 * kDegToRad)));
  }
  assertWithin("speedSin, -360..360 deg", worst, 4e-8);
}

void test_speedCos_matches_cos_over_two_turns()
{
  double worst = 0;
  for (int t = -3600; t <= 3600; t++)
  {
    worst = fmax(worst, fabs(speedCos(t) - cos(t * 0.1 * kDegToRad)));
  }
  assertWithin("speedCos, -360..360 deg", worst, 4e-8);
}

void test_speedTan_matches_abs_tan()
{
  // Relative error where |tan| > 1, absolute below it; 90 deg is excluded.
  double worst = 0;
  for (int t = -1799; t <= 1799; t++)
  {
    if (t == 900 || t == -900)
    {
      continue;
    }
    double expected = fabs(tan(t * 0.1 * kDegToRad));
    worst = fmax(worst, fabs(speedTan(t) - expected) / fmax(1.0, expected));
  }
  assertWithin("speedTan vs |tan|, -179.9..179.9 deg", worst, 1e-7);
  // 90 deg is clamped to the last table entry
  TEST_ASSERT_EQUAL_FLOAT(573.0f, speedTan(900));
  TEST_ASSERT_EQUAL_FLOAT(573.0f, speedTan(-900));
}

void test_speedArcTan_within_a_tenth_and_truncates_toward_zero()
{
  double worst = 0;
  for (long i = -89990; i <= 89990; i++)
  {
    double angle = i * 0.001; // degrees
    float value = tan(angle * kDegToRad);
    double err = speedArcTan(value) * 0.1 - atan((double)value) * kRadToDeg;
    worst = fmax(worst, fabs(err));
    // Below the 89.9 deg table entry the result is the table angle at or below
    // the true angle (above it, for negative values).
    if (fabs(value) <= TAN_TABLE[899])
    {
      TEST_ASSERT_TRUE_MESSAGE(value >= 0 ? err <= 1e-4 : err >= -1e-4, "speedArcTan does not truncate toward zero");
    }
  }
  assertWithin("speedArcTan, -89.99..89.99 deg", worst, 0.1001);
  TEST_ASSERT_EQUAL_INT(0, speedArcTan(0.0f));
  TEST_ASSERT_EQUAL_INT(900, speedArcTan(1e6f));
  TEST_ASSERT_EQUAL_INT(-900, speedArcTan(-1e6f));
}

void test_speedArcSin_within_a_tenth_on_open_interval()
{
  double worst = 0;
  for (long i = -999999; i <= 999999; i++)
  {
    float value = i * 1e-6;
    double err = speedArcSin(value) * 0.1 - asin((double)value) * kRadToDeg;
    worst = fmax(worst, fabs(err));
    TEST_ASSERT_TRUE_MESSAGE(value >= 0 ? err <= 1e-4 : err >= -1e-4, "speedArcSin does not truncate toward zero");
  }
  assertWithin("speedArcSin, -1 < x < 1", worst, 0.1001);
  TEST_ASSERT_EQUAL_INT(900, speedArcSin(1.0f));
  TEST_ASSERT_EQUAL_INT(0, speedArcSin(0.0f));
}

void test_speedAtan2_within_a_tenth_in_every_quadrant()
{
  double worst = 0;
  for (long i = -1799999; i <= 1800000; i++)
  {
    double angle = i * 1e-4 * kDegToRad;
    float y = 2.5 * sin(angle);
    float x = 2.5 * cos(angle);
    double err = speedAtan2(y, x) * 0.1 - atan2((double)y, (double)x) * kRadToDeg;
    // -180 and +180 deg are the same direction
    if (err > 180)
    {
      err -= 360;
    }
    else if (err < -180)
    {
      err += 360;
    }
    worst = fmax(worst, fabs(err));
  }
  assertWithin("speedAtan2, full circle", worst, 0.1001);
  TEST_ASSERT_EQUAL_INT(0, speedAtan2(0.0f, 0.0f));
  TEST_ASSERT_EQUAL_INT(900, speedAtan2(1.0f, 0.0f));
  TEST_ASSERT_EQUAL_INT(-900, speedAtan2(-1.0f, 0.0f));
  TEST_ASSERT_EQUAL_INT(1800, speedAtan2(0.0f, -1.0f));
}

static double worstSmallAngleError(float (*fn)(float), double (*ref)(double), double limit)
{
  double worst = 0;
  for (long i = -1000000; i <= 1000000; i++)
  {
    float rad = limit * i * 1e-6;
    worst = fmax(worst, fabs(fn(rad) - ref((double)rad)));
  }
  return worst;
}

void test_sinSmallAngle_matches_sin()
{
  assertWithin("sinSmallAngle, |x| <= pi", worstSmallAngleError(sinSmallAngle, sin, M_PI), 1.5e-7);
}

void test_cosSmallAngle_matches_cos()
{
  // getDcm2dRotn() passes it one 10 ms roll step: 20 deg at the 2000 deg/s limit
  assertWithin("cosSmallAngle, |x| <= 1 deg", worstSmallAngleError(cosSmallAngle, cos, 1 * kDegToRad), 4e-8);
  assertWithin("cosSmallAngle, |x| <= 20 deg", worstSmallAngleError(cosSmallAngle, cos, 20 * kDegToRad), 6.5e-4);
  assertWithin("cosSmallAngle, |x| <= pi", worstSmallAngleError(cosSmallAngle, cos, M_PI), 4.1e-2);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_speedSin_matches_sin_over_two_turns);
  RUN_TEST(test_speedCos_matches_cos_over_two_turns);
  RUN_TEST(test_speedTan_matches_abs_tan);
  RUN_TEST(test_speedArcTan_within_a_tenth_and_truncates_toward_zero);
  RUN_TEST(test_speedArcSin_within_a_tenth_on_open_interval);
  RUN_TEST(test_speedAtan2_within_a_tenth_in_every_quadrant);
  RUN_TEST(test_sinSmallAngle_matches_sin);
  RUN_TEST(test_cosSmallAngle_matches_cos);
  return UNITY_END();
}
