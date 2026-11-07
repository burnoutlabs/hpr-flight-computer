/**
 * @file test_rotation_teensy4.cpp
 * @brief Rotation.cpp on the Teensy 4.x path: getQuatRotn() in double precision.
 */
#define __IMXRT1062__

#include <unity.h>

#include "flightcomp_subset.h"

#include "rotation_cases.h"

// Guard against testing the other path by mistake
static_assert(sizeof(quat[0]) == 8, "expected the double getQuatRotn() path");

int main() { return runRotationTests(); }
