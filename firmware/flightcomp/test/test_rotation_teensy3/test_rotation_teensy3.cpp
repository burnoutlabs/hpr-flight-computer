/**
 * @file test_rotation_teensy3.cpp
 * @brief Rotation.cpp on the Teensy 3.x path: getQuatRotn() in single precision.
 *
 * Built with -fsingle-precision-constant, as the Teensy 3.x boards are (see
 * test/suite_flags.py), so floating-point literals in the firmware are floats here too.
 */
#if defined(__IMXRT1062__)
#error "the Teensy 3.x suite must not define __IMXRT1062__"
#endif

#include <unity.h>

#include "flightcomp_subset.h"

#include "rotation_cases.h"

// Guard against testing the other path by mistake
static_assert(sizeof(quat[0]) == 4, "expected the float getQuatRotn() path");

int main() { return runRotationTests(); }
