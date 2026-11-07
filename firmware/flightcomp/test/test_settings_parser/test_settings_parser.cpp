/**
 * @file test_settings_parser.cpp
 * @brief The SD card settings parsers in SD.cpp, on the sample files and on files that break them.
 *
 * @details
 * SD.cpp is compiled as it is, against the in-memory card in test/shim/SD.h. The sample
 * EEPROMsettings.txt and Settings.txt are read from src/, so a change to either sample that moves
 * a value shows here.
 *
 * Both files are read by position: parseNextVariable() skips to the next '=', skips the one
 * character after it, and takes the rest of the line. The key names are never compared.
 */
#include <unity.h>

#include <stdio.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <Arduino.h>

// The Teensy core's integer and float formatters, which SD.cpp's writers call
static char *itoa(int value, char *out, int) { return snprintf(out, 16, "%d", value), out; }
static char *ltoa(long value, char *out, int) { return snprintf(out, 24, "%ld", value), out; }
static char *ultoa(unsigned long value, char *out, int) { return snprintf(out, 24, "%lu", value), out; }
static char *dtostrf(double value, signed char width, unsigned char prec, char *out)
{
  return snprintf(out, 64, "%*.*f", width, prec, value), out;
}

// The radio timers that board.h declares
struct IntervalTimer
{
};
IntervalTimer radioTimer;
IntervalTimer syncTimer;

#include "flightcomp_subset.h"

#include "../src/SD.cpp"

//-----------------------------------------
// Helpers
//-----------------------------------------
static ShimEEPROM &shimEeprom = EEPROM;

/** The text of a file in src/, found from this file's path. */
static std::string readSource(const char *name)
{
  std::string path = __FILE__;
  path = path.substr(0, path.rfind('/')) + "/../../src/" + name;
  std::ifstream in(path, std::ios::binary);
  std::ostringstream text;
  text << in.rdbuf();
  char message[512];
  snprintf(message, sizeof(message), "cannot read %s", path.c_str());
  TEST_ASSERT_TRUE_MESSAGE(in.good() || in.eof(), message);
  TEST_ASSERT_TRUE_MESSAGE(!text.str().empty(), message);
  return text.str();
}

static void resetState()
{
  SD.files.clear();
  shimEeprom.shimErase();
  settings = UserSettings();
  unitConvert = 3.2808F;
  magCalibrateMode = 0;
  memset(dataString, 0, sizeof(dataString));
}

/** Every EEPROM byte that parseEepromSettingsSd() writes for the sample file. */
struct ExpectedByte
{
  int address;
  uint8_t value;
};

static std::vector<ExpectedByte> sampleEepromImage()
{
  const uint8_t minus = (uint8_t)(int8_t)-1;
  std::vector<ExpectedByte> e = {
      // device IDs
      {eeprom.accelId, 6},
      {eeprom.magId, 3},
      {eeprom.gyroId, 6},
      {eeprom.highGId, 2},
      {eeprom.baroId, 5},
      {eeprom.radioId, 1},
      {eeprom.gpsId, 2},
      {eeprom.sdId, 1},
      // buses: the letter, then the digit after it less '0'
      {eeprom.accelBusType, 'I'},
      {eeprom.accelBusNum, 0},
      {eeprom.magBusType, 'I'},
      {eeprom.magBusNum, 0},
      {eeprom.gyroBusType, 'I'},
      {eeprom.gyroBusNum, 0},
      {eeprom.highGBusType, 'S'},
      {eeprom.highGBusNum, 0},
      {eeprom.baroBusType, 'I'},
      {eeprom.baroBusNum, 0},
      {eeprom.radioBusType, 'S'},
      {eeprom.radioBusNum, 0},
      {eeprom.gpsBusType, 'U'},
      {eeprom.gpsBusNum, 1},
      // "SD = IO" is the letter O, so the bus number is 'O' - '0'
      {eeprom.sdBusType, 'I'},
      {eeprom.sdBusNum, 'O' - '0'},
      // orientation to the board: the axis letter, and -1 or 1 for the sign
      {eeprom.accel2BoardXAxis, 'X'},
      {eeprom.accel2BoardXSign, 1},
      {eeprom.accel2BoardYAxis, 'Y'},
      {eeprom.accel2BoardYSign, 1},
      {eeprom.accel2BoardZAxis, 'Z'},
      {eeprom.accel2BoardZSign, 1},
      {eeprom.gyro2BoardXAxis, 'X'},
      {eeprom.gyro2BoardXSign, 1},
      {eeprom.gyro2BoardYAxis, 'Y'},
      {eeprom.gyro2BoardYSign, 1},
      {eeprom.gyro2BoardZAxis, 'Z'},
      {eeprom.gyro2BoardZSign, 1},
      {eeprom.highG2BoardXAxis, 'X'},
      {eeprom.highG2BoardXSign, minus},
      {eeprom.highG2BoardYAxis, 'Y'},
      {eeprom.highG2BoardYSign, 1},
      {eeprom.highG2BoardZAxis, 'Z'},
      {eeprom.highG2BoardZSign, 1},
      {eeprom.mag2BoardXAxis, 'X'},
      {eeprom.mag2BoardXSign, 1},
      {eeprom.mag2BoardYAxis, 'Y'},
      {eeprom.mag2BoardYSign, 1},
      {eeprom.mag2BoardZAxis, 'Z'},
      {eeprom.mag2BoardZSign, 1},
      // GPIO pins
      {eeprom.pyro1ContPin, 14},
      {eeprom.pyro1FirePin, 39},
      {eeprom.pyro2ContPin, 16},
      {eeprom.pyro2FirePin, 15},
      {eeprom.pyro3ContPin, 20},
      {eeprom.pyro3FirePin, 17},
      {eeprom.pyro4ContPin, 22},
      {eeprom.pyro4FirePin, 21},
      {eeprom.nullPin, 56},
      {eeprom.beepPin, 23},
      {eeprom.battReadPin, 40},
      {eeprom.testModeGndPin, 2},
      {eeprom.testModeReadPin, 5},
      {eeprom.radioIrqPin, 28},
      {eeprom.radioRstPin, 34},
      {eeprom.radioEnPin, 56},
      {eeprom.radioCsPin, 9},
      {eeprom.accelCsPin, 56},
      {eeprom.gyroCsPin, 56},
      {eeprom.highGCsPin, 10},
      {eeprom.magCsPin, 56},
      {eeprom.baroCsPin, 56},
      {eeprom.sdCsPin, 56},
      // servo pins
      {eeprom.servo1Pin, 3},
      {eeprom.servo2Pin, 4},
      {eeprom.servo3Pin, 7},
      {eeprom.servo4Pin, 6},
      {eeprom.servo5Pin, 38},
      {eeprom.servo6Pin, 37},
      {eeprom.servo7Pin, 36},
      {eeprom.servo8Pin, 35},
      // hardware ID, one character per byte
      {eeprom.hwVersion, '4'},
      {eeprom.hwSubVersion, '0'},
      {eeprom.hwUnitNum, '1'},
  };
  const char call[] = "KK4ELF";
  for (int i = 0; i < 6; i++)
  {
    e.push_back({eeprom.callSign + i, (uint8_t)call[i]});
  }
  return e;
}

//-----------------------------------------
// Tests
//-----------------------------------------
/** The sample EEPROMsettings.txt writes exactly the bytes above, and nothing else. */
void test_sample_eeprom_settings()
{
  resetState();
  SD.files["EEPROMsettings.txt"] = readSource("EEPROMsettings.txt");
  magCalibrateMode = 9;
  parseEepromSettingsSd();

  uint8_t expected[ShimEEPROM::size];
  memset(expected, 0xFF, sizeof(expected));
  const std::vector<ExpectedByte> image = sampleEepromImage();
  for (const ExpectedByte &b : image)
  {
    expected[b.address] = b.value;
  }
  for (int a = 0; a < ShimEEPROM::size; a++)
  {
    char message[96];
    snprintf(message, sizeof(message), "EEPROM address %d", a);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(expected[a], shimEeprom.bytes[a], message);
  }
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, magCalibrateMode, "calibrateMode");
  TEST_ASSERT_FALSE_MESSAGE(SD.exists("EEPROMsettings.txt"), "the file is removed after use");
  printf("  EEPROMsettings.txt: %zu EEPROM bytes written, the rest untouched\n", image.size());
}

/** The sample Settings.txt gives the settings it lists, in the units readFlightSettingsSd() keeps. */
void test_sample_settings()
{
  resetState();
  SD.files["Settings.txt"] = readSource("Settings.txt");
  readFlightSettingsSd();

  TEST_ASSERT_EQUAL_STRING("Mach Shock", settings.rocketName);
  TEST_ASSERT_EQUAL_CHAR('S', settings.fltProfile);
  TEST_ASSERT_EQUAL_CHAR('S', settings.units);
  TEST_ASSERT_EQUAL_CHAR('P', settings.reportStyle);
  TEST_ASSERT_EQUAL_UINT32(10000UL, settings.setupTime); // ms
  TEST_ASSERT_EQUAL_FLOAT(700.0F / 3.2808F, settings.mainDeployAlt); // feet in, metres kept
  TEST_ASSERT_EQUAL_UINT32(0UL, settings.apogeeDelay);              // us
  TEST_ASSERT_EQUAL_UINT32(900000000UL, settings.rcdTime);          // us
  TEST_ASSERT_TRUE(settings.silentMode);
  TEST_ASSERT_FALSE(settings.magSwitchEnable);
  TEST_ASSERT_EQUAL_UINT8(1, settings.inflightRecover);
  TEST_ASSERT_FALSE(settings.gpsLog);
  TEST_ASSERT_EQUAL_UINT32(500000UL, settings.fireTime); // us
  TEST_ASSERT_EQUAL_CHAR('M', settings.pyro4Func);
  TEST_ASSERT_EQUAL_CHAR('A', settings.pyro3Func);
  TEST_ASSERT_EQUAL_CHAR('N', settings.pyro2Func);
  TEST_ASSERT_EQUAL_CHAR('N', settings.pyro1Func);
  TEST_ASSERT_TRUE(settings.txEnable);
  TEST_ASSERT_EQUAL_UINT8(13, settings.txPwr);
  TEST_ASSERT_EQUAL_FLOAT(433.250F, settings.txFreq);
  TEST_ASSERT_FALSE(settings.fhss);
  TEST_ASSERT_EQUAL_UINT32(500000UL, settings.boosterSeparationDelay);
  TEST_ASSERT_EQUAL_UINT32(500000UL, settings.sustainerFireDelay);
  TEST_ASSERT_EQUAL_CHAR('I', settings.airStart1Event);
  TEST_ASSERT_EQUAL_UINT32(500000UL, settings.airStart1Delay);
  TEST_ASSERT_EQUAL_CHAR('B', settings.airStart2Event);
  TEST_ASSERT_EQUAL_UINT32(0UL, settings.airStart2Delay);
  TEST_ASSERT_EQUAL_INT((int)(400 / 3.2808F), settings.altThreshold); // feet in, metres kept
  TEST_ASSERT_EQUAL_INT(200, settings.maxAngle);                      // tenths of a degree
  TEST_ASSERT_FALSE(settings.stableRotn);
  TEST_ASSERT_FALSE(settings.stableVert);
  TEST_ASSERT_FALSE(settings.flyBack);
  TEST_ASSERT_EQUAL_UINT8(1, settings.serialDebug);
}

/** The sample text up to and including the line that starts with `lastLine`, with or without its line end. */
static std::string truncatedAfter(const std::string &text, const std::string &lastLine, bool keepLineEnd)
{
  const size_t start = text.find("\n" + lastLine);
  TEST_ASSERT_TRUE_MESSAGE(start != std::string::npos, lastLine.c_str());
  const size_t end = text.find('\n', start + 1);
  return text.substr(0, keepLineEnd ? end + 1 : end);
}

/** Runs a parser, failing the test if it reads on past the end of the file. */
static void runToEnd(void (*parser)(), const char *what)
{
  try
  {
    parser();
  }
  catch (const ShimReadPastEnd &e)
  {
    char message[160];
    snprintf(message, sizeof(message), "%s read past the end of the file %ld times without stopping", what, e.reads);
    TEST_FAIL_MESSAGE(message);
  }
}

/** A Settings.txt that ends before its last values: the parser stops, and the missing values read as 0. */
void test_settings_missing_values_stop_at_end_of_file()
{
  resetState();
  SD.files["Settings.txt"] = truncatedAfter(readSource("Settings.txt"), "TXfreq = ", true);
  settings.serialDebug = 7;
  runToEnd(readFlightSettingsSd, "readFlightSettingsSd()");

  // the values in the file are read as before
  TEST_ASSERT_EQUAL_STRING("Mach Shock", settings.rocketName);
  TEST_ASSERT_EQUAL_UINT8(13, settings.txPwr);
  TEST_ASSERT_EQUAL_FLOAT(433.250F, settings.txFreq);
  // the missing ones are 0
  TEST_ASSERT_FALSE(settings.fhss);
  TEST_ASSERT_EQUAL_UINT32(0UL, settings.boosterSeparationDelay);
  TEST_ASSERT_EQUAL_CHAR('\0', settings.airStart1Event);
  TEST_ASSERT_EQUAL_INT(0, settings.maxAngle);
  TEST_ASSERT_EQUAL_UINT8(0, settings.serialDebug);
}

/** A Settings.txt whose last value has no line end after it is read to its end. */
void test_settings_last_value_without_line_end()
{
  resetState();
  SD.files["Settings.txt"] = truncatedAfter(readSource("Settings.txt"), "serialDebug = ", false);
  settings.serialDebug = 7;
  runToEnd(readFlightSettingsSd, "readFlightSettingsSd()");
  TEST_ASSERT_EQUAL_UINT8(1, settings.serialDebug);
  TEST_ASSERT_FALSE(settings.flyBack);
}

/** An EEPROMsettings.txt that ends before its last values: the parser stops and removes the file. */
void test_eeprom_settings_missing_values_stop_at_end_of_file()
{
  resetState();
  SD.files["EEPROMsettings.txt"] = truncatedAfter(readSource("EEPROMsettings.txt"), "sdCS = ", true);
  runToEnd(parseEepromSettingsSd, "parseEepromSettingsSd()");
  TEST_ASSERT_EQUAL_UINT8(56, shimEeprom.bytes[eeprom.sdCsPin]);
  TEST_ASSERT_EQUAL_UINT8(0, shimEeprom.bytes[eeprom.servo1Pin]);
  TEST_ASSERT_EQUAL_UINT8(0, shimEeprom.bytes[eeprom.servo8Pin]);
  TEST_ASSERT_FALSE_MESSAGE(SD.exists("EEPROMsettings.txt"), "the file is removed after use");
}

void setUp() {}
void tearDown() {}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_sample_eeprom_settings);
  RUN_TEST(test_sample_settings);
  RUN_TEST(test_settings_missing_values_stop_at_end_of_file);
  RUN_TEST(test_settings_last_value_without_line_end);
  RUN_TEST(test_eeprom_settings_missing_values_stop_at_end_of_file);
  return UNITY_END();
}
