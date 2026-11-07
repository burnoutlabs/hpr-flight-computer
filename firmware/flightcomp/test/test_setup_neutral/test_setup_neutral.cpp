/**
 * @file test_setup_neutral.cpp
 * @brief setup() against the code it was refactored from, side by side.
 *
 * @details
 * The reference is setup() at commit edc2ebf, compiled here from verbatim copies in
 * test/legacy_setup/ inside namespace legacy (the files cannot carry a comment, so their origin
 * is recorded here):
 * - setup.inc: src/flightcomp.cpp lines 172-1603, setup();
 * - check_pyro_continuity.inc: src/flightcomp.cpp lines 2197-2447, checkPyroContinuity();
 * - parse_beep.inc: src/flightcomp.cpp lines 2179-2195, parseBeep();
 * - set_poly_morphs.inc: src/Device_Mgmt.cpp lines 49-311, setPolyMorphs().
 * Each can be checked against the commit, for example:
 *   git show edc2ebf:firmware/flightcomp/src/flightcomp.cpp | sed -n 172,1603p \
 *     | diff - firmware/flightcomp/test/legacy_setup/setup.inc
 * Inside the namespace, the reference's calls to checkPyroContinuity(), parseBeep() and
 * setPolyMorphs() reach these copies; everything else it calls is shared with the code under
 * test (startup_harness.h).
 *
 * Each scenario sets the EEPROM, the settings file and the inputs, runs the reference setup(),
 * saves every global and the I/O log, restores the starting state, runs setup() from
 * Startup.cpp, and requires the same bytes in every global (flight_state.h, plus the EEPROM,
 * ports, servos, clock and input positions) and the same I/O log, call for call.
 * The scenarios between them take every branch of setup() that returns; the ones that never do
 * (uninitialized EEPROM, a failed sensor) are not run. Nor is a board with no high-G
 * accelerometer: setup() then divides by a zero sample count, which the Teensy's divide
 * instruction answers with 0 and the host's traps. setPolyMorphs() is compared on its own for
 * every device ID, 0 included.
 */
#include <unity.h>

#include "startup_harness.h"

#include "../src/InflightRecover.cpp"

// The reference's rapidReset() was unreachable from its setup() (#28): any call is logged, and
// the test fails on it. The scenarios here leave the in-flight recovery off or have no flight to
// resume, so the new setup() starts normally too; test_inflight_recovery covers the recovery.
bool rapidReset()
{
  logIo("rapidReset");
  return false;
}

//-----------------------------------------
// The reference: setup() at edc2ebf, verbatim
//-----------------------------------------
namespace legacy
{
// The reference's names for the pyro channels, servos and servo trims, which are elements of
// pyros, servos and servoTrim now (the trims as legacy_names.h spells them)
static PyroMap &pyro1 = pyros[0];
static PyroMap &pyro2 = pyros[1];
static PyroMap &pyro3 = pyros[2];
static PyroMap &pyro4 = pyros[3];
static PWMServo &canardYaw1 = servos[0];
static PWMServo &canardYaw2 = servos[1];
static PWMServo &canardPitch3 = servos[2];
static PWMServo &canardPitch4 = servos[3];
static int8_t &servo1Trim = servoTrim[0];
static int8_t &servo2Trim = servoTrim[1];
static int8_t &servo3Trim = servoTrim[2];
static int8_t &servo4Trim = servoTrim[3];
// The reference's name for the telemetry values, radioTelemetry now (it also uses sensors.radio,
// so a macro in legacy_names.h cannot map it)
static RadioTelemetry &radio = radioTelemetry;

#include "../legacy_names.h"

#include "../legacy_setup/parse_beep.inc"

#include "../legacy_setup/check_pyro_continuity.inc"

#include "../legacy_setup/set_poly_morphs.inc"

#include "../legacy_setup/setup.inc"

#include "../legacy_names_end.h"
} // namespace legacy

//-----------------------------------------
// Scenarios
//-----------------------------------------
struct SetupScenario
{
  const char *name;
  // EEPROMsettings.txt as stored in EEPROM
  uint8_t accelId = 6, gyroId = 6, magId = 3, highGId = 2, baroId = 5, radioId = 1, gnssID = 2;
  uint8_t gnssBusNum = 1;
  uint8_t pyroCont[4] = {14, 16, 20, 22};
  uint8_t pyroFire[4] = {39, 15, 17, 21};
  uint8_t battPin = 40;
  uint8_t radioEn = 56;
  bool calibrated = true;   // bias and baro offsets written (else erased, 0xFF)
  bool initialUse = false;  // maxFltAlt digits erased, as on a new part
  uint8_t lastEvent = 27;
  uint8_t inflightRecover = 1;
  // Settings.txt
  UserSettings file;
  // inputs
  std::vector<std::pair<int, uint8_t>> testButton = {{1, HIGH}};
  bool continuity[4] = {true, true, true, true};
  int battery = 2600;
  std::string serialInput;
  int magDetectAfter = 1000000;
  // what the scenario is for, checked on the reference's run
  bool (*reached)() = nullptr;
};

static void writeEepromWord(int address, int16_t value)
{
  shimEeprom.bytes[address] = (uint8_t)(value & 0xFF);
  shimEeprom.bytes[address + 1] = (uint8_t)((value >> 8) & 0xFF);
}

static void writeEepromFloat(int address, float value)
{
  memcpy(&shimEeprom.bytes[address], &value, 4);
}

/** The EEPROM a board with the scenario's EEPROMsettings.txt and history would hold. */
static void loadEeprom(const SetupScenario &s)
{
  shimEeprom.shimErase();
  uint8_t *e = shimEeprom.bytes;
  e[eeprom.accelId] = s.accelId;
  e[eeprom.gyroId] = s.gyroId;
  e[eeprom.magId] = s.magId;
  e[eeprom.highGId] = s.highGId;
  e[eeprom.baroId] = s.baroId;
  e[eeprom.radioId] = s.radioId;
  e[eeprom.gpsId] = s.gnssID;
  e[eeprom.accelBusType] = 'I';
  e[eeprom.gyroBusType] = 'I';
  e[eeprom.magBusType] = 'I';
  e[eeprom.highGBusType] = 'S';
  e[eeprom.baroBusType] = 'I';
  e[eeprom.sdBusType] = 'I';
  e[eeprom.gpsBusType] = 'U';
  e[eeprom.radioBusType] = 'S';
  e[eeprom.accelBusNum] = 0;
  e[eeprom.gyroBusNum] = 1;
  e[eeprom.magBusNum] = 2;
  e[eeprom.highGBusNum] = 0;
  e[eeprom.baroBusNum] = 3;
  e[eeprom.sdBusNum] = 4;
  e[eeprom.gpsBusNum] = s.gnssBusNum;
  e[eeprom.radioBusNum] = 1;
  e[eeprom.pyro1ContPin] = s.pyroCont[0];
  e[eeprom.pyro1FirePin] = s.pyroFire[0];
  e[eeprom.pyro2ContPin] = s.pyroCont[1];
  e[eeprom.pyro2FirePin] = s.pyroFire[1];
  e[eeprom.pyro3ContPin] = s.pyroCont[2];
  e[eeprom.pyro3FirePin] = s.pyroFire[2];
  e[eeprom.pyro4ContPin] = s.pyroCont[3];
  e[eeprom.pyro4FirePin] = s.pyroFire[3];
  e[eeprom.nullPin] = 56;
  e[eeprom.beepPin] = 23;
  e[eeprom.battReadPin] = s.battPin;
  e[eeprom.testModeGndPin] = 2;
  e[eeprom.testModeReadPin] = 5;
  e[eeprom.radioIrqPin] = 28;
  e[eeprom.radioRstPin] = 34;
  e[eeprom.radioEnPin] = s.radioEn;
  e[eeprom.radioCsPin] = 9;
  e[eeprom.accelCsPin] = 56;
  e[eeprom.gyroCsPin] = 57;
  e[eeprom.highGCsPin] = 10;
  e[eeprom.magCsPin] = 58;
  e[eeprom.baroCsPin] = 59;
  e[eeprom.sdCsPin] = 60;
  const uint8_t servoPins[8] = {3, 4, 7, 6, 38, 37, 36, 35};
  const int servoPinAddr[8] = {eeprom.servo1Pin, eeprom.servo2Pin, eeprom.servo3Pin, eeprom.servo4Pin,
                               eeprom.servo5Pin, eeprom.servo6Pin, eeprom.servo7Pin, eeprom.servo8Pin};
  const int servoTrimAddr[8] = {eeprom.servo1Trim, eeprom.servo2Trim, eeprom.servo3Trim, eeprom.servo4Trim,
                                eeprom.servo5Trim, eeprom.servo6Trim, eeprom.servo7Trim, eeprom.servo8Trim};
  for (int i = 0; i < 8; i++)
  {
    e[servoPinAddr[i]] = servoPins[i];
    e[servoTrimAddr[i]] = (uint8_t)(int8_t)(i * 3 - 7);
  }
  const char call[] = "KK4ELF";
  memcpy(&e[eeprom.callSign], call, sizeof(call));
  e[eeprom.hwVersion] = '4';
  e[eeprom.hwSubVersion] = '0';
  e[eeprom.hwUnitNum] = '1';
  if (s.calibrated)
  {
    writeEepromWord(eeprom.accelBiasX, 12);
    writeEepromWord(eeprom.accelBiasY, -8);
    writeEepromWord(eeprom.accelBiasZ, 30);
    writeEepromWord(eeprom.gyroBiasX, -3);
    writeEepromWord(eeprom.gyroBiasY, 4);
    writeEepromWord(eeprom.gyroBiasZ, -1);
    writeEepromWord(eeprom.highGBiasX, 2);
    writeEepromWord(eeprom.highGBiasY, -2);
    writeEepromWord(eeprom.highGBiasZ, 5);
    writeEepromWord(eeprom.magBiasX, 100);
    writeEepromWord(eeprom.magBiasY, -60);
    writeEepromWord(eeprom.magBiasZ, 25);
    writeEepromFloat(eeprom.baroPressOffset, 0.37F);
    writeEepromFloat(eeprom.baroTempOffset, -0.8F);
  }
  if (!s.initialUse)
  {
    const uint8_t digits[6] = {5, 3, 4, 10, 0, 0}; // 4035 (10 is a zero digit)
    for (int i = 0; i < 6; i++)
    {
      e[eeprom.maxFltAlt + i] = digits[i];
    }
  }
  e[eeprom.lastEvent] = s.lastEvent;
  e[eeprom.inflightRecover] = s.inflightRecover;
}

/** A Settings.txt like the sample one. */
static UserSettings sampleSettings()
{
  UserSettings f;
  strcpy(f.rocketName, "Hyperion III");
  f.fltProfile = 'S';
  f.units = 'S';
  f.reportStyle = 'P';
  f.setupTime = 5000UL;
  f.mainDeployAlt = 152.0F;
  f.apogeeDelay = 1000000UL;
  f.rcdTime = 900000000UL;
  f.silentMode = false;
  f.magSwitchEnable = false;
  f.inflightRecover = 1;
  f.gpsLog = false;
  f.fireTime = 500000UL;
  f.pyro4Func = 'M';
  f.pyro3Func = 'A';
  f.pyro2Func = 'N';
  f.pyro1Func = 'N';
  f.txEnable = true;
  f.txPwr = 10;
  f.txFreq = 433.250F;
  f.fhss = false;
  f.boosterSeparationDelay = 500000UL;
  f.sustainerFireDelay = 1000000UL;
  f.airStart1Event = 'B';
  f.airStart1Delay = 500000UL;
  f.airStart2Event = 'B';
  f.airStart2Delay = 500000UL;
  f.altThreshold = 120;
  f.maxAngle = 200;
  f.stableRotn = false;
  f.stableVert = false;
  f.flyBack = false;
  f.serialDebug = 0;
  return f;
}

static std::vector<uint8_t> pristine;
static int scenariosRun = 0;

/** Put the board in the scenario's starting state. */
static void startScenario(const SetupScenario &s)
{
  restoreStartupState(pristine);
  ioLog.clear();
  loadEeprom(s);
  settingsFile_ = s.file;
  boardInputs = BoardInputs();
  boardInputs.testButton = s.testButton;
  boardInputs.testReadPin = 5;
  for (int i = 0; i < 4; i++)
  {
    if (s.continuity[i])
    {
      boardInputs.highPins.push_back(s.pyroCont[i]);
    }
  }
  boardInputs.analogValue = s.battery;
  boardInputs.serialInput = s.serialInput;
  boardInputs.magDetectAfter = s.magDetectAfter;
  sensorFeed = SensorFeed();
  shimSetMicros(0);
  shimMicrosPerCall = 37;
  // what the drivers' begin functions set (the stand-ins do not)
  accel.gainX = accel.gainY = accel.gainZ = 0.000488F;
  accel.adcMax = (int16_t)(0.98 * 32768);
  accel.timeBtwnSamp = 1000;
  gyro.gainX = gyro.gainY = gyro.gainZ = 0.07F;
  gyro.timeBtwnSamp = 1000;
  highG.gainX = highG.gainY = highG.gainZ = 0.049F;
  highG.timeBtwnSamp = 1000;
  mag.timeBtwnSamp = 25000;
  baro.timeBtwnSamp = 9090;
}

static void runSideBySide(const SetupScenario &s)
{
  startScenario(s);
  legacy::setup();
  const std::vector<uint8_t> referenceState = saveStartupState();
  const std::vector<std::string> referenceLog = ioLog;
  const bool reached = s.reached == nullptr || s.reached();

  startScenario(s);
  setup();
  const std::vector<uint8_t> state = saveStartupState();
  const std::vector<std::string> &log = ioLog;

  char message[512];
  snprintf(message, sizeof(message), "%s: the scenario did not reach what it is for", s.name);
  TEST_ASSERT_TRUE_MESSAGE(reached, message);
  TEST_ASSERT_TRUE_MESSAGE(std::find(referenceLog.begin(), referenceLog.end(), "rapidReset") == referenceLog.end(), "rapidReset() was called");

  size_t firstLog = 0;
  while (firstLog < referenceLog.size() && firstLog < log.size() && referenceLog[firstLog] == log[firstLog])
  {
    firstLog++;
  }
  if (firstLog != referenceLog.size() || firstLog != log.size())
  {
    snprintf(message, sizeof(message), "%s: I/O differs at entry %zu of %zu/%zu: reference '%s', new '%s'", s.name, firstLog,
             referenceLog.size(), log.size(), firstLog < referenceLog.size() ? referenceLog[firstLog].c_str() : "(end)",
             firstLog < log.size() ? log[firstLog].c_str() : "(end)");
    TEST_FAIL_MESSAGE(message);
  }
  const std::string difference = firstStartupDifference(referenceState, state);
  snprintf(message, sizeof(message), "%s: state differs in %s", s.name, difference.c_str());
  TEST_ASSERT_TRUE_MESSAGE(difference.empty(), message);
  printf("  %-44s %6zu I/O entries, identical\n", s.name, log.size());
  scenariosRun++;
}

//-----------------------------------------
// Tests
//-----------------------------------------
void test_flight_configuration()
{
  SetupScenario s;
  s.name = "flight, single stage, all continuity";
  s.file = sampleSettings();
  s.reached = [] { return !settings.testMode && cont.main && cont.apogee && !cont.error; };
  runSideBySide(s);
}

void test_test_mode_with_servos_uncalibrated_first_use()
{
  SetupScenario s;
  s.name = "test mode, servos, uncalibrated, first use";
  s.file = sampleSettings();
  s.file.fltProfile = 'B';
  s.file.stableRotn = true;
  s.file.silentMode = true;
  s.file.txFreq = 915.0F;
  s.file.serialDebug = 3;
  s.calibrated = false;
  s.initialUse = true;
  s.testButton = {{2, LOW}, {1, HIGH}};
  s.reached = [] { return settings.testMode && !settings.calibrationMode && servos[0].pin == 3 && settings.txPwr == 2 && radioTelemetry.event == BOOSTER_PREFLIGHT; };
  runSideBySide(s);
}

void test_calibration_mode_and_serial_commands()
{
  SetupScenario s;
  s.name = "calibration mode, serial b and c";
  s.file = sampleSettings();
  s.serialInput = "bxc";
  s.testButton = {{1, LOW}, {40, HIGH}, {1, LOW}};
  s.reached = [] { return settings.calibrationMode; };
  runSideBySide(s);
}

void test_magnetic_switch_with_continuity_error()
{
  SetupScenario s;
  s.name = "magnetic switch, continuity error, Marsa";
  s.file = sampleSettings();
  s.file.magSwitchEnable = true;
  s.file.reportStyle = 'M';
  s.file.pyro1Func = 'A';
  s.continuity[0] = false;
  s.magDetectAfter = 25;
  s.reached = [] { return settings.magSwitchEnable && cont.error && cont.beepCode == 1; };
  runSideBySide(s);
}

void test_radio_off_two_stage_without_pyros()
{
  SetupScenario s;
  s.name = "radio off, two stage with no stage pyros";
  s.file = sampleSettings();
  s.file.txEnable = false;
  s.file.fltProfile = '2';
  s.file.pyro1Func = 'I';
  s.file.pyro2Func = 'B';
  s.continuity[0] = s.continuity[1] = false;
  s.radioEn = 33;
  s.file.fhss = true;
  s.file.txFreq = 430.0F;
  s.reached = [] { return settings.fltProfile == 'S' && pyros[0].func == 'N' && pyros[1].func == 'N' && !settings.fhss; };
  runSideBySide(s);
}

void test_airstart_separate_gyro()
{
  SetupScenario s;
  s.name = "airstart, separate gyro, ADXL377, M9N";
  s.file = sampleSettings();
  s.file.fltProfile = 'A';
  s.file.pyro1Func = '1';
  s.file.pyro2Func = '2';
  s.file.fhss = true;
  s.file.txFreq = 915.0F;
  s.file.units = 'M';
  s.gyroId = 1;
  s.highGId = 3;
  s.magId = 0;
  s.gnssID = 3;
  s.gnssBusNum = 4;
  s.reached = [] { return settings.fltProfile == 'A' && cont.beepCode == 4 && unitConvert == 1.0F; };
  runSideBySide(s);
}

void test_settings_clamped_high()
{
  SetupScenario s;
  s.name = "every setting above its limit";
  s.file = sampleSettings();
  s.file.apogeeDelay = 9000000UL;
  s.file.fireTime = 3000000UL;
  s.file.mainDeployAlt = 900.0F;
  s.file.sustainerFireDelay = 9000000UL;
  s.file.boosterSeparationDelay = 4000000UL;
  s.file.airStart1Delay = 3000000UL;
  s.file.airStart2Delay = 3000000UL;
  s.file.maxAngle = 600;
  s.file.setupTime = 90000UL;
  s.file.txPwr = 20;
  s.file.txFreq = 950.0F;
  s.file.fhss = true;
  s.file.fltProfile = '2';
  s.file.pyro1Func = 'I';
  s.file.pyro2Func = 'B';
  s.reached = [] { return settings.apogeeDelay == 5000000UL && settings.mainDeployAlt == 458 && settings.txFreq == 914.9F && boosterBurpTime == 1000000UL; };
  runSideBySide(s);
}

void test_settings_clamped_low_old_battery_pin()
{
  SetupScenario s;
  s.name = "every setting below its limit, old battery pin";
  s.file = sampleSettings();
  s.file.fireTime = 1000UL;
  s.file.mainDeployAlt = 10.0F;
  s.file.altThreshold = 20;
  s.file.rcdTime = 1000000UL;
  s.file.setupTime = 100UL;
  s.file.txPwr = 0;
  s.file.txFreq = 400.0F;
  s.file.txEnable = false;
  s.file.pyro2Func = 'M';
  s.file.serialDebug = 1;
  s.radioId = 0;
  s.battPin = 22;
  s.reached = [] { return settings.fireTime == 200000UL && settings.txFreq == 421.0F && pins.batt == 16; };
  runSideBySide(s);
}

void test_last_event_not_touchdown()
{
  SetupScenario s;
  s.name = "lastEvent 3 with recovery off";
  s.file = sampleSettings();
  s.lastEvent = 3;
  s.inflightRecover = 0;
  s.reached = [] { return shimEeprom.bytes[eeprom.lastEvent] == 27; };
  runSideBySide(s);
  SetupScenario underMain;
  underMain.name = "lastEvent 7 (under the main), recovery on";
  underMain.file = sampleSettings();
  underMain.lastEvent = 7;
  underMain.reached = [] { return shimEeprom.bytes[eeprom.lastEvent] == 27; };
  runSideBySide(underMain);
}

void test_other_devices()
{
  // every device type, a few at a time, through setup()
  const uint8_t accels[] = {1, 2, 3, 4, 5, 7, 8, 0, 9};
  const uint8_t gyros[] = {0, 1, 2, 7, 6, 3, 4, 5, 9};
  // never high-G ID 0 here: with no high-G accelerometer, setup() divides by a zero sample count
  // (highG.sumX0 / highGsamps), which gives 0 on the Teensy and traps on the host
  const uint8_t highGs[] = {1, 2, 3, 4, 1, 2, 3, 4, 2};
  const uint8_t mags[] = {1, 2, 3, 4, 5, 0, 6, 1, 2};
  const uint8_t baros[] = {1, 2, 3, 4, 6, 7, 8, 0, 9};
  const uint8_t radios[] = {2, 3, 0, 4, 1, 2, 3, 0, 1};
  const uint8_t gnss[] = {1, 3, 4, 0, 5, 2, 1, 4, 3};
  static char names[9][48];
  for (int i = 0; i < 9; i++)
  {
    SetupScenario s;
    snprintf(names[i], sizeof(names[i]), "devices %u/%u/%u/%u/%u/%u/%u", accels[i], gyros[i], highGs[i], mags[i], baros[i], radios[i], gnss[i]);
    s.name = names[i];
    s.file = sampleSettings();
    s.accelId = accels[i];
    s.gyroId = gyros[i];
    s.highGId = highGs[i];
    s.magId = mags[i];
    s.baroId = baros[i];
    s.radioId = radios[i];
    s.gnssID = gnss[i];
    runSideBySide(s);
  }
}

/** setPolyMorphs() alone, every ID 0-255 of every device with the others at the sample's. */
void test_set_poly_morphs_every_id()
{
  uint8_t *ids[] = {&sensors.accel, &sensors.gyro, &sensors.highG, &sensors.mag, &sensors.baro, &sensors.radio, &sensors.gnss};
  int compared = 0;
  for (uint8_t *id : ids)
  {
    for (int value = 0; value < 256; value++)
    {
      restoreStartupState(pristine);
      sensors.accel = 6;
      sensors.gyro = 6;
      sensors.highG = 2;
      sensors.mag = 3;
      sensors.baro = 5;
      sensors.radio = 1;
      sensors.gnss = 2;
      *id = (uint8_t)value;
      legacy::setPolyMorphs();
      const std::vector<uint8_t> reference = saveStartupState();
      restoreStartupState(pristine);
      sensors.accel = 6;
      sensors.gyro = 6;
      sensors.highG = 2;
      sensors.mag = 3;
      sensors.baro = 5;
      sensors.radio = 1;
      sensors.gnss = 2;
      *id = (uint8_t)value;
      setPolyMorphs();
      const std::string difference = firstStartupDifference(reference, saveStartupState());
      char message[160];
      snprintf(message, sizeof(message), "setPolyMorphs, ID %d: differs in %s", value, difference.c_str());
      TEST_ASSERT_TRUE_MESSAGE(difference.empty(), message);
      compared++;
    }
  }
  printf("  setPolyMorphs: %d ID sets, identical pointers\n", compared);
}

void setUp() {}
void tearDown() {}

int main()
{
  pristine = saveStartupState();
  UNITY_BEGIN();
  RUN_TEST(test_flight_configuration);
  RUN_TEST(test_test_mode_with_servos_uncalibrated_first_use);
  RUN_TEST(test_calibration_mode_and_serial_commands);
  RUN_TEST(test_magnetic_switch_with_continuity_error);
  RUN_TEST(test_radio_off_two_stage_without_pyros);
  RUN_TEST(test_airstart_separate_gyro);
  RUN_TEST(test_settings_clamped_high);
  RUN_TEST(test_settings_clamped_low_old_battery_pin);
  RUN_TEST(test_last_event_not_touchdown);
  RUN_TEST(test_other_devices);
  RUN_TEST(test_set_poly_morphs_every_id);
  printf("  setup scenarios compared: %d\n", scenariosRun);
  return UNITY_END();
}
