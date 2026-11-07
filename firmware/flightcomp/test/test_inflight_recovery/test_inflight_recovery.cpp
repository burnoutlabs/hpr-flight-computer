/**
 * @file test_inflight_recovery.cpp
 * @brief A reset in flight, then setup() and the flight step from where the flight was (#28).
 *
 * @details
 * Each scenario flies a synthetic flight through the real flight step (flight_harness.h) with
 * in-flight recovery on, up to a chosen moment. The EEPROM writes the flight step asked for are
 * made in the shim EEPROM, which already holds the board's hardware settings. Then the board
 * resets: every global goes back to its start-up value except the EEPROM, and the real setup()
 * (Startup.cpp) runs, with the sensors reading the same flight (startup_harness.h). The flight
 * then continues through the flight step to the end of the profile.
 *
 * Pyro 1 is the apogee charge (pin 21), pyro 2 the main (pin 22). The flight from
 * synthetic_flights.h: liftoff at 5 s, a 3 s burn at 100 m/s^2, the apogee charge (1 s delay) at
 * 28.5 s without a reset, a drogue (about 40 m/s), the main at 200 m above the pad.
 */
#include <unity.h>

#include "startup_harness.h"

#include "synthetic_flights.h"

#include "../src/InflightRecover.cpp"

static std::vector<uint8_t> pristine;
static const SyntheticFlight *truthSource = nullptr;
static SensorModel truthModel;

// What the sensors read during setup(): the flight at the current clock
static void accelFromTruth()
{
  const Truth t = truthSource->at(shimMicrosNow);
  accel.rawX = 0;
  accel.rawY = 0;
  accel.rawZ = truthModel.accelCounts(t.specificForce);
  gyro.rawX = gyro.rawY = gyro.rawZ = 0;
}
static void baroFromTruth()
{
  const Truth t = truthSource->at(shimMicrosNow);
  baro.pressure = t.pressurePa * 0.01F;
  baro.rawAlt = baroAltitude(t.pressurePa);
  baro.newSamp = true;
}

static ProfileSpec profile()
{
  ProfileSpec spec;
  spec.burns = {{5.0F, 3.0F, 100.0F}};
  spec.duration = 135.0F;
  return spec;
}

/** The EEPROM of a board set up as the sample EEPROMsettings.txt, pyros on pins 21 to 24. */
static void loadBoardEeprom(uint8_t inflightRecover)
{
  shimEeprom.shimErase();
  uint8_t *e = shimEeprom.bytes;
  e[eeprom.accelId] = 6;
  e[eeprom.gyroId] = 6;
  e[eeprom.magId] = 3;
  e[eeprom.highGId] = 2;
  e[eeprom.baroId] = 5;
  e[eeprom.radioId] = 1;
  e[eeprom.gpsId] = 2;
  e[eeprom.accelBusType] = 'I';
  e[eeprom.accelBusNum] = 0;
  e[eeprom.gyroBusType] = 'I';
  e[eeprom.gyroBusNum] = 0;
  e[eeprom.magBusType] = 'I';
  e[eeprom.magBusNum] = 0;
  e[eeprom.highGBusType] = 'S';
  e[eeprom.highGBusNum] = 0;
  e[eeprom.baroBusType] = 'I';
  e[eeprom.baroBusNum] = 0;
  e[eeprom.gpsBusType] = 'U';
  e[eeprom.gpsBusNum] = 1;
  e[eeprom.radioBusType] = 'S';
  e[eeprom.radioBusNum] = 0;
  const uint8_t cont[4] = {14, 16, 20, 22};
  const int contAddr[4] = {eeprom.pyro1ContPin, eeprom.pyro2ContPin, eeprom.pyro3ContPin, eeprom.pyro4ContPin};
  const int fireAddr[4] = {eeprom.pyro1FirePin, eeprom.pyro2FirePin, eeprom.pyro3FirePin, eeprom.pyro4FirePin};
  for (int i = 0; i < 4; i++)
  {
    e[contAddr[i]] = cont[i];
    e[fireAddr[i]] = (uint8_t)(21 + i);
  }
  e[eeprom.nullPin] = 56;
  e[eeprom.beepPin] = 23;
  e[eeprom.battReadPin] = 40;
  e[eeprom.testModeGndPin] = 2;
  e[eeprom.testModeReadPin] = 5;
  e[eeprom.radioCsPin] = 9;
  e[eeprom.radioEnPin] = 56;
  e[eeprom.highGCsPin] = 10;
  const int biasAddr[] = {eeprom.accelBiasX, eeprom.accelBiasY, eeprom.accelBiasZ, eeprom.gyroBiasX, eeprom.gyroBiasY, eeprom.gyroBiasZ,
                          eeprom.highGBiasX, eeprom.highGBiasY, eeprom.highGBiasZ, eeprom.magBiasX, eeprom.magBiasY, eeprom.magBiasZ};
  for (int address : biasAddr)
  {
    e[address] = 0;
    e[address + 1] = 0;
  }
  memset(&e[eeprom.baroPressOffset], 0, 4);
  memset(&e[eeprom.baroTempOffset], 0, 4);
  for (int i = 0; i < 6; i++)
  {
    e[eeprom.maxFltAlt + i] = 0;
  }
  e[eeprom.lastEvent] = 27;
  e[eeprom.inflightRecover] = inflightRecover;
}

/** Settings.txt: single stage, apogee charge on pyro 1, main on pyro 2 at 200 m. */
static UserSettings flightSettingsFile(uint8_t inflightRecover, uint32_t apogeeDelay)
{
  UserSettings f;
  strcpy(f.rocketName, "Recovery");
  f.fltProfile = 'S';
  f.units = 'M';
  f.reportStyle = 'M';
  f.setupTime = 3000UL;
  f.mainDeployAlt = 200.0F;
  f.apogeeDelay = apogeeDelay;
  f.rcdTime = 900000000UL;
  f.silentMode = false;
  f.magSwitchEnable = false;
  f.inflightRecover = inflightRecover;
  f.gpsLog = false;
  f.fireTime = 500000UL;
  f.pyro1Func = 'A';
  f.pyro2Func = 'M';
  f.pyro3Func = 'N';
  f.pyro4Func = 'N';
  f.txEnable = true;
  f.txPwr = 10;
  f.txFreq = 433.25F;
  f.fhss = false;
  f.altThreshold = 91;
  f.maxAngle = 300;
  f.serialDebug = 0;
  return f;
}

/** What one run saw after the reset. */
struct RecoveryRun
{
  bool recovered = false;          // setup() resumed the flight
  EventList eventsAtResume;        // events right after setup()
  uint8_t accelBusNum = 255;       // as setup() left it
  bool beginAccelSet = false;      // beginAccel pointed at the LSM6DSOX driver
  bool telemetryBegun = false;     // beginTelemetry() was called
  uint8_t radioEvent = 0;          // radio.event after setup()
  uint8_t lastEventAfterSetup = 0; // eeprom.lastEvent after setup()
  float resumeAt = -1.0F;          // s, when setup() returned
  float apogeeFireAt = -1.0F;      // s, pin 21 high after the reset (-1: never)
  float mainFireAt = -1.0F;        // s, pin 22 high after the reset (-1: never)
  float mainFireHeight = -1.0F;    // m above the pad, true height when the main fired
  EventList finalEvents;
};

static float trueHeight(const SyntheticFlight &flight, uint32_t now)
{
  const float pad = 1400.0F;
  const float pressure = flight.at(now).pressurePa;
  // invert standardPressure()
  return (1.0F - powf(pressure / 101325.0F, 1.0F / 5.25588F)) / 2.25577e-5F - pad;
}

static void applyEepromWrites(const std::vector<std::string> &log)
{
  for (const std::string &entry : log)
  {
    int address, value;
    if (sscanf(entry.c_str(), "eeprom %d %d", &address, &value) == 2)
    {
      shimEeprom.bytes[address] = (uint8_t)value;
    }
  }
}

/** The board's flight-step configuration, as setup() leaves it, for the flight before the reset. */
static void configureFlight(const UserSettings &file)
{
  configureSensors();
  configurePyros(file.pyro1Func, file.pyro2Func, file.pyro3Func, file.pyro4Func);
  settings.mainDeployAlt = file.mainDeployAlt;
  settings.apogeeDelay = file.apogeeDelay;
  settings.altThreshold = file.altThreshold;
  settings.maxAngle = file.maxAngle;
  settings.inflightRecover = file.inflightRecover;
  settings.fireTime = file.fireTime;
}

/**
 * @brief Fly until resetWhen() holds, reset, run setup(), and fly on to the end of the profile.
 * resetWhen null: no reset, fly the whole profile (the reference run).
 */
static RecoveryRun flyWithReset(uint8_t inflightRecover, uint32_t apogeeDelay, bool (*resetWhen)())
{
  RecoveryRun run;
  const SyntheticFlight flight(profile());
  truthSource = &flight;
  const UserSettings file = flightSettingsFile(inflightRecover, apogeeDelay);

  restoreStartupState(pristine);
  loadBoardEeprom(inflightRecover);
  shimMicrosPerCall = 0;
  configureFlight(file);
  ioLog.clear();

  LoopState loopState;
  uint32_t jitter = 1U;
  uint32_t now = 0;
  bool reset = false;
  while (now < flight.durationMicros())
  {
    jitter = jitter * 1103515245U + 12345U;
    now += 300 + (jitter >> 16) % 401;
    const size_t logStart = ioLog.size();
    runPass(loopState, now, flight.at(now));
    for (size_t i = logStart; i < ioLog.size(); i++)
    {
      if (ioLog[i] == "pin 21 1" && run.apogeeFireAt < 0.0F && (reset || resetWhen == nullptr))
      {
        run.apogeeFireAt = now * 1e-6F;
      }
      if (ioLog[i] == "pin 22 1" && run.mainFireAt < 0.0F && (reset || resetWhen == nullptr))
      {
        run.mainFireAt = now * 1e-6F;
        run.mainFireHeight = trueHeight(flight, now);
      }
    }
    if (!reset && resetWhen != nullptr && resetWhen())
    {
      // the reset: the EEPROM keeps what the flight step wrote, everything else starts over
      reset = true;
      applyEepromWrites(ioLog);
      const std::vector<uint8_t> keptEeprom(shimEeprom.bytes, shimEeprom.bytes + ShimEEPROM::size);
      restoreStartupState(pristine);
      memcpy(shimEeprom.bytes, keptEeprom.data(), keptEeprom.size());
      ioLog.clear();
      settingsFile_ = file;
      boardInputs = BoardInputs();
      boardInputs.testReadPin = 5;
      boardInputs.testButton = {{1, HIGH}};
      boardInputs.highPins = {14, 16};
      boardInputs.analogValue = 2600;
      sensorFeed = SensorFeed();
      accelFeedHook = accelFromTruth;
      baroFeedHook = baroFromTruth;
      // what the drivers' begin functions set (the stand-ins do not)
      accel.gainX = accel.gainY = accel.gainZ = ACCEL_G_PER_COUNT;
      accel.adcMax = (int16_t)(0.98 * 32768);
      accel.timeBtwnSamp = ACCEL_SAMPLE_US;
      gyro.gainX = gyro.gainY = gyro.gainZ = 0.07F;
      gyro.timeBtwnSamp = GYRO_SAMPLE_US;
      highG.gainX = highG.gainY = highG.gainZ = HIGHG_G_PER_COUNT;
      highG.timeBtwnSamp = HIGHG_SAMPLE_US;
      mag.timeBtwnSamp = 25000;
      baro.timeBtwnSamp = BARO_SAMPLE_US;
      SensorData *all[] = {&accel, &gyro, &mag, &highG};
      for (SensorData *s : all)
      {
        s->dirX = s->dirY = s->dirZ = 1;
      }
      shimSetMicros(now);
      shimMicrosPerCall = 37;

      setup();

      shimMicrosPerCall = 0;
      accelFeedHook = nullptr;
      baroFeedHook = nullptr;
      now = shimMicrosNow;
      run.recovered = std::find(ioLog.begin(), ioLog.end(), "createNextFileSD") == ioLog.end();
      run.eventsAtResume = events;
      run.accelBusNum = sensors.accelBusNum;
      run.beginAccelSet = beginAccel == beginLSM6DSOX;
      run.telemetryBegun = std::find(ioLog.begin(), ioLog.end(), "beginTelemetry") != ioLog.end();
      run.radioEvent = radio.event;
      run.lastEventAfterSetup = shimEeprom.bytes[eeprom.lastEvent];
      run.resumeAt = now * 1e-6F;
      if (!run.recovered)
      {
        break;
      }
      // loop() samples from where setup() left the sensors
      loopState = LoopState();
    }
  }
  run.finalEvents = events;
  printf("  recovered %d, resumed at %.3f s, apogee charge %.3f s, main %.3f s at %.1f m\n", run.recovered, run.resumeAt, run.apogeeFireAt, run.mainFireAt,
         run.mainFireHeight);
  return run;
}

static bool afterApogeeCharge() { return events.apogeeFire && fltTime.timeCurrent - fltTime.apogeeFire > 5000000UL; }
static bool afterApogeeBeforeCharge() { return events.apogee && fltTime.timeCurrent - fltTime.apogee > 1000000UL; }
static bool inCoast() { return events.boosterBurnout && fltTime.timeCurrent - fltTime.boosterBurnout > 2000000UL; }

//-----------------------------------------
// Tests
//-----------------------------------------

/** Under the drogue: setup() resumes, the apogee charge does not fire again, the main does. */
void test_reset_under_drogue_fires_only_the_main()
{
  const RecoveryRun run = flyWithReset(2, 1000000UL, afterApogeeCharge);
  TEST_ASSERT_TRUE_MESSAGE(run.recovered, "setup() did not resume the flight");
  TEST_ASSERT_TRUE(run.eventsAtResume.liftoff && run.eventsAtResume.apogee && run.eventsAtResume.apogeeFire);
  TEST_ASSERT_FALSE(run.eventsAtResume.mainDeploy);
  TEST_ASSERT_TRUE_MESSAGE(run.apogeeFireAt < 0.0F, "the apogee charge fired again");
  TEST_ASSERT_TRUE_MESSAGE(run.mainFireAt > 0.0F, "the main did not fire");
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(25.0F, 175.0F, run.mainFireHeight, "the main did not fire just below 200 m");
  TEST_ASSERT_TRUE(run.finalEvents.touchdown);
  TEST_ASSERT_EQUAL_UINT8(POWER_LOSS_RESTART, run.radioEvent);
}

/** The hardware comes up as in setup(): drivers, telemetry, and the accelerometer bus number. */
void test_reset_brings_the_hardware_up_as_setup_does()
{
  const RecoveryRun run = flyWithReset(2, 1000000UL, afterApogeeCharge);
  TEST_ASSERT_TRUE(run.recovered);
  TEST_ASSERT_TRUE_MESSAGE(run.beginAccelSet, "the device functions were not set (setPolyMorphs)");
  TEST_ASSERT_TRUE_MESSAGE(run.telemetryBegun, "telemetry was not started (beginTelemetry)");
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, run.accelBusNum, "accelerometer bus number not read from eeprom.accelBusNum");
  // the record of the flight stays, for a second reset
  TEST_ASSERT_EQUAL_UINT8(FIRE_APOGEE, run.lastEventAfterSetup);
}

/** After apogee, before the apogee charge: the charge fires the apogee delay after the restart. */
void test_reset_after_apogee_fires_the_apogee_charge_then_the_main()
{
  const RecoveryRun run = flyWithReset(2, 3000000UL, afterApogeeBeforeCharge);
  TEST_ASSERT_TRUE(run.recovered);
  TEST_ASSERT_TRUE(run.eventsAtResume.apogee);
  TEST_ASSERT_FALSE(run.eventsAtResume.apogeeFire);
  TEST_ASSERT_TRUE_MESSAGE(run.apogeeFireAt > 0.0F, "the apogee charge did not fire");
  TEST_ASSERT_FLOAT_WITHIN(0.05F, run.resumeAt + 3.0F, run.apogeeFireAt);
  TEST_ASSERT_TRUE(run.mainFireAt > 0.0F);
}

/** In the coast: apogee is detected near the real apogee, not at the restart. */
void test_reset_in_coast_detects_apogee_at_apogee()
{
  const RecoveryRun reference = flyWithReset(2, 1000000UL, nullptr);
  const RecoveryRun run = flyWithReset(2, 1000000UL, inCoast);
  TEST_ASSERT_TRUE(run.recovered);
  TEST_ASSERT_TRUE(run.eventsAtResume.boosterBurnout);
  TEST_ASSERT_FALSE(run.eventsAtResume.apogee);
  TEST_ASSERT_TRUE(reference.apogeeFireAt > 0.0F);
  TEST_ASSERT_TRUE_MESSAGE(run.apogeeFireAt > 0.0F, "the apogee charge did not fire");
  char message[160];
  snprintf(message, sizeof(message), "apogee charge at %.3f s, %.3f s without the reset", run.apogeeFireAt, reference.apogeeFireAt);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5F, reference.apogeeFireAt, run.apogeeFireAt, message);
  TEST_ASSERT_TRUE(run.mainFireAt > 0.0F);
}

/** Recovery without pyros (setting 1): the flight resumes and nothing fires. */
void test_reset_without_pyros_fires_nothing()
{
  const RecoveryRun run = flyWithReset(1, 1000000UL, afterApogeeCharge);
  TEST_ASSERT_TRUE(run.recovered);
  TEST_ASSERT_TRUE_MESSAGE(run.apogeeFireAt < 0.0F && run.mainFireAt < 0.0F, "a pyro fired");
  TEST_ASSERT_TRUE(run.finalEvents.mainDeploy);
}

/** On the pad with a flight's record left in EEPROM: setup() starts normally. */
void test_reset_on_the_pad_starts_normally()
{
  // a stale record: the flight before was powered off before touchdown was detected
  restoreStartupState(pristine);
  const RecoveryRun run = []
  {
    RecoveryRun r;
    const SyntheticFlight flight(profile());
    truthSource = &flight;
    loadBoardEeprom(2);
    shimEeprom.bytes[eeprom.lastEvent] = FIRE_APOGEE;
    floatUnion.val = 1400.0F;
    memcpy(&shimEeprom.bytes[eeprom.baseAlt], floatUnion.bytes, 4);
    const std::vector<uint8_t> keptEeprom(shimEeprom.bytes, shimEeprom.bytes + ShimEEPROM::size);
    restoreStartupState(pristine);
    memcpy(shimEeprom.bytes, keptEeprom.data(), keptEeprom.size());
    ioLog.clear();
    settingsFile_ = flightSettingsFile(2, 1000000UL);
    boardInputs = BoardInputs();
    boardInputs.testReadPin = 5;
    boardInputs.testButton = {{1, HIGH}};
    boardInputs.highPins = {14, 16};
    accelFeedHook = accelFromTruth;
    baroFeedHook = baroFromTruth;
    accel.gainX = accel.gainY = accel.gainZ = ACCEL_G_PER_COUNT;
    accel.timeBtwnSamp = ACCEL_SAMPLE_US;
    gyro.gainX = gyro.gainY = gyro.gainZ = 0.07F;
    gyro.timeBtwnSamp = GYRO_SAMPLE_US;
    highG.gainX = highG.gainY = highG.gainZ = HIGHG_G_PER_COUNT;
    highG.timeBtwnSamp = HIGHG_SAMPLE_US;
    mag.timeBtwnSamp = 25000;
    baro.timeBtwnSamp = BARO_SAMPLE_US;
    SensorData *all[] = {&accel, &gyro, &mag, &highG};
    for (SensorData *s : all)
    {
      s->dirX = s->dirY = s->dirZ = 1;
    }
    shimSetMicros(1000000UL); // at rest: the profile's motor lights at 5 s
    shimMicrosPerCall = 37;
    setup();
    shimMicrosPerCall = 0;
    accelFeedHook = nullptr;
    baroFeedHook = nullptr;
    r.recovered = std::find(ioLog.begin(), ioLog.end(), "createNextFileSD") == ioLog.end();
    r.eventsAtResume = events;
    r.lastEventAfterSetup = shimEeprom.bytes[eeprom.lastEvent];
    return r;
  }();
  TEST_ASSERT_FALSE_MESSAGE(run.recovered, "setup() resumed a flight on the pad");
  TEST_ASSERT_FALSE(run.eventsAtResume.liftoff);
  TEST_ASSERT_TRUE(run.eventsAtResume.preLiftoff);
  TEST_ASSERT_EQUAL_UINT8(27, run.lastEventAfterSetup);
}

void setUp() {}
void tearDown() {}

int main()
{
  pristine = saveStartupState();
  UNITY_BEGIN();
  RUN_TEST(test_reset_under_drogue_fires_only_the_main);
  RUN_TEST(test_reset_brings_the_hardware_up_as_setup_does);
  RUN_TEST(test_reset_after_apogee_fires_the_apogee_charge_then_the_main);
  RUN_TEST(test_reset_in_coast_detects_apogee_at_apogee);
  RUN_TEST(test_reset_without_pyros_fires_nothing);
  RUN_TEST(test_reset_on_the_pad_starts_normally);
  return UNITY_END();
}
