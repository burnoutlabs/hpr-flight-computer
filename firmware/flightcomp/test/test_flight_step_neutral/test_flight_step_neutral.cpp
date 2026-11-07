/**
 * @file test_flight_step_neutral.cpp
 * @brief flightStep() against the V4.7.2 code it was extracted from, side by side, pass by pass.
 *
 * @details
 * The reference is the code at commit 7c775e3, compiled here from verbatim copies in
 * test/legacy_v4_7_2/ (the files cannot carry a comment, so their origin is recorded here):
 * - loop_flight_step.inc: src/flightcomp.cpp lines 1738-1906, liftoff detection through the
 *   checkEvents() call inside loop();
 * - check_events.inc: include/Event_Logic.h lines 37-643, checkEvents();
 * - fire_and_pulse_pyros.inc: src/flightcomp.cpp lines 2601-2693, firePyros() and pulsePyro().
 * Each can be checked against the commit, for example:
 *   git show 7c775e3:firmware/flightcomp/src/flightcomp.cpp | sed -n 1738,1906p \
 *     | diff - firmware/flightcomp/test/legacy_v4_7_2/loop_flight_step.inc
 * One line differs on purpose. #14 removed accelerometer apogee detection from the flight step,
 * so check_events.inc line 451 sets accelApogee to false instead of accelVel < -10; the diff
 * against Event_Logic.h shows only that line. Everything else is V4.7.2 as it was. Against the
 * unmodified V4.7.2 line, only the low-reading-accelerometer scenario diverges (in events, at
 * apogee): that is the behaviour #14 changes.
 * Their I/O (digitalWrite, EEPROM.update, the radio timer, syncSd, Serial) is redirected into the
 * same log that RecordingIo writes for the new code.
 *
 * Every loop pass of every scenario: the sensors are sampled and processBaroSamp() runs once
 * (shared, not under test); the whole flight state is saved; the reference runs; the state is
 * restored and flightStep() runs, then performFlightActions(). The two must leave identical bytes
 * in every global (flight_state.h), log identical I/O in the same order, and agree on whether the
 * pass was in flight. The loop period varies pseudo-randomly between 300 and 700 us.
 *
 * The scenarios are synthetic flights (synthetic_flights.h) and the three logged flights of the
 * replay suite (test/replay/, CC BY 4.0, see ATTRIBUTION.md).
 */
#include <unity.h>

#include "synthetic_flights.h"

#include "replay/flight_hyperion1.h"
#include "replay/flight_hyperion2.h"
#include "replay/flight_prometheus.h"

#include "flight_state.h"

//-----------------------------------------
// The reference's I/O, logged like RecordingIo
//-----------------------------------------
void digitalWrite(uint8_t pin, uint8_t level) { RecordingIo::pinWrite(pin, level); }
void syncSd() { RecordingIo::syncSd(); }
void timerBuildPkt() { buildPkt = true; }

struct LegacyEeprom
{
  void update(int address, uint8_t value) { RecordingIo::eepromUpdate(address, value); }
};
static LegacyEeprom legacyEeprom;

struct LegacySerial
{
  void println(const char *text) { RecordingIo::println(text); }
  void println(char c)
  {
    const char text[2] = {c, 0};
    RecordingIo::println(text);
  }
};
static LegacySerial legacySerial;

struct LegacyTimer
{
  bool begin(void (*)(), uint32_t microseconds)
  {
    RecordingIo::radioTimerBegin(microseconds);
    return true;
  }
  void end() { RecordingIo::radioTimerEnd(); }
};
static LegacyTimer radioTimer;

//-----------------------------------------
// The reference: V4.7.2's code, verbatim
//-----------------------------------------
// The reference names the pyro channels pyro1 to pyro4, which are pyros[0] to pyros[3] now
static PyroMap &pyro1 = pyros[0];
static PyroMap &pyro2 = pyros[1];
static PyroMap &pyro3 = pyros[2];
static PyroMap &pyro4 = pyros[3];
// The reference's name for the telemetry values, radioTelemetry now
static RadioTelemetry &radio = radioTelemetry;

#define EEPROM legacyEeprom
#define Serial legacySerial
#define checkEvents legacyCheckEvents
#define firePyros legacyFirePyros
#define pulsePyro legacyPulsePyro

#include "../legacy_names.h"

#include "../legacy_v4_7_2/fire_and_pulse_pyros.inc"

#include "../legacy_v4_7_2/check_events.inc"

/** loop()'s flight part as V4.7.2 wrote it; returns whether the in-flight block ran. */
static bool legacyFlightStep()
{
  bool inFlight = false;
#include "../legacy_v4_7_2/loop_flight_step.inc"
  inFlight = true;
}
return inFlight;
}

#include "../legacy_names_end.h"

#undef EEPROM
#undef Serial
#undef checkEvents
#undef firePyros
#undef pulsePyro

//-----------------------------------------
// Side-by-side runner
//-----------------------------------------
static std::vector<uint8_t> initialState;

struct RunSummary
{
  uint32_t passes = 0;
  uint32_t inFlightPasses = 0;
  uint32_t actions = 0;
  std::vector<std::string> io; // every I/O entry of the run, in order
  EventList finalEvents;
  bool sawFalseReset = false;
};

static bool ioContains(const RunSummary &run, const std::string &prefix)
{
  for (const std::string &entry : run.io)
  {
    if (entry.compare(0, prefix.size(), prefix) == 0)
    {
      return true;
    }
  }
  return false;
}

/** Restore the globals to their start-up values, then apply the scenario's settings. */
static void startScenario()
{
  restoreFlightState(initialState);
  configureSensors();
  configurePyros('A', 'M', 'N', 'N');
  settings.mainDeployAlt = 200;
  settings.apogeeDelay = 1000000UL;
  settings.altThreshold = 91;
  settings.maxAngle = 300;
}

template <typename Source>
static RunSummary runSideBySide(const char *name, const Source &flight, const SensorModel &model = SensorModel())
{
  RunSummary run;
  LoopState loopState;
  loopState.model = model;
  uint32_t jitter = 1U;
  char message[256];
  uint32_t now = 0;
  std::vector<uint8_t> before, legacyState, newState;
  std::vector<std::string> legacyIo;
  while (now < flight.durationMicros())
  {
    jitter = jitter * 1103515245U + 12345U;
    now += 300 + (jitter >> 16) % 401;
    samplePass(loopState, now, flight.at(now));
    // a GNSS clock that moves, so a capture at the wrong moment shows
    gps.time.hours = (uint8_t)((now / 3600000000UL) % 24);
    gps.time.minutes = (uint8_t)((now / 60000000UL) % 60);
    gps.time.seconds = (uint8_t)((now / 1000000UL) % 60);
    gps.time.centiseconds = (uint8_t)((now / 10000UL) % 100);

    saveFlightState(before);
    ioLog.clear();
    const bool legacyInFlight = legacyFlightStep();
    legacyIo.swap(ioLog);
    saveFlightState(legacyState);

    restoreFlightState(before);
    ioLog.clear();
    const FlightActions actions = flightStep();
    performFlightActions<RecordingIo>(actions);
    saveFlightState(newState);

    if (legacyInFlight != actions.inFlight)
    {
      snprintf(message, sizeof(message), "%s, t=%lu us: in-flight %d (V4.7.2) vs %d (flightStep)", name, (unsigned long)now, legacyInFlight, actions.inFlight);
      TEST_FAIL_MESSAGE(message);
    }
    if (legacyIo != ioLog)
    {
      snprintf(message, sizeof(message), "%s, t=%lu us: I/O differs: %zu entries (V4.7.2) vs %zu (flightStep), first %s vs %s", name, (unsigned long)now,
               legacyIo.size(), ioLog.size(), legacyIo.empty() ? "-" : legacyIo[0].c_str(), ioLog.empty() ? "-" : ioLog[0].c_str());
      TEST_FAIL_MESSAGE(message);
    }
    if (legacyState != newState)
    {
      const std::string differs = firstDifference(legacyState, newState);
      snprintf(message, sizeof(message), "%s, t=%lu us: state differs in %s", name, (unsigned long)now, differs.c_str());
      TEST_FAIL_MESSAGE(message);
    }
    TEST_ASSERT_TRUE(actions.count <= FLIGHT_ACTIONS_MAX);

    run.passes++;
    run.inFlightPasses += actions.inFlight;
    run.actions += actions.count;
    run.io.insert(run.io.end(), ioLog.begin(), ioLog.end());
    for (uint8_t i = 0; i < actions.count; i++)
    {
      run.sawFalseReset |= actions.kind[i] == FlightAction::RADIO_PRELIFTOFF;
    }
    finishPass(actions.inFlight);
  }
  run.finalEvents = events;
  printf("%-40s %7lu passes, %7lu in flight, %4lu actions\n", name, (unsigned long)run.passes, (unsigned long)run.inFlightPasses, (unsigned long)run.actions);
  return run;
}

//-----------------------------------------
// Scenarios
//-----------------------------------------
static ProfileSpec singleStage()
{
  ProfileSpec spec;
  spec.burns = {{5.0F, 3.0F, 100.0F}};
  spec.duration = 135.0F;
  return spec;
}

void test_single_stage_flight()
{
  startScenario();
  settings.inflightRecover = 1;
  const RunSummary run = runSideBySide("single stage, in-flight recovery on", SyntheticFlight(singleStage()));
  TEST_ASSERT_TRUE(run.finalEvents.boosterBurnout);
  TEST_ASSERT_TRUE(run.finalEvents.apogeeFire);
  TEST_ASSERT_TRUE(run.finalEvents.mainDeploy);
  TEST_ASSERT_TRUE(run.finalEvents.touchdown);
  TEST_ASSERT_TRUE(ioContains(run, "eeprom 228 "));
  TEST_ASSERT_TRUE(ioContains(run, "pin 21 1"));
  TEST_ASSERT_TRUE(ioContains(run, "pin 22 1"));
  TEST_ASSERT_TRUE(ioContains(run, "syncSd"));
}

void test_booster_profile_flight()
{
  startScenario();
  settings.fltProfile = 'B';
  settings.inflightRecover = 1;
  const RunSummary run = runSideBySide("booster profile", SyntheticFlight(singleStage()));
  TEST_ASSERT_TRUE(run.finalEvents.touchdown);
}

void test_test_mode_with_pyro_pwm()
{
  startScenario();
  settings.testMode = true;
  applyTestModeOverrides();
  sensors.pyroPwm = true;
  settings.inflightRecover = 1;
  configurePyros('A', 'M', 'A', 'M'); // all four channels pulse
  settings.mainDeployAlt = 400; // above apogee: main fires 1 s after the apogee charge, inside the 15 s
  ProfileSpec spec; // apogee well inside test mode's 15 s record time
  spec.burns = {{5.0F, 1.5F, 60.0F}};
  spec.duration = 30.0F;
  const RunSummary run = runSideBySide("test mode, pyro PWM", SyntheticFlight(spec));
  TEST_ASSERT_TRUE(run.finalEvents.timeOut);
  TEST_ASSERT_TRUE(run.finalEvents.apogeeFire);
  TEST_ASSERT_TRUE(ioContains(run, "println Simulated Liftoff Detected!"));
  TEST_ASSERT_TRUE(ioContains(run, "println Live-pyro alarm activated"));
  TEST_ASSERT_TRUE(ioContains(run, "pin 21 0"));
  TEST_ASSERT_TRUE(ioContains(run, "pin 23 0"));
  TEST_ASSERT_TRUE(ioContains(run, "pin 24 0"));
}

void test_no_liftoff()
{
  startScenario();
  ProfileSpec spec;
  spec.padBumps = {{5.0F, 0.2F, 12.0F}}; // 2.2 g for 0.2 s: below the 2.5 g trigger
  spec.duration = 30.0F;
  const RunSummary run = runSideBySide("on the pad, no liftoff", SyntheticFlight(spec));
  TEST_ASSERT_EQUAL_UINT32(0, run.inFlightPasses);
  TEST_ASSERT_EQUAL_UINT32(0, run.actions);
}

void test_false_liftoff_then_flight()
{
  startScenario();
  settings.testMode = false;
  ProfileSpec spec = singleStage();
  spec.burns = {{10.0F, 3.0F, 100.0F}};
  spec.padBumps = {{5.0F, 0.15F, 30.0F}}; // 4 g for 150 ms: liftoff, then a false-liftoff reset
  spec.duration = 140.0F;
  const RunSummary run = runSideBySide("false liftoff, then a flight", SyntheticFlight(spec));
  TEST_ASSERT_TRUE(run.sawFalseReset);
  TEST_ASSERT_TRUE(run.finalEvents.touchdown);
}

void test_false_liftoff_in_test_mode()
{
  startScenario();
  // test mode with the flight timings; the bench timings' own reset is in test_setup_neutral
  // (test_bench_mode_resets_a_short_bump)
  settings.testMode = true;
  ProfileSpec spec;
  spec.padBumps = {{5.0F, 0.15F, 30.0F}};
  spec.duration = 10.0F;
  const RunSummary run = runSideBySide("false liftoff in test mode", SyntheticFlight(spec));
  TEST_ASSERT_TRUE(ioContains(run, "println False Trigger Reset"));
}

void test_short_burn_is_reset_as_false_liftoff()
{
  startScenario();
  ProfileSpec spec;
  spec.burns = {{5.0F, 0.4F, 60.0F}};
  spec.duration = 40.0F;
  const RunSummary run = runSideBySide("0.4 s burn (aborted flight)", SyntheticFlight(spec));
  TEST_ASSERT_TRUE(run.sawFalseReset);
}

void test_low_flight_below_altitude_threshold()
{
  startScenario();
  ProfileSpec spec;
  spec.burns = {{5.0F, 0.8F, 60.0F}};
  spec.duration = 40.0F;
  const RunSummary run = runSideBySide("0.8 s burn, low apogee", SyntheticFlight(spec));
  TEST_ASSERT_TRUE(run.finalEvents.apogeeFire);
  TEST_ASSERT_TRUE(run.finalEvents.mainDeploy);
}

void test_timeout_during_descent()
{
  startScenario();
  settings.rcdTime = 40000000UL;
  const RunSummary run = runSideBySide("record time ends in descent", SyntheticFlight(singleStage()));
  TEST_ASSERT_TRUE(run.finalEvents.timeOut);
  TEST_ASSERT_FALSE(run.finalEvents.touchdown);
}

void test_apogee_from_a_low_reading_accelerometer()
{
  startScenario();
  SensorModel model;
  // reads 0.05 g low, so its integrated velocity reaches -10 m/s before apogee: V4.7.2 set apogee
  // on that; with accelerometer apogee removed (#14) both sides wait for baro or fusion
  model.accelBias = -0.5F;
  const RunSummary run = runSideBySide("accelerometer 0.05 g low", SyntheticFlight(singleStage()), model);
  TEST_ASSERT_TRUE(run.finalEvents.apogeeFire);
  TEST_ASSERT_TRUE(run.finalEvents.touchdown);
}

void test_high_g_boost()
{
  startScenario();
  ProfileSpec spec = singleStage();
  spec.burns = {{5.0F, 1.2F, 250.0F}}; // 26 g, past the 16 g accelerometer
  const RunSummary run = runSideBySide("26 g boost on the high-G sensor", SyntheticFlight(spec));
  TEST_ASSERT_TRUE(run.finalEvents.touchdown);
}

static ProfileSpec twoStage()
{
  ProfileSpec spec;
  // booster burns out at 7.5 s; separation 0.5 s later, sustainer fired 1.0 s after that
  spec.burns = {{5.0F, 2.5F, 100.0F}, {9.1F, 3.0F, 80.0F}};
  spec.duration = 195.0F;
  return spec;
}

void test_two_stage_flight()
{
  startScenario();
  settings.fltProfile = '2';
  settings.inflightRecover = 1;
  configurePyros('B', 'I', 'A', 'M');
  const RunSummary run = runSideBySide("two stage", SyntheticFlight(twoStage()));
  TEST_ASSERT_TRUE(run.finalEvents.boosterSeparation);
  TEST_ASSERT_TRUE(run.finalEvents.sustainerFire);
  TEST_ASSERT_TRUE(run.finalEvents.sustainerIgnition);
  TEST_ASSERT_TRUE(run.finalEvents.sustainerBurnout);
  TEST_ASSERT_TRUE(run.finalEvents.touchdown);
}

void test_two_stage_inhibited_by_tilt()
{
  startScenario();
  settings.fltProfile = '2';
  settings.inflightRecover = 1;
  configurePyros('B', 'I', 'A', 'M');
  ProfileSpec spec = twoStage();
  spec.pitchRate = 15.0F;
  const RunSummary run = runSideBySide("two stage, sustainer inhibited by tilt", SyntheticFlight(spec));
  TEST_ASSERT_TRUE(run.finalEvents.sustainerFireCheck);
  TEST_ASSERT_FALSE(run.finalEvents.sustainerFire);
}

void test_two_stage_inhibited_by_altitude()
{
  startScenario();
  settings.fltProfile = '2';
  settings.altThreshold = 30000;
  configurePyros('B', 'I', 'A', 'M');
  const RunSummary run = runSideBySide("two stage, sustainer inhibited by altitude", SyntheticFlight(twoStage()));
  TEST_ASSERT_TRUE(run.finalEvents.sustainerFireCheck);
  TEST_ASSERT_FALSE(run.finalEvents.sustainerFire);
}

static ProfileSpec airstart()
{
  ProfileSpec spec;
  // booster burnout at 7.0 s; airstart 1 fired 0.5 s later; airstart 2 fired 0.5 s after airstart 1 ignites
  spec.burns = {{5.0F, 2.0F, 100.0F}, {7.6F, 1.5F, 60.0F}, {8.2F, 1.5F, 60.0F}};
  spec.duration = 160.0F;
  return spec;
}

void test_airstart_on_burnout_then_ignition()
{
  startScenario();
  settings.fltProfile = 'A';
  settings.inflightRecover = 1;
  settings.airStart1Event = 'B';
  settings.airStart2Event = 'I';
  configurePyros('1', '2', 'A', 'M');
  const RunSummary run = runSideBySide("airstart: 1 on burnout, 2 on ignition", SyntheticFlight(airstart()));
  TEST_ASSERT_TRUE(run.finalEvents.airStart1Fire);
  TEST_ASSERT_TRUE(run.finalEvents.airStart1Ignition);
  TEST_ASSERT_TRUE(run.finalEvents.airStart2Fire);
  TEST_ASSERT_TRUE(run.finalEvents.airStart2Ignition);
  TEST_ASSERT_TRUE(run.finalEvents.touchdown);
}

void test_airstart_on_ignition_then_burnout()
{
  startScenario();
  settings.fltProfile = 'A';
  settings.inflightRecover = 1;
  settings.airStart1Event = 'I';
  settings.airStart1Delay = 1500000UL;
  settings.airStart2Event = 'B';
  configurePyros('1', '2', 'A', 'M');
  ProfileSpec spec;
  // airstart 1 fired 1.5 s after liftoff (during the booster burn); airstart 2 on its burnout
  spec.burns = {{5.0F, 2.0F, 100.0F}, {6.6F, 1.5F, 40.0F}, {8.7F, 1.5F, 60.0F}};
  spec.duration = 150.0F;
  const RunSummary run = runSideBySide("airstart: 1 on ignition, 2 on burnout", SyntheticFlight(spec));
  TEST_ASSERT_TRUE(run.finalEvents.airStart1Fire);
  TEST_ASSERT_TRUE(run.finalEvents.airStart2Check);
}

void test_airstart_inhibited()
{
  startScenario();
  settings.fltProfile = 'A';
  settings.airStart1Event = 'B';
  settings.airStart2Event = 'B';
  settings.altThreshold = 30000;
  configurePyros('1', '2', 'A', 'M');
  ProfileSpec spec = airstart();
  spec.pitchRate = 15.0F;
  const RunSummary run = runSideBySide("airstart inhibited by tilt and altitude", SyntheticFlight(spec));
  TEST_ASSERT_TRUE(run.finalEvents.airStart1Check);
  TEST_ASSERT_FALSE(run.finalEvents.airStart1Fire);
}

void test_stable_rotation_path()
{
  startScenario();
  settings.stableRotn = true;
  ProfileSpec spec = singleStage();
  spec.pitchRate = 5.0F;
  const RunSummary run = runSideBySide("active stabilization rotation path", SyntheticFlight(spec));
  TEST_ASSERT_TRUE(run.finalEvents.touchdown);
}

/** A logged flight as a source of sensor truth, with the pad and rest periods replayFlight() adds. */
struct LoggedFlight
{
  ReplayLog log;
  explicit LoggedFlight(const ReplayFixture &fixture) : log(fixture) {}
  Truth at(uint32_t micros) const { return log.at(ReplayLog::logTime(micros)); }
  uint32_t durationMicros() const { return (uint32_t)((REPLAY_PAD_SECONDS + log.t.back() + REPLAY_REST_SECONDS) * 1e6F); }
};

static void runLoggedFlight(const ReplayFixture &fixture)
{
  restoreFlightState(initialState);
  replaySettings();
  const RunSummary run = runSideBySide(fixture.name, LoggedFlight(fixture));
  TEST_ASSERT_TRUE(run.finalEvents.apogeeFire);
  TEST_ASSERT_TRUE(run.finalEvents.mainDeploy);
  TEST_ASSERT_TRUE(run.finalEvents.touchdown);
}

void test_logged_flight_prometheus() { runLoggedFlight(flight_prometheus::fixture); }
void test_logged_flight_hyperion1() { runLoggedFlight(flight_hyperion1::fixture); }
void test_logged_flight_hyperion2() { runLoggedFlight(flight_hyperion2::fixture); }

void setUp() {}
void tearDown() {}

int main()
{
  initialState = saveFlightState();
  UNITY_BEGIN();
  RUN_TEST(test_single_stage_flight);
  RUN_TEST(test_booster_profile_flight);
  RUN_TEST(test_test_mode_with_pyro_pwm);
  RUN_TEST(test_no_liftoff);
  RUN_TEST(test_false_liftoff_then_flight);
  RUN_TEST(test_false_liftoff_in_test_mode);
  RUN_TEST(test_short_burn_is_reset_as_false_liftoff);
  RUN_TEST(test_low_flight_below_altitude_threshold);
  RUN_TEST(test_timeout_during_descent);
  RUN_TEST(test_apogee_from_a_low_reading_accelerometer);
  RUN_TEST(test_high_g_boost);
  RUN_TEST(test_two_stage_flight);
  RUN_TEST(test_two_stage_inhibited_by_tilt);
  RUN_TEST(test_two_stage_inhibited_by_altitude);
  RUN_TEST(test_airstart_on_burnout_then_ignition);
  RUN_TEST(test_airstart_on_ignition_then_burnout);
  RUN_TEST(test_airstart_inhibited);
  RUN_TEST(test_stable_rotation_path);
  RUN_TEST(test_logged_flight_prometheus);
  RUN_TEST(test_logged_flight_hyperion1);
  RUN_TEST(test_logged_flight_hyperion2);
  return UNITY_END();
}
