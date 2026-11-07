/**
 * @file flightcomp.cpp
 * @brief High-Power Rocketry Flight Computer for Teensy 3.2, 3.5, 3.6, 4.0, and 4.1
 *
 * Original sketch by Bryan Sparkman, TRA #12111, NAR #85720, L3
 * Modified and continued by Cameron K. Brooks
 *
 *
 * @section features Features
 * - Dual-deploy flight computer capable to over 100,000ft
 * - Two-stage & airstart capable with tilt-sensing safety features
 * - Live telemetry over 433MHz or 915MHz bands
 *   - 433MHz: USA amateur 70cm band, EUR license-free
 *   - 915MHz: USA FHSS license-free
 *   - Optional APRS packets at 0.5Hz or telemetry data packets at 5Hz (APRS in development)
 *   - Optional LoRa or GFSK modulation (in development)
 *   - Optional 915MHz single channel or FHSS
 *   - Auto-adjusting power gain
 * - 4 programmable high-current pyro outputs with continuity checks
 * - Captures high-rate data at approximately 50,000 data points per second on SD card
 *   - 1000Hz 3-axis digital 16G and 100G accelerometer data logging
 *   - 1000Hz 3-axis digital 2000dps gyroscope data logging
 *   - 1000Hz of 21 flight events & continuity data logging on 4 pyros
 *   - 1000Hz of sensor-fused speed & altitude
 *   - 100Hz of roll, pitch, yaw, and combined pitch-yaw 3D rotation
 *   - 40Hz of magnetic data logging and magnetic roll
 *   - 30Hz-100Hz of digital barometric data logging (data rate is chip-dependent)
 *   - 30Hz of main battery voltage (1000Hz during pyro events)
 *   - 20Hz of LoRa telemetry output (time, event, acceleration, speed, altitude, rotation, GNSS altitude, GNSS position, signal strength, packet number)
 *   - 5Hz-25Hz of GNSS data logging (chip-dependent data rates & constellations)
 * - Separate data file for each flight (up to 100 flights)
 * - Simple, easy-to-use setup through the SD card
 *   - User Selectable Flight Mode: Single-Stage, Two-Stage, Airstart, or Booster
 *   - Configurable Apogee delay
 *   - Optional Audible Battery Voltage report at startup
 *   - Optional Magnetic Switch Startup & Shutdown
 *   - Preflight audible reporting options: Perfectflight or Marsa
 *   - 8 configurable servo outputs (4 powered, 4 unpowered)
 *   - User-selectable inflight brownout recovery
 *   - User-selectable active stabilization for roll, pitch, and yaw correction
 *   - User-selectable return-to-pad controlled recovery
 * - Tilt-sensing lockout for ignition of second stages and/or airstarts
 * - Mach immune, sensor-fusion based apogee event
 * - Barometric-based main deploy event
 * - Optional GNSS NMEA Log for TRA altitude contest reporting
 * - Audible pre-flight continuity report
 * - Audible Post-flight max altitude & speed report
 * - Mount in any orientation with automatic orientation detection during calibration
 * - Bench-test mode activated with tactile button; user-configurable status messages over USB Serial
 * - Built-in self-calibration mode
 * - Report in SI or Metric units
 * - Compatible with Teensy 3.2, 3.5, 3.6, 4.0, 4.1
 *   - Compatible with any sensor over I2C or SPI: optimized drivers support specific sensors, external libraries can be used for unsupported devices
 *   - Configurable GPIO pins and hardware I2C/SPI bus options
 *   - GNSS receiver can be wired to any available HW Serial port (UBLOX chipsets & Adafruit Ultimate GPS supported, or use an external library)
 *
 * @section changelog Change Log
 * - V4_0_0 combines all previous versions coded for specific hardware setups
 * - V4_1_0 adds upgrades for airstart capability, more flight event codes, improved settings file, PWM Pyro firing, quaternion rotation
 * - V4_1_1 eliminates timer interrupts, improves telemetry timing, enables the Galileo GNSS system
 * - V4_1_2 increases the GPS data rates and fixes some bugs in the pyro firing timing
 * - V4_1_3 adds features to the settings file, moves some settings to the EEPROM file, adds calibration for the MPL3115A2
 * - V4_1_4 adds magnetic rotation and increases the magnetic data capture rate
 * - V4_2_0 adds 915MHz FHSS telemetry capability, configurable GNSS & radio debugging
 * - V4_2_2 adds support for the MS5611 pressure sensor
 * - V4_2_3 adds inflight recovery from brown-out or MCU reset, sensor fusion altitude and velocity, eliminates gyro high-pass filter
 * - V4_3_0 updates Quaternion rotation for the best possible precision, corrects some GNSS bugs
 * - V4_3_1 makes FHSS an option on 915MHz and corrects some bugs in the FHSS power settings code
 * - V4_3_2 fixes bugs in the orientation & rotation routines, forces inline functions for faster execution, fixes multiple bugs picked up by the new Arduino compiler
 * - V4_3_3 creates a user interface over serial to calibrate the barometer, extends gyro orientation integration through the entire flight
 * - V4_3_4 enables active stabilization, creates basic fin trim code, and enables auto-adjusting gain on the IMU accelerometer
 * - V4_3_5 removes auto-adjusting gain, improves calibration routines, extends baro calibration to all sensors, removed high-G axis mode from user settings
 * - V4_3_6 adds return-to-base functionality (beta testing), supports LSM6DS33, LIS3MDL, and MS5607, updates code for separate mag sensors
 * - V4_4_0 creates flexible I2C and SPI bus routines for various devices and buses across Teensy3.X and Teensy4.X platforms
 * - V4_4_2 further streamlines the bus management functions with more efficient pointers, overclocks I2C speed on some sensors, implements controlled sampling of all sensors, continuously checks pre-flight continuity, improved barometric smoothing
 * - V4_5_0 eliminates RadioHead library due to interferences with the PWMServo library, now uses an interval timer for precise control of the radio packet timing
 * - V4_5_1 adds optional GPS output file to capture NMEA sentences for contest flights, creates better radio resiliency by adding the callsign back into the header of the packet
 * - V4_5_2 finishes the draft RTB code, adds a canard calibration flight mode, fixed GPS bug with NEO-M8N
 * - V4_5_3 uses the RFM96W in both 70cm and 900MHz mode based on user frequency input for 3 options
 * - V4_5_4 fixes a bug in the sensor timing that reduced the effective data capture rate
 * - V4_5_5 removes launch detection user options since the algorithm is proven reliable, fixes a timing bug with the barometers, makes GPS configuration code more portable, adds LPS25H support, fixes bugs with LSM6DS33 and LIS3MDL
 * - V4_6_0 improves portability of quaternion rotation code, adds MPU6050 support, fixed calibration routine bug, cleaned up some of the GPS data processing, corrected major bug in high-G moving average
 * - V4_7_0 adds support for any sensor with an external library, fixes a major bug that caused intermittent resetting when moving up the rail, breaks sensor management out into its own file, cleans up some of the structs, adds LSM6DSOX support, fixes more LIS3MDL bugs
 * - V4_7_1 uses polymorphisms for the sensor function calls, cleans up some of the radio function calls, adds fault indication if a critical sensor fails, fixes a GNSS output to serial monitor bug, adds LSM6DS3TR support, fixes a beeper bug on Teensy 4.1
 * - V4_7_2 applies a new telemetry packet scheme to increase efficiency, fixes an LSM6DS3TR bug in the accelerometer output, reduces motor shutdown detection time to better detect apogee after skywriting or motor failure, adds an alarm when pyros can be actively fired, fixes bugs in the beeping routine, adds file rename functionality
 *
 * @section future_upgrades Future Upgrades
 * - Active Stabilization (started)
 * - Return-to-Base capability (started)
 * - 3D position physics model
 * - Remote Arm & Shutdown Commands over telemetry
 * - Ground Station Bluetooth Datalink to smartphone (started)
 * - Smartphone App
 *
 * @section todo To-Do List
 * - Finish bench testing of inflight recovery routines
 * - Flight test airstart code
 * - Debug inflight recovery
 * - Stuck-in-a-loop detection and breakout
 */

//-------CODE START--------
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <PWMServo.h>
#include <SPI.h>
#include <string.h>
#include "declarations.h"
#include "board.h"
#include "flightcomp.h"
#include "Bus_Mgmt.h"
#include "Calibration.h"
#include "Device_Mgmt.h"
#include "FlightStep.h"
#include "InflightRecover.h"
#include "Rotation.h"
#include "SD_Card.h"
#include "SX127X_Driver.h"
#include "Startup.h"
#include "Telemetry.h"

// GPS Setup
TinyGPSPlus gps;
bool restoreDefaults = false;

// Servo Setup
PWMServo servos[8];

// Timer for the radio
#if defined(STM32_CORE_VERSION)
#if defined(TIM1)
TIM_TypeDef *Instance = TIM1;
#else
TIM_TypeDef *Instance = TIM2;
#endif
HardwareTimer *radioTimer = new HardwareTimer(Instance);
HardwareTimer *syncTimer = new HardwareTimer(Instance);
#else
IntervalTimer radioTimer;
IntervalTimer syncTimer;
#endif

// any routines called in the main flight loop need to be as fast as possible, so flightStep()
// (FlightStep.h) and writeSdFlightData() (SD_Card.h) are defined "always inline" in their headers

// The board I/O that the flight step's actions drive (performFlightActions() in FlightStep.h)
struct BoardIo
{
  static inline __attribute__((always_inline)) void pinWrite(uint8_t pin, uint8_t level) { digitalWrite(pin, level); }
  static inline __attribute__((always_inline)) void eepromUpdate(int address, uint8_t value) { EEPROM.update(address, value); }
  static inline __attribute__((always_inline)) void radioTimerEnd() { radioTimer.end(); }
  static inline __attribute__((always_inline)) void radioTimerBegin(uint32_t microseconds) { radioTimer.begin(timerBuildPkt, microseconds); }
  static inline __attribute__((always_inline)) void syncSd() { ::syncSd(); }
  static inline __attribute__((always_inline)) void println(const char *text) { Serial.println(text); }
};

/**
 * @brief Renames the closed flight data file to the GNSS liftoff date and time and the rocket name.
 *
 * @details Defined here rather than in SD.cpp so that loop() can inline its early return, as it
 * did when the flight computer was one sketch.
 *
 * The new name is YYYYMMDDHHMM (UTC) followed by a space and the rocket name, with a
 * .csv extension, for example "202611061530 MyRocket.csv". The rename is by path, so it is called
 * after writeSdFooterClose() has closed the file. Without a GNSS date at liftoff the file keeps
 * its FLIGHTnn.txt name, and if the rename fails (the name exists, or the rocket name holds a
 * character the card's file system rejects) the file is left as it was.
 */
void renameFileSd()
{
  // month is 0 when no GNSS date was captured at liftoff
  if (gnss.liftoff.month == 0)
  {
    return;
  }
  char *p = dataString;
  itoa(gnss.liftoff.year, p, 10);
  p += strlen(p);
  const uint8_t field[4] = {gnss.liftoff.month, gnss.liftoff.day, gnss.liftoff.hour, gnss.liftoff.minute};
  for (uint8_t i = 0; i < 4; i++)
  {
    *p++ = '0' + field[i] / 10;
    *p++ = '0' + field[i] % 10;
  }
  *p++ = ' ';
  // bounded copy: rocketName is not guaranteed to be null-terminated
  for (uint8_t i = 0; i < sizeof(settings.rocketName) && settings.rocketName[i] != '\0'; i++)
  {
    *p++ = settings.rocketName[i];
  }
  strcpy(p, ".csv");
  SD.rename(outputFileName, dataString);
}

/*!
 * @brief Main flight loop for the flight computer.
 *
 * This loop performs sensor sampling, event checking, and other critical tasks during rocket flight.
 */
void loop(void)
{

  // debug
  if (settings.testMode)
  {
    sampleStart = micros();
  }

  // Check if an accelerometer sample is needed and set timestamp
  fltTime.tmClock = sampleTimeCheck = micros();
  if (sampleTimeCheck - accel.timeLastSamp > accel.timeBtwnSamp)
  {
    getAccel();
    orientAccel();
    accel.timeLastSamp += accel.timeBtwnSamp;
    while (sampleTimeCheck - accel.timeLastSamp > accel.timeBtwnSamp)
    {
      accel.timeLastSamp += accel.timeBtwnSamp;
    }
  }

  // Check if an gyroscope sample is needed
  sampleTimeCheck = micros();
  if (sampleTimeCheck - gyro.timeLastSamp > gyro.timeBtwnSamp)
  {
    getGyro();
    orientGyro();
    gyro.timeLastSamp += gyro.timeBtwnSamp;
    while (sampleTimeCheck - gyro.timeLastSamp > gyro.timeBtwnSamp)
    {
      gyro.timeLastSamp += gyro.timeBtwnSamp;
    }
  }

  // Check if a high-G accelerometer sample is needed
  sampleTimeCheck = micros();
  if (sampleTimeCheck - highG.timeLastSamp > highG.timeBtwnSamp)
  {
    getHighG();
    orientHighG();
    highG.timeLastSamp += highG.timeBtwnSamp;
    while (sampleTimeCheck - highG.timeLastSamp > highG.timeBtwnSamp)
    {
      highG.timeLastSamp += highG.timeBtwnSamp;
    }
  }

  // Check if a barometer sample is needed
  sampleTimeCheck = micros();
  if (sampleTimeCheck - baro.timeLastSamp > baro.timeBtwnSamp)
  {
    getBaro();
    // barometers work in single-shot mode at a much lower rate, so we cannot use any time-blocking methods
    baro.timeLastSamp = sampleTimeCheck;
  }

  // Check if a magnetometer sample is needed
  sampleTimeCheck = micros();
  if (sampleTimeCheck - mag.timeLastSamp > mag.timeBtwnSamp)
  {
    getMag();
    orientMag();
    mag.timeLastSamp += mag.timeBtwnSamp;
    while (sampleTimeCheck - mag.timeLastSamp > mag.timeBtwnSamp)
    {
      mag.timeLastSamp += mag.timeBtwnSamp;
    }
  }

  // Sample continuity
  checkPyroContinuity();

  // debug
  if (settings.testMode)
  {
    sampleTime = micros() - sampleStart;
  }

  // process barometric samples
  // See if a new altitude reading is available
  if (baro.newSamp)
  {
    processBaroSamp();
  }

  // look for a shutdown command and if seen, stop all progress for a hard reset
  if (events.preLiftoff && settings.magSwitchEnable)
  {
    uint8_t magShutdown = 0;
    if (abs(mag.x) > magTrigger || abs(mag.y) > magTrigger || abs(mag.z) > magTrigger)
    {
      magShutdown = 1;
    }
    while (magShutdown == 1)
    {
      digitalWrite(pins.beep, HIGH);
      delay(1000);
    }
  }

  // liftoff detection and, in flight, the integration, rotation and event logic (FlightStep.h)
  const FlightActions actions = flightStep();
  performFlightActions<BoardIo>(actions);

  if (actions.inFlight)
  {

    // update the canards throws if stabilization or flyback is enabled (Rotation.cpp)
    if (settings.stableRotn || settings.stableVert || settings.flyBack)
    {
      uint32_t controlTime = micros();
      if (controlTime - timeLastControl >= CONTROL_INTERVAL)
      {
        // if active stabilization is activated, set the canards
        if ((settings.stableVert || settings.stableRotn) && !events.apogee && events.boosterBurnout)
        {
          setCanards();
        }
        // if RTB is on and we are post apogee, then set canards to return to launch point
        if (settings.flyBack && events.apogeeFire && !events.mainDeploy)
        {
          setRtb();
        }
        // update control timer
        timeLastControl = controlTime;
      }
    }

    // Read the battery voltage
    if ((fltTime.timeCurrent - lastVolt > voltRate) || pyroFire)
    {
      voltReading = analogRead(pins.batt);
      voltage = (float)(voltReading) * 3.3 * 3.2 * adcConvert;
      if (pins.batt == pins.pyro4Cont)
      {
        voltage *= (2.72 / 3.2);
      } // on old units we used to pull the battery voltage through one of the pyro channels which had diodes
      writeVolt = true;
      lastVolt = fltTime.timeCurrent;
    }

    // Write the data to a string if we have a new sample
    if (accel.newSamp || gyro.newSamp || highG.newSamp || mag.newSamp || baro.newSamp || baro.newTemp || gnss.sdWrite || writeVolt || sdRadioTx)
    {
      writeSdFlightData();
    }

    // Close file at Touchdown or Timeout
    if (events.timeOut || events.touchdown)
    {
      // write the final footer data to SD card and close
      writeSdFooterClose();
      fileClose = true;
      events.liftoff = false;
      // rename the file using the timestamp and rocket name
      if (!settings.testMode)
      {
        renameFileSd();
      }
      // shutoff all pyro outputs
      for (uint8_t i = 0; i < 4; i++)
      {
        digitalWrite(pyros[i].firePin, LOW);
      }
      // shutoff the live-pyro alarm
      livePyroAlarm = false;
      // if FHSS, start the postFlight synch packet timer
      if (settings.fhss)
      {
        syncTimer.begin(timerSyncPkt, pktInterval.postFlight);
        delayMicroseconds(300000UL);
      }
      // Set the radio transmitter to post-flight data rate
      radioTimer.begin(timerBuildPkt, pktInterval.postFlight);
      // Read max altitude into its beep array
      parseBeep((uint32_t)(baro.maxAlt * unitConvert), maxAltDigits, 6);
      // Read max velocity into its beep array
      parseBeep((uint32_t)(maxVelocity * unitConvert), maxVelDigits, 4);
      // set the beeper controls to audibly beep out the maximum altitude and speed
      digitalWrite(pins.beep, LOW);
      beep = false;
      while (maxAltDigits[altDigits - 1] == 0)
      {
        altDigits--;
      }
      while (maxVelDigits[velDigits - 1] == 0)
      {
        velDigits--;
      }
      beepPosn = altDigits;
      beepCode = maxAltDigits[beepPosn - 1];
      beepAlt = true;
      beepCont = false;
      beepAlarm = false;
      beepDelay = LONG_BEEP_DELAY;
      // store the maximum altitude in EEPROM
      if (!settings.testMode)
      {
        for (uint8_t i = 0; i < 6; i++)
        {
          EEPROM.update(eeprom.maxFltAlt + i, maxAltDigits[i]);
        }
      }
      // update the EEPROM flight counter
      if (!settings.testMode)
      {
        uint8_t flightCount = (uint8_t)EEPROM.read(eeprom.totalFlights);
        flightCount++;
        EEPROM.write(eeprom.totalFlights, flightCount);
      }
      // set the final radio variables
      radioTelemetry.maxAlt = (int16_t)baro.maxAlt;
      radioTelemetry.maxVel = (int16_t)maxVelocity;
      radioTelemetry.maxG = (int16_t)((maxG / 0.980665));
      radioTelemetry.maxGpsAlt = (int16_t)gnss.maxAlt;
      // write out the SD card timing variables
      if (settings.testMode)
      {
        Serial.print(F("Max SD Write Time: "));
        Serial.println(maxWriteTime);
        Serial.print(F("Write Threshold: "));
        Serial.print(writeThreshold);
        Serial.print(F(", Count: "));
        Serial.println(writeThreshCount);
      }
    } // end of timeout/touchdown protocols

  } // end of liftoff flag

  // Radio packet handling
  noInterrupts();
  // use the interrupt to build the packets at regular intervals
  bool pktFlag = buildPkt;
  interrupts();
  if (settings.txEnable && pktFlag)
  {
    buildTelemetryPkt();
  }
  // send the packet when its ready and the radio is available
  if (settings.txEnable && sendPkt && !txInProgress && sensors.statusRadio)
  {
    sendTelemetryPkt();
  }

  // Radio Synchronization packet when 915MHz FHSS is used
  if (settings.fhss && (events.touchdown || events.timeOut))
  {
    noInterrupts();
    syncFreq = syncFlag;
    interrupts();
  }

  // Send the sych packet if needed
  if (syncFreq && !txInProgress)
  {
    syncPkt();
  }

  // Radio clear interrupts on SX127X, this must be called in the main loop because we never know when the interrupt will fire
  if (sensors.radio == 1)
  {
    noInterrupts();
    bool irqFlag = clearTx;
    interrupts();
    if (irqFlag)
    {
      clearFlagsSx127x();
    }
  }

  // pre-flight continuity alarm beep
  if (events.preLiftoff && settings.reportStyle != 'M' && beepAlarm && !beep && fltTime.tmClock - timeLastBeepEnd > beepDelay)
  {
    digitalWrite(pins.beep, HIGH);
    beep = true;
    beepLen = ALARM_BEEP_LEN;
    beepDelay = ALARM_BEEP_DELAY;
    timeBeepStart = fltTime.tmClock;
    beepCounter++;
    // if the counter reaches the beepCode, then switch to error pyro channel
    if (beepCounter == beepCode)
    {
      beepCounter = 0;
      // cycle to the error pyro channel
      beepCont = true;
      beepAlarm = false;
      beepCode = cont.beepCode;
      beepDelay = LONG_BEEP_DELAY;
    }
  }

  // pre-flight continuity beep
  if (events.preLiftoff && beepCont && !beep && fltTime.tmClock - timeLastBeepEnd > beepDelay)
  {
    digitalWrite(pins.beep, HIGH);
    beep = true;
    beepLen = MED_BEEP_LEN;
    beepDelay = SHORT_BEEP_DELAY;
    timeBeepStart = fltTime.tmClock;
    beepCounter++;
    // if the counter reaches the beepCode, then pause for a long delay
    if (beepCounter == beepCode)
    {
      beepCounter = 0;
      beepDelay = LONG_BEEP_DELAY;
      // cycle to the alarm if there is a continuity error
      if (cont.error && settings.reportStyle != 'M')
      {
        beepAlarm = true;
        beepCont = false;
        beepCode = 50;
        beepDelay = LONG_BEEP_DELAY;
      }
    }
  }

  // inflight live-pyro activation Warning
  if (livePyroAlarm && !beep && fltTime.tmClock - timeLastBeepEnd > beepDelay)
  {
    digitalWrite(pins.beep, HIGH);
    beepCode = 50;
    beep = true;
    beepLen = ALARM_BEEP_LEN;
    beepDelay = ALARM_BEEP_DELAY;
    timeBeepStart = fltTime.tmClock;
    beepCounter++;
    // if the counter reaches the beepCode, then give a long delay between beeps
    if (beepCounter == beepCode)
    {
      beepCounter = 0;
      beepDelay = 3000000;
    }
  }

  // post-flight max velocity beeping
  if (fileClose && beepVel && !beep && fltTime.tmClock - timeLastBeepEnd > beepDelay)
  {
    digitalWrite(pins.beep, HIGH);
    beepCode = maxVelDigits[beepPosn - 1];
    beep = true;
    beepLen = MED_BEEP_LEN;
    beepDelay = SHORT_BEEP_DELAY;
    timeBeepStart = fltTime.tmClock;
    beepCounter++;
    // if the beep has hit the current digit, then move to the next digit
    if (beepCounter == beepCode)
    {
      beepCounter = 0;
      beepDelay = LONG_BEEP_DELAY;
      beepPosn--;
      // if we are at the end of the velocity array, switch to the altitude array
      if (beepPosn == 0)
      {
        beepVel = false;
        beepAlt = true;
        beepPosn = altDigits;
        beepDelay = 3000000UL;
      }
    }
  }

  // post-flight max altitude beeping
  if (fileClose && beepAlt && !beep && fltTime.tmClock - timeLastBeepEnd > beepDelay)
  {
    digitalWrite(pins.beep, HIGH);
    beepCode = maxAltDigits[beepPosn - 1];
    beep = true;
    beepLen = MED_BEEP_LEN;
    beepDelay = SHORT_BEEP_DELAY;
    timeBeepStart = fltTime.tmClock;
    beepCounter++;
    // if the beep has hit the current digit, then move to the next digit
    if (beepCounter == beepCode)
    {
      beepCounter = 0;
      beepDelay = LONG_BEEP_DELAY;
      beepPosn--;
      // if we are at the end of the altitude array, switch to the velocity array
      if (beepPosn == 0)
      {
        beepAlt = false;
        beepVel = true;
        beepPosn = velDigits;
        beepDelay = 3000000UL;
      }
    }
  }

  // Code to stop the beep
  if (beep && (fltTime.tmClock - timeBeepStart > beepLen))
  {
    digitalWrite(pins.beep, LOW);
    beep = false;
    timeBeepStart = 0UL;
    timeLastBeepEnd = fltTime.tmClock;
  }

  // GPS Code
  // restore defaults if desired
  if (restoreDefaults)
  {
    Serial.println("restoring GNSS Default Settings...");
    gnssRestoreDefaults();
    restoreDefaults = false;
  }
  // 5 seconds after touchdown put GPS into Power Save Mode (PSM)
  if (!gnss.configPwrSave && (events.touchdown || events.timeOut) && micros() - fltTime.touchdown > 5000000UL)
  {
    gnssPowerSave();
    gnss.configPwrSave = true;
  }
  // Read from serial (no port when the configured bus is not on this board)
  if (gnssSerial != nullptr && gnssSerial->available() > 0)
  {
    gnss.msgRx = true;
  }
  while (gnssSerial != nullptr && gnssSerial->available() > 0)
  {
    char c = gnssSerial->read();
    gps.encode(c);
    if (settings.gpsLog && !fileClose)
    {
      updateGpsLogSd(c);
    }
    if (settings.testMode && gpsEcho && !events.liftoff)
    {
      gnss.serialBuffer[gnss.serialPosn] = c;
      gnss.serialPosn++;
      // print NMEA strings to the serial buffer if needed
      if (settings.testMode && gpsEcho && !events.liftoff && gnss.msgRx && (gnss.serialPosn >= 55))
      {
        gnss.serialBuffer[gnss.serialPosn] = '\0';
        Serial.print(gnss.serialBuffer);
        gnss.serialPosn = 0;
        gnss.msgRx = false;
      }
    }
  }
  // update the GNSS values
  radioTelemetry.satNum = (uint8_t)gps.satellites.value();
  if (micros() - gnss.timeLastFix > 2000000UL && !events.postFlight)
  {
    gnss.fix = 0;
  }
  // process new GNSS position update
  if (gps.location.isUpdated() || gps.altitude.isUpdated())
  {
    gnss.timeLastFix = micros();
    gnss.fixCount++;
    gnss.fix = 1;
    gnss.sdWrite = true;
    gnss.alt = (gps.altitude.meters() - baro.baseAlt);
    radioTelemetry.gpsAlt = (int16_t)gnss.alt;
    // capture max GPS alt
    if (radioTelemetry.gpsAlt > gnss.maxAlt)
    {
      gnss.maxAlt = radioTelemetry.gpsAlt;
    }
    gnss.latitude = gps.location.lat();
    gnss.longitude = gps.location.lng();
    // configure the GPS if the fix is stable
    if (!gnss.configFlight && gnss.fixCount > 40)
    {
      gnssConfig();
      gnss.fix = 0;
      gnss.configFlight = true;
      gnss.configDefaults = false;
    }
    // capture GPS vertical descent velocity with a moving average of 5 samples
    gnss.altPosn++;
    if (gnss.altPosn >= (uint8_t)(sizeof(gnss.altBuff) / sizeof(gnss.altBuff[0])))
    {
      gnss.altPosn = 0;
    }
    gnss.vel = (gps.altitude.meters() - gnss.altBuff[gnss.altPosn]) / ((micros() - gnss.timeBuff[gnss.altPosn]) * MILLIONTH);
    gnss.altBuff[gnss.altPosn] = gps.altitude.meters();
    gnss.timeBuff[gnss.altPosn] = micros();
    // update sensor fusion velocity if descending
    if (events.apogeeFire)
    {
      fusionVel = 0.9 * gnss.vel + 0.1 * baro.vel;
    }
    // capture the GPS takeoff position and correct base altitude
    if (events.preLiftoff)
    {
      if (gps.altitude.meters() != 0)
      {
        // Correct sea level pressure with running average of 5 samples
        // GPS altitude running average
        gnss.bufferPosn++;
        if (gnss.bufferPosn >= (byte)(sizeof(gnss.avgAlt) / sizeof(gnss.avgAlt[0])))
        {
          gnss.bufferPosn = 0;
          gnss.bufferFull = true;
        }
        gnss.altSum = gnss.altSum + gps.altitude.meters() - gnss.avgAlt[gnss.bufferPosn];
        gnss.avgAlt[gnss.bufferPosn] = gps.altitude.meters();
        gnss.baseAlt = gnss.altSum / (float)(sizeof(gnss.avgAlt) / sizeof(gnss.avgAlt[0]));
        // barometric pressure running average
        pressurePosn++;
        if (pressurePosn >= (byte)(sizeof(pressureAvg5) / sizeof(pressureAvg5[0])))
        {
          pressurePosn = 0;
        }
        pressureSum = pressureSum + baro.pressure - pressureAvg5[pressurePosn];
        pressureAvg5[pressurePosn] = baro.pressure;
        pressureAvg = pressureSum / (float)(sizeof(pressureAvg5) / sizeof(pressureAvg5[0]));
        // sea level correction
        if (gnss.bufferFull)
        {
          baro.seaLevelPressure = pressureAvg / powf((44330 - gnss.baseAlt) / 44330, 5.254861);
        }
      }
      gnss.liftoff.latitude = gps.location.lat();
      gnss.liftoff.longitude = gps.location.lng();
      gnss.liftoff.year = gps.date.year();
      gnss.liftoff.month = gps.date.month();
      gnss.liftoff.day = gps.date.day();
    }
    // capture the last GPS position
    if (events.mainDeploy || !events.touchdown || !events.timeOut)
    {
      gnss.touchdown.latitude = gps.location.lat();
      gnss.touchdown.longitude = gps.location.lng();
      gnss.touchdown.alt = gps.altitude.meters();
    }
  }

} // end void main loop

void parseBeep(uint32_t value, uint8_t array[], uint8_t arrayLen)
{
  bool flag = false;
  for (uint8_t i = arrayLen; i > 0; i--)
  {
    array[i - 1] = uint8_t((float)value / powf(10, i - 1));
    value -= array[i - 1] * powf(10, i - 1);
    if (!flag && array[i - 1] > 0)
    {
      flag = true;
    }
    if (flag && array[i - 1] == 0)
    {
      array[i - 1] = 10;
    }
  }
} // end void

void clearIrq() { clearTx = true; }

void timerBuildPkt() { buildPkt = true; }

void timerSyncPkt() { syncFlag = true; }