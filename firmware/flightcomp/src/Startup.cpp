/**
 * @file Startup.cpp
 * @brief setup(), its initialization steps, and the pyro continuity check that setup() and the
 * preflight loop share.
 *
 * @details
 * setup() runs the steps below in order. The ones declared in Startup.h bring up hardware that
 * the in-flight recovery, rapidReset() (InflightRecover.cpp), brings up too; the static ones are
 * setup()'s alone (bench test mode, the preflight reports, the pad calibration).
 *
 * setup() was in flightcomp.cpp beside loop(). It is in its own file so that the host tests can
 * build it against the stand-ins in test/shim and compare it with the code it came from
 * (test_setup_neutral).
 */

#include <EEPROM.h>
#include <PWMServo.h>
#include <string.h>
#include "declarations.h"
#include "board.h"
#include "flightcomp.h"
#include "Bus_Mgmt.h"
#include "Calibration.h"
#include "Device_Mgmt.h"
#include "InflightRecover.h"
#include "Rotation.h"
#include "SD_Card.h"
#include "Startup.h"
#include "Telemetry.h"

/**
 * @brief Read the hardware settings from EEPROM: the pins, the device IDs, bus types and bus numbers,
 * the call sign and the hardware ID.
 */
void readHardwareSettingsEeprom()
{
  // Read pin settings from EEPROM
  Serial.print(F("Reading EEPROM..."));
  pins.accelCs = EEPROM.read(eeprom.accelCsPin);
  pins.gyroCs = EEPROM.read(eeprom.gyroCsPin);
  pins.magCs = EEPROM.read(eeprom.magCsPin);
  pins.highGCs = EEPROM.read(eeprom.highGCsPin);
  pins.sdCs = EEPROM.read(eeprom.sdCsPin);
  pins.baroCs = EEPROM.read(eeprom.baroCsPin);
  pins.pyro1Cont = EEPROM.read(eeprom.pyro1ContPin);
  pins.pyro1Fire = EEPROM.read(eeprom.pyro1FirePin);
  pins.pyro2Cont = EEPROM.read(eeprom.pyro2ContPin);
  pins.pyro2Fire = EEPROM.read(eeprom.pyro2FirePin);
  pins.pyro3Cont = EEPROM.read(eeprom.pyro3ContPin);
  pins.pyro3Fire = EEPROM.read(eeprom.pyro3FirePin);
  pins.pyro4Cont = EEPROM.read(eeprom.pyro4ContPin);
  pins.pyro4Fire = EEPROM.read(eeprom.pyro4FirePin);
  pins.nullCont = EEPROM.read(eeprom.nullPin);
  pins.nullFire = pins.nullCont;
  pins.beep = EEPROM.read(eeprom.beepPin);
  pins.batt = EEPROM.read(eeprom.battReadPin);
  pins.testGnd = EEPROM.read(eeprom.testModeGndPin);
  pins.testRead = EEPROM.read(eeprom.testModeReadPin);
  pins.radioCs = EEPROM.read(eeprom.radioCsPin);
  pins.radioIrq = EEPROM.read(eeprom.radioIrqPin);
  pins.radioRst = EEPROM.read(eeprom.radioRstPin);
  pins.radioEn = EEPROM.read(eeprom.radioEnPin);
  pins.servo1 = EEPROM.read(eeprom.servo1Pin);
  pins.servo2 = EEPROM.read(eeprom.servo2Pin);
  pins.servo3 = EEPROM.read(eeprom.servo3Pin);
  pins.servo4 = EEPROM.read(eeprom.servo4Pin);
  pins.servo5 = EEPROM.read(eeprom.servo5Pin);
  pins.servo6 = EEPROM.read(eeprom.servo6Pin);
  pins.servo7 = EEPROM.read(eeprom.servo7Pin);
  pins.servo8 = EEPROM.read(eeprom.servo8Pin);
  // read sensor settings from EEPROM
  sensors.accel = EEPROM.read(eeprom.accelId);
  sensors.mag = EEPROM.read(eeprom.magId);
  sensors.gyro = EEPROM.read(eeprom.gyroId);
  sensors.highG = EEPROM.read(eeprom.highGId);
  sensors.baro = EEPROM.read(eeprom.baroId);
  sensors.radio = EEPROM.read(eeprom.radioId);
  sensors.gnss = EEPROM.read(eeprom.gpsId);
  sensors.accelBusType = (char)EEPROM.read(eeprom.accelBusType);
  sensors.gyroBusType = (char)EEPROM.read(eeprom.gyroBusType);
  sensors.magBusType = (char)EEPROM.read(eeprom.magBusType);
  sensors.highGBusType = (char)EEPROM.read(eeprom.highGBusType);
  sensors.baroBusType = (char)EEPROM.read(eeprom.baroBusType);
  sensors.sdBusType = (char)EEPROM.read(eeprom.sdBusType);
  sensors.gnssBusType = (char)EEPROM.read(eeprom.gpsBusType);
  sensors.radioBusType = (char)EEPROM.read(eeprom.radioBusType);
  sensors.accelBusNum = EEPROM.read(eeprom.accelBusNum);
  sensors.gyroBusNum = EEPROM.read(eeprom.gyroBusNum);
  sensors.magBusNum = EEPROM.read(eeprom.magBusNum);
  sensors.highGBusNum = EEPROM.read(eeprom.highGBusNum);
  sensors.baroBusNum = EEPROM.read(eeprom.baroBusNum);
  sensors.sdBusNum = EEPROM.read(eeprom.sdBusNum);
  sensors.gnssBusNum = EEPROM.read(eeprom.gpsBusNum);
  sensors.radioBusNum = EEPROM.read(eeprom.radioBusNum);
  for (uint8_t i = 0; i < sizeof(settings.callSign); i++)
  {
    settings.callSign[i] = EEPROM.read(eeprom.callSign + i);
  }
  settings.callSign[sizeof(settings.callSign) - 1] = '\0';
  settings.hwId[0] = (char)EEPROM.read(eeprom.hwVersion);
  settings.hwId[1] = '.';
  settings.hwId[2] = (char)EEPROM.read(eeprom.hwSubVersion);
  settings.hwId[3] = '.';
  settings.hwId[4] = (char)EEPROM.read(eeprom.hwUnitNum);
  settings.hwId[5] = '\0';
  Serial.println(F("complete!"));
}

/**
 * @brief Set the pin modes, deselect the radio and high-G accelerometer, and hold the pyro firing pins
 * low.
 */
void setPinModes()
{
  // Set the mode of the output pins
  pinMode(pins.nullCont, INPUT);
  pinMode(pins.nullFire, INPUT);
  pinMode(pins.pyro1Cont, INPUT);
  pinMode(pins.pyro2Cont, INPUT);
  pinMode(pins.pyro3Cont, INPUT);
  pinMode(pins.pyro4Cont, INPUT);
  pinMode(pins.pyro1Fire, OUTPUT);
  pinMode(pins.pyro2Fire, OUTPUT);
  pinMode(pins.pyro3Fire, OUTPUT);
  pinMode(pins.pyro4Fire, OUTPUT);
  pinMode(pins.beep, OUTPUT);
  pinMode(pins.testRead, INPUT_PULLUP);
  pinMode(pins.testGnd, OUTPUT);
  pinMode(pins.radioCs, OUTPUT);
  pinMode(pins.highGCs, OUTPUT);
  // Set the CS pins to HIGH
  digitalWrite(pins.radioCs, HIGH);
  digitalWrite(pins.highGCs, HIGH);
  // Set the pyro firing pins to LOW for safety
  digitalWrite(pins.pyro1Fire, LOW);
  digitalWrite(pins.pyro2Fire, LOW);
  digitalWrite(pins.pyro3Fire, LOW);
  digitalWrite(pins.pyro4Fire, LOW);
}

/**
 * @brief Select the GNSS serial port and start it at the receiver's baud rate.
 *
 * A bus number the board has no serial port for leaves no port: the GNSS receiver is then
 * treated as absent (sensors.gnss 0), so setPolyMorphs() gives it the dummy functions and
 * loop() does not read the port.
 */
void beginGnssSerial()
{
  // Start Harware Serial communication
  setGnssSerial();
  if (gnssSerial == nullptr)
  {
    sensors.gnss = 0;
    Serial.println("GNSS serial port not available, GNSS disabled");
    return;
  }
  if (sensors.gnss == 3)
  {
    gnssSerial->begin(38400);
    Serial.println("Starting HWSerial at 38400 baud");
  }
  else
  {
    gnssSerial->begin(9600);
    Serial.println("Starting HWSerial at 9600 baud");
  }
}

/**
 * @brief Set the ADC resolution for the battery and the analog high-G accelerometer.
 */
void setAnalogResolution()
{
// setup the ADC for sampling the battery and ADXL377 if present
#if defined(__MK66FX1M0__) || defined(__MK64FX512__) || defined(__MK20DX256__)
  // 16 bit for Teensy3.5 and 3.6
  analogReadResolution(16);
#elif defined(__IMXRT1062__)
  // 12 bit resolution for Teensy 4.0 & 4.1
  analogReadResolution(12);
  adcConvert = 1.0F / 4096.0F;
  adcMidValue = 2048;
#endif
}

/**
 * @brief Point the device functions at the configured drivers (setPolyMorphs()) and start the sensors.
 */
void beginSensors()
{
  // set the function pointers
  setPolyMorphs();

  // Start Sensors
  Serial.println(F("Starting Sensors..."));
  sensors.statusAccel = beginAccel();
  sensors.statusGyro = beginGyro();
  sensors.statusMag = beginMag();
  sensors.statusHighG = beginHighG();
  sensors.statusBaro = beginBaro();
}

/**
 * @brief Set the accelerometer 1 g counts and the liftoff trigger, and convert the accelerometer gains
 * to m/s^2.
 */
void scaleSensorGains()
{
  // set Accelerometer G level
  accel1G = (int16_t)(1.0 / accel.gainX);
  accel.gainX *= 9.80665;
  accel.gainY *= 9.80665;
  accel.gainZ *= 9.80665;

  // set the accelerometer liftoff detection g-trigger
  gTrigger = 2.5 * accel1G;

  // set high-G accelerometer level, if there is one (its gain is 0 without one)
  if (sensors.highG != 0)
  {
    high1G = (int16_t)(1.0 / highG.gainX);
    highG.gainX *= 9.80665;
    highG.gainY *= 9.80665;
    highG.gainZ *= 9.80665;
  }
}

/**
 * @brief Power and start the radio, and set up telemetry if it is enabled.
 */
void beginRadioLink()
{
  // Initialize the radio
  // if the Adafruit RFM9XW board is used, make sure its on
  if (pins.radioEn != pins.nullCont)
  {
    pinMode(pins.radioEn, OUTPUT);
    digitalWrite(pins.radioEn, HIGH);
  }
  // start the radio
  sensors.statusRadio = beginRadio();
  if (settings.txEnable)
  {
    beginTelemetry();
    if (!sensors.statusRadio)
    {
      Serial.println(F("Radio Not Found!"));
    }
  }
  else
  {
    Serial.println("Radio Disabled!");
  }
}

/**
 * @brief Hold the user settings to their safe limits.
 */
void limitSettings()
{
  // safety override of user settings
  if (settings.apogeeDelay > 5000000UL)
  {
    settings.apogeeDelay = 5000000UL;
  } // 5s max apogee delay
  if (settings.fireTime > 1000000UL)
  {
    settings.fireTime = 1000000UL;
  } // 1s max firing length
  if (settings.mainDeployAlt > 458)
  {
    settings.mainDeployAlt = 458;
  } // max of 1500 ft
  if (settings.mainDeployAlt < 30)
  {
    settings.mainDeployAlt = 30;
  } // minimum of 100ft
  if (settings.sustainerFireDelay > 8000000UL)
  {
    settings.sustainerFireDelay = 8000000UL;
  } // maximum 8s 2nd stage ignition delay
  if (settings.boosterSeparationDelay > 3000000UL)
  {
    settings.boosterSeparationDelay = 3000000UL;
  } // max 3s booster separation delay after burnout
  if (settings.airStart1Delay > 2000000UL)
  {
    settings.airStart1Delay = 2000000UL;
  } // max 2s airstart delay
  if (settings.airStart2Delay > 2000000UL)
  {
    settings.airStart2Delay = 2000000UL;
  } // max 2s airstart delay
  if (settings.altThreshold < 91)
  {
    settings.altThreshold = 91;
  } // minimum 100ft threshold
  if (settings.maxAngle > 450)
  {
    settings.maxAngle = 450;
  } // maximum 45 degree off vertical
  if (settings.rcdTime < 300000000UL)
  {
    settings.rcdTime = 300000000UL;
  } // min 5min of recording time
  if (settings.fireTime < 200000UL)
  {
    settings.fireTime = 200000UL;
  } // min 0.2s of firing time
  if (settings.fireTime > 1000000UL)
  {
    settings.fireTime = 1000000UL;
  } // max 1.0s of firing time
  if (settings.setupTime > 60000UL)
  {
    settings.setupTime = 60000UL;
  } // max 60 seconds from power-on to preflight start
  if (settings.setupTime < 3000UL)
  {
    settings.setupTime = 3000UL;
  } // min 3 seconds of setup time
  if (settings.txPwr > 14)
  {
    settings.txPwr = 14;
  } // power cant exceed 14dbm on these units wher the RX/TX antenna pins are shared
  if (settings.txPwr < 2)
  {
    settings.txPwr = 2;
  } // power setting can't go below 2
  if (settings.fhss && settings.txFreq < 900.000F)
  {
    settings.fhss = false;
  } // FHSS not used on 70cm band
  if (settings.fltProfile == '2' or settings.fltProfile == 'A')
  {
    boosterBurpTime = min(1000000UL, settings.boosterSeparationDelay - 10000UL);
  }
  if (settings.txFreq < 420.000F)
  {
    settings.txFreq = 421.000F;
  } // cannot be below the 70cm band, reset to 421MHz band
  if (settings.txFreq > 450.000F && settings.txFreq < 902.300F)
  {
    settings.txFreq = 902.200F;
  } // cannot be above the 70cm ham band and below 915MHz ISM band
  if (settings.txFreq > 928.000F)
  {
    settings.txFreq = 914.900F;
  } // cannot be above 915MHz ISM band
}

/**
 * @brief Store the in-flight recovery setting in EEPROM, where setup() reads it on the next start
 * before the SD card is started.
 *
 * @details The rest of the flight settings are no longer stored: their EEPROM copy overlapped
 * (docs/eeprom-map.md) and nothing read it.
 */
static void writeInflightRecoverEeprom() { EEPROM.update(eeprom.inflightRecover, settings.inflightRecover); }

/**
 * @brief Give each pyro channel its function and pins, not firing.
 */
void configurePyroChannels()
{
  const char funcs[4] = {settings.pyro1Func, settings.pyro2Func, settings.pyro3Func, settings.pyro4Func};
  const uint8_t contPins[4] = {pins.pyro1Cont, pins.pyro2Cont, pins.pyro3Cont, pins.pyro4Cont};
  const uint8_t firePins[4] = {pins.pyro1Fire, pins.pyro2Fire, pins.pyro3Fire, pins.pyro4Fire};
  for (uint8_t i = 0; i < 4; i++)
  {
    pyros[i].func = funcs[i];
    pyros[i].contPin = contPins[i];
    pyros[i].firePin = firePins[i];
    pyros[i].fireStatus = false;
    pyros[i].fireStart = 0UL;
  }
}

/**
 * @brief Set the radio's frequency and power (lowest power on a dedicated 915 MHz channel), or with
 * telemetry off, power the radio down or put it to sleep.
 */
void configureRadio()
{
  // setup the radio, default 70cm band
  if (settings.txEnable)
  {
    // 915MHz FHSS
    if (settings.fhss)
    {
      if (settings.testMode)
      {
        Serial.println("FHSS Active!");
      }
    }
    // 915MHz dedicated frequency (no FHSS)
    if (settings.txFreq > 900.000F && !settings.fhss)
    {
      settings.txPwr = 2; // minimum power due to FCC regulations
      if (settings.testMode)
      {
        Serial.println(F("Dedicated ISM band frequency. Power limit 2mW"));
      }
    }
    // Set the radio output power & frequency
    setRadioFreq(settings.txFreq);
    setRadioPwr(settings.txPwr); // 23 max setting; 20mW=13dBm, 30mW=15dBm, 50mW=17dBm, 100mW=20dBm
    if (settings.testMode)
    {
      Serial.print("Radio Freq: ");
      Serial.println(settings.txFreq, 3);
      Serial.print("Radio Power: ");
      Serial.println(settings.txPwr);
    }
  }
  // if present, disconnect power or send the radio to sleep mode
  else
  {
    if (pins.radioEn != pins.nullCont)
    {
      pinMode(pins.radioEn, OUTPUT);
      digitalWrite(pins.radioEn, LOW);
    }
    if (sensors.radio != 0)
    {
      radioSleep();
      digitalWrite(pins.radioCs, HIGH);
    }
    if (settings.testMode)
    {
      Serial.println(F("Telemetry OFF!"));
    }
  }
}

/**
 * @brief Attach the four canard servos and read their trims from EEPROM.
 */
void attachServos()
{
  const uint8_t servoPins[4] = {pins.servo1, pins.servo2, pins.servo3, pins.servo4};
  for (uint8_t i = 0; i < 4; i++)
  {
    servos[i].attach(servoPins[i]);
  }
  // read the trim settings from EEPROM (servo 1's at eeprom.servo1Trim, the rest after it)
  for (uint8_t i = 0; i < 4; i++)
  {
    servoTrim[i] = (int8_t)EEPROM.read(eeprom.servo1Trim + i);
  }
}

/**
 * @brief Drive the servo pins low so that stray voltages cannot move a servo.
 */
static void disableServos()
{
  pinMode(pins.servo3, OUTPUT);
  pinMode(pins.servo4, OUTPUT);
  pinMode(pins.servo5, OUTPUT);
  pinMode(pins.servo6, OUTPUT);
  pinMode(pins.servo7, OUTPUT);
  pinMode(pins.servo8, OUTPUT);

  digitalWrite(pins.servo1, LOW);
  digitalWrite(pins.servo2, LOW);
  digitalWrite(pins.servo3, LOW);
  digitalWrite(pins.servo4, LOW);
  digitalWrite(pins.servo5, LOW);
  digitalWrite(pins.servo6, LOW);
  digitalWrite(pins.servo7, LOW);
  digitalWrite(pins.servo8, LOW);
}

/**
 * @brief Read the sensor biases and barometer offsets from EEPROM, zeroing any that were never
 * written.
 */
void readCalibrationEeprom()
{
  // read the bias from EEPROM
  calUnion.calByte[0] = EEPROM.read(eeprom.accelBiasX);
  calUnion.calByte[1] = EEPROM.read(eeprom.accelBiasX + 1);
  accel.biasX = calUnion.calValue;
  calUnion.calByte[0] = EEPROM.read(eeprom.accelBiasY);
  calUnion.calByte[1] = EEPROM.read(eeprom.accelBiasY + 1);
  accel.biasY = calUnion.calValue;
  calUnion.calByte[0] = EEPROM.read(eeprom.accelBiasZ);
  calUnion.calByte[1] = EEPROM.read(eeprom.accelBiasZ + 1);
  accel.biasZ = calUnion.calValue;
  calUnion.calByte[0] = EEPROM.read(eeprom.gyroBiasX);
  calUnion.calByte[1] = EEPROM.read(eeprom.gyroBiasX + 1);
  gyro.biasX = calUnion.calValue;
  calUnion.calByte[0] = EEPROM.read(eeprom.gyroBiasY);
  calUnion.calByte[1] = EEPROM.read(eeprom.gyroBiasY + 1);
  gyro.biasY = calUnion.calValue;
  calUnion.calByte[0] = EEPROM.read(eeprom.gyroBiasZ);
  calUnion.calByte[1] = EEPROM.read(eeprom.gyroBiasZ + 1);
  gyro.biasZ = calUnion.calValue;
  calUnion.calByte[0] = EEPROM.read(eeprom.highGBiasX);
  calUnion.calByte[1] = EEPROM.read(eeprom.highGBiasX + 1);
  highG.biasX = calUnion.calValue;
  calUnion.calByte[0] = EEPROM.read(eeprom.highGBiasY);
  calUnion.calByte[1] = EEPROM.read(eeprom.highGBiasY + 1);
  highG.biasY = calUnion.calValue;
  calUnion.calByte[0] = EEPROM.read(eeprom.highGBiasZ);
  calUnion.calByte[1] = EEPROM.read(eeprom.highGBiasZ + 1);
  highG.biasZ = calUnion.calValue;
  calUnion.calByte[0] = EEPROM.read(eeprom.magBiasX);
  calUnion.calByte[1] = EEPROM.read(eeprom.magBiasX + 1);
  mag.biasX = calUnion.calValue;
  calUnion.calByte[0] = EEPROM.read(eeprom.magBiasY);
  calUnion.calByte[1] = EEPROM.read(eeprom.magBiasY + 1);
  mag.biasY = calUnion.calValue;
  calUnion.calByte[0] = EEPROM.read(eeprom.magBiasZ);
  calUnion.calByte[1] = EEPROM.read(eeprom.magBiasZ + 1);
  mag.biasZ = calUnion.calValue;
  for (byte i = 0; i < 4; i++)
  {
    floatUnion.bytes[i] = (byte)EEPROM.read(eeprom.baroPressOffset + i);
  }
  baro.pressOffset = floatUnion.val;
  for (byte i = 0; i < 4; i++)
  {
    floatUnion.bytes[i] = (byte)EEPROM.read(eeprom.baroTempOffset + i);
  }
  baro.tempOffset = floatUnion.val;

  // If the barometric pressure sensor calibration values have never been written, then set the offsets to zero
  byte unusedBytes = 0;
  for (byte i = 0; i < 4; i++)
  {
    if ((byte)EEPROM.read(eeprom.baroPressOffset + i) == (byte)255)
    {
      unusedBytes++;
    }
  }
  if (unusedBytes >= 4)
  {
    if (settings.testMode)
    {
      Serial.println(F("Warning: Barometer is not calibrated!"));
    }
    baro.pressOffset = 0.0F;
    baro.tempOffset = 0.0F;
  }

  // If the accelerometer calibration values have never been written, then set the offsets to zero
  unusedBytes = 0;
  for (byte i = 0; i < 2; i++)
  {
    if ((byte)EEPROM.read(eeprom.accelBiasX + i) == (byte)255)
    {
      unusedBytes++;
    }
  }
  if (unusedBytes >= 2)
  {
    if (settings.testMode)
    {
      Serial.println(F("Warning: Accelerometer is not calibrated!"));
    }
    accel.biasX = accel.biasY = accel.biasZ = 0;
    accel.dirX = accel.dirY = accel.dirZ = gyro.dirX = gyro.dirY = gyro.dirZ = 1;
    accel.orientX = 'X';
    accel.orientY = 'Y';
    accel.orientZ = 'Z';
    gyro.orientX = 'X';
    gyro.orientY = 'Y';
    gyro.orientZ = 'Z';
  }

  // If the High-G accelerometer calibration values have never been written, then set the offsets to zero
  unusedBytes = 0;
  for (byte i = 0; i < 2; i++)
  {
    if ((byte)EEPROM.read(eeprom.accelBiasX + i) == (byte)255)
    {
      unusedBytes++;
    }
  }
  if (unusedBytes >= 2 && sensors.highG != 0)
  {
    if (settings.testMode)
    {
      Serial.println(F("Warning: High-G Accelerometer is not calibrated!"));
    }
    highG.biasX = highG.biasY = highG.biasZ = 0;
  }

  // If the magnetometer calibration values have never been written, then set the offsets to zero
  unusedBytes = 0;
  for (byte i = 0; i < 2; i++)
  {
    if ((byte)EEPROM.read(eeprom.magBiasX + i) == (byte)255)
    {
      unusedBytes++;
    }
  }
  if (unusedBytes >= 2)
  {
    if (settings.testMode)
    {
      Serial.println(F("Warning: Magnetometer is not calibrated!"));
    }
    mag.biasX = mag.biasY = mag.biasZ = 0;
  }
}

/**
 * @brief Bench test mode: low radio power and short times and thresholds.
 */
static void applyBenchTestOverrides()
{
  // Overrides for bench test mode
  if (settings.txEnable)
  {
    settings.txPwr = 2;
    setRadioPwr(settings.txPwr); // lowest power setting
    Serial.print(F("Radio Power Reduced for Bench Test Mode: "));
    Serial.println(settings.txPwr);
  }
  fltTime.detectLiftoffTime = 10000UL; // 0.01s
  settings.setupTime = 3000UL;         // 3s startup time
  settings.apogeeDelay = 1000000UL;    // 1s apogee delay
  settings.rcdTime = 15000000UL;       // 15s record time
  if (settings.stableRotn || settings.stableVert)
  {
    settings.rcdTime = 30000000UL;
  } // 30s of record time for serial plotting
  gTrigger = (int)(1.5 * accel1G); // 1.5G trigger
  baro.maxAlt = 11101 / unitConvert;
  maxVelocity = 202 / unitConvert;
  pktInterval.postFlight = 1000000UL;
  thresholdVel = 15.5F;
  // clear the rail halfway through the false-liftoff check, as in flight (0.25 s of 0.5 s), so a
  // bump that ends inside the check is reset
  clearRailTime = 5000UL; // 0.005s
  settings.magSwitchEnable = false;
}

/**
 * @brief Beep the continuity code until the magnet is sensed.
 *
 * The magnet is sensed on the oriented, bias-corrected field (orientMag()), as loop() senses it.
 */
static void waitForMagneticSwitch()
{
  // if the magnetic switch is enabled, beep the continuity code until the magnet is sensed
  delay(250);
  uint8_t ii = 0;
  bool magDetect = false;
  // clear out the FIFO
  for (uint8_t i = 0; i < 10; i++)
  {
    getMag();
    delay(100);
  }
  while (!magDetect)
  {
    ii = 0;
    if (cont.error)
    {
      // signal the continuity error alarm
      for (uint8_t i = 0; i < 20; i++)
      {
        digitalWrite(pins.beep, HIGH);
        delay(12);
        digitalWrite(pins.beep, LOW);
        delay(13);
      }
      delay(250);
    }
    while (!magDetect && ii < beepCode)
    {
      digitalWrite(pins.beep, HIGH);
      delay(250);
      digitalWrite(pins.beep, LOW);
      delay(250);
      getMag();
      orientMag();
      if (abs(mag.x) > magTrigger || abs(mag.y) > magTrigger || abs(mag.z) > magTrigger)
      {
        magDetect = true;
      }
      ii++;
    } // end inner loop
    ii = 0;
    getMag();
    orientMag();
    checkPyroContinuity();
    if (abs(mag.x) > magTrigger || abs(mag.y) > magTrigger || abs(mag.z) > magTrigger)
    {
      magDetect = true;
    }
    while (!magDetect && ii < 10)
    {
      delay(100);
      ii++;
    }
  } // end outer loop
}

/**
 * @brief Wait for the rocket to be held vertical, then sample the sensors for 3 seconds: the gyro
 * biases, the resting values, the high-G bias against the accelerometer, and the initial
 * rotation.
 */
static void calibrateOnPad()
{
  if (settings.testMode)
  {
    Serial.println(F("Hold Rocket Vertical"));
  }
  // wait for the rocket to be installed vertically
  digitalWrite(pins.beep, HIGH);
  delay(settings.setupTime);
  digitalWrite(pins.beep, LOW);
  delay(500);

  if (settings.testMode)
  {
    Serial.println(F("Sampling Sensors"));
  }
  // sample the sensors for 3 seconds to determine the offsets and initial values
  // all gyros must be calibrated at the pad except the LSM6DSOX
  if (sensors.gyro != 6)
  {
    gyro.biasX = 0;
    gyro.biasY = 0;
    gyro.biasZ = 0;
  }
  getHighG(); // clear out the buffer, useful only for the ADXL377 combo
  int16_t accelSamps = 0;
  int16_t gyroSamps = 0;
  int16_t highGsamps = 0;
  int16_t magSamps = 0;
  uint32_t sampTime = 3000000;
  uint32_t calibrationStart = micros();
  bool samplePrint = false;
  while (micros() - calibrationStart < sampTime)
  {
    // accelerometer
    if (micros() - accel.timeLastSamp > accel.timeBtwnSamp)
    {
      getAccel();
      orientAccel();
      accel.timeLastSamp = micros();
      accel.sumX0 += accel.x;
      accel.sumY0 += accel.y;
      accel.sumZ0 += accel.z;
      accelSamps++;
      samplePrint = true;
    }
    // gyroscope
    if (micros() - gyro.timeLastSamp > gyro.timeBtwnSamp)
    {
      getGyro();
      orientGyro();
      gyro.timeLastSamp = micros();
      gyro.sumX0 += gyro.rawX;
      gyro.sumY0 += gyro.rawY;
      gyro.sumZ0 += gyro.rawZ;
      gyroSamps++;
    }
    // high-G accelerometer
    if (sensors.highG != 0 && micros() - highG.timeLastSamp > highG.timeBtwnSamp)
    {
      getHighG();
      orientHighG();
      highG.timeLastSamp = micros();
      highG.sumX0 += highG.x;
      highG.sumY0 += highG.y;
      highG.sumZ0 += highG.z;
      highGsamps++;
    }
    // magnetometer
    if (micros() - mag.timeLastSamp > mag.timeBtwnSamp)
    {
      getMag();
      orientMag();
      mag.timeLastSamp = micros();
      mag.sumX0 += mag.x;
      mag.sumY0 += mag.y;
      mag.sumZ0 += mag.z;
      magSamps++;
    }
    if (settings.testMode && accelSamps % 100 == 0 && samplePrint)
    {
      Serial.print("Gyro: ");
      Serial.print(gyro.rawX);
      Serial.print(',');
      Serial.print(gyro.rawY);
      Serial.print(',');
      Serial.println(gyro.rawZ);
      Serial.print("Accel: ");
      Serial.print(accel.x);
      Serial.print(',');
      Serial.print(accel.y);
      Serial.print(',');
      Serial.println(accel.z);
      Serial.print("HighG: ");
      Serial.print(highG.x);
      Serial.print(',');
      Serial.print(highG.y);
      Serial.print(',');
      Serial.println(highG.z);
      Serial.print("Mag: ");
      Serial.print(mag.x);
      Serial.print(',');
      Serial.print(mag.y);
      Serial.print(',');
      Serial.println(mag.z);
      samplePrint = false;
    }
  } // end sample period

  // Compute the average of the sample period
  Serial.print("gyro samples: ");
  Serial.println(gyroSamps);
  Serial.print("accel samples: ");
  Serial.println(accelSamps);
  Serial.print("highG samples: ");
  Serial.println(highGsamps);
  Serial.print("mag samples: ");
  Serial.println(magSamps);
  // LSM6DSOX is accurate enough to not need calibration
  if (sensors.gyro != 6)
  {
    gyro.biasX = (int)round(gyro.sumX0 / gyroSamps);
    gyro.biasY = (int)round(gyro.sumY0 / gyroSamps);
    gyro.biasZ = (int)round(gyro.sumZ0 / gyroSamps);
  }
  accel.x0 = (int)round(accel.sumX0 / accelSamps);
  accel.y0 = (int)round(accel.sumY0 / accelSamps);
  accel.z0 = (int)round(accel.sumZ0 / accelSamps);
  // no samples without a high-G accelerometer
  if (highGsamps > 0)
  {
    highG.x0 = (int)round(highG.sumX0 / highGsamps);
    highG.y0 = (int)round(highG.sumY0 / highGsamps);
    highG.z0 = (int)round(highG.sumZ0 / highGsamps);
  }
  mag.x0 = (int)round(mag.sumX0 / magSamps);
  mag.y0 = (int)round(mag.sumY0 / magSamps);
  mag.z0 = (int)round(mag.sumZ0 / magSamps);
  // print out the results of the initial calibration sequence
  if (settings.testMode)
  {
    Serial.println(F("Sampling complete"));
    Serial.print(F("Gyro Offsets: "));
    Serial.print(gyro.biasX);
    Serial.print(F(", "));
    Serial.print(gyro.biasY);
    Serial.print(F(", "));
    Serial.println(gyro.biasZ);
    Serial.print(F("Accel X0,Y0,Z0: "));
    Serial.print(accel.x0);
    Serial.print(F(", "));
    Serial.print(accel.y0);
    Serial.print(F(", "));
    Serial.println(accel.z0);
    Serial.print(F("HighG X0,Y0,Z0: "));
    Serial.print(highG.x0);
    Serial.print(F(", "));
    Serial.print(highG.y0);
    Serial.print(F(", "));
    Serial.println(highG.z0);
    Serial.print(F("Mag X0,Y0,Z0: "));
    Serial.print(mag.x0);
    Serial.print(F(", "));
    Serial.print(mag.y0);
    Serial.print(F(", "));
    Serial.println(mag.z0);
  }

  // Calibrate the analog accelerometer to the digital one, if there is a high-G accelerometer
  if (sensors.highG != 0)
  {
    float gainRatio = highG.gainZ / accel.gainZ; //.64599
    if (settings.testMode)
    {
      Serial.println(F("Re-calibrating high-G accelerometer..."));
      Serial.print(F("HighG.Z0: "));
      Serial.println(highG.z0);
    }
    // Z0 + correction = correctZ0
    // correctZ0 = accel.z0/gainRatio
    // correction = correctZ0 - Z0
    // corrected bias = original bias - directionX *(highG.z0 - accel.z0/gainRatio)
    if (highG.orientX == 'Z')
    {
      if (settings.testMode)
      {
        Serial.print("Old Bias: ");
        Serial.println(highG.biasX);
      }
      highG.biasX -= highG.dirX * (highG.z0 - (int)((float)accel.z0 / (float)gainRatio));
      if (settings.testMode)
      {
        Serial.print(F("New Bias: "));
        Serial.println(highG.biasX);
      }
    }
    if (highG.orientY == 'Z')
    {
      if (settings.testMode)
      {
        Serial.print("Old Bias: ");
        Serial.println(highG.biasY);
      }
      highG.biasY -= highG.dirY * (highG.z0 - (int)((float)accel.z0 / (float)gainRatio));
      if (settings.testMode)
      {
        Serial.print(F("New Bias: "));
        Serial.println(highG.biasY);
      }
    }
    if (highG.orientZ == 'Z')
    {
      if (settings.testMode)
      {
        Serial.print("Old Bias: ");
        Serial.println(highG.biasZ);
      }
      highG.biasZ -= highG.dirZ * (highG.z0 - (int)((float)accel.z0 / (float)gainRatio));
      if (settings.testMode)
      {
        Serial.print(F("New Bias: "));
        Serial.println(highG.biasZ);
      }
    }
  }
  // highG.biasX = highGx0 - (int)((float)accel.x0 / (float)gainRatio) - 27;//old formula is kept for reference

  // Compute the acceleromter based rotation angle
  if (accel.y0 >= 0)
  {
    yawY0 = asinf(min(1, (float)accel.y0 / (float)accel1G)) * RAD_TO_DEG;
  }
  else
  {
    yawY0 = asinf(max(-1, (float)accel.y0 / (float)accel1G)) * RAD_TO_DEG;
  }

  if (accel.x0 >= 0)
  {
    pitchX0 = asinf(min(1, (float)accel.x0 / (float)accel1G)) * RAD_TO_DEG;
  }
  else
  {
    pitchX0 = asinf(max(-1, (float)accel.x0 / (float)accel1G)) * RAD_TO_DEG;
  }

  // update quaternion rotation
  getQuatRotn(pitchX0 * 1000000 / gyro.gainZ, yawY0 * 1000000 / gyro.gainZ, 0, gyro.gainZ);
  // use DCM2D if we are deploying control surfaces
  if (settings.stableRotn || settings.stableVert)
  {
    getDcm2dRotn(pitchX0 * 1000000 / gyro.gainZ, yawY0 * 1000000 / gyro.gainZ, 0, gyro.gainZ);
  }
  // Output the initial rotation conditions reltative to the Earth
  if (settings.testMode)
  {
    Serial.println(F("Rotation Computation Complete"));
    Serial.print(F("Yaw: "));
    Serial.println(yawY0, 2);
    Serial.print(F("Pitch: "));
    Serial.println(pitchX0, 2);
    Serial.print(F("Off Vertical: "));
    Serial.println(((float)offVert) * .1, 1);
  }
}

/**
 * @brief Read the battery voltage, from a pyro continuity pin on old hardware.
 */
static void readBatteryVoltage()
{
  // Read the battery voltage
  voltReading = analogRead(pins.batt);
  voltage = ((float)voltReading) * 3.3 * 3.2 * adcConvert;

  // Read the battery voltage on old hardware if present
  if (pins.batt == pins.pyro4Cont)
  {
    if (settings.pyro1Func == 'M')
    {
      pins.batt = pins.pyro1Cont;
    }
    if (settings.pyro2Func == 'M')
    {
      pins.batt = pins.pyro2Cont;
    }
    if (settings.pyro3Func == 'M')
    {
      pins.batt = pins.pyro3Cont;
    }
    voltReading = analogRead(pins.batt);
    voltage = (float)(voltReading) * 3.3 * 2.72 * adcConvert;
  }
}

/**
 * @brief Beep out the main deployment altitude setting.
 */
static void reportMainDeployAlt()
{
  // Read main deploy setting into its beep array
  parseBeep((uint32_t)((float)(settings.mainDeployAlt) * unitConvert), maxVelDigits, 4);
  if (settings.testMode)
  {
    Serial.print(F("Reporting Main Deploy Settings: "));
    Serial.println((uint16_t)((float)(settings.mainDeployAlt) * unitConvert));
  }
  // Beep out the main deployment altitude setting
  while (maxVelDigits[velDigits - 1] == 0)
  {
    velDigits--;
  }
  for (byte i = velDigits; i > 0; i--)
  {
    delay(800);
    for (byte j = maxVelDigits[i - 1]; j > 0; j--)
    {
      digitalWrite(pins.beep, HIGH);
      delay(100);
      digitalWrite(pins.beep, LOW);
      delay(100);
    }
  }
  velDigits = 4;
  delay(2000);
}

/**
 * @brief On a new part, write the initial values of the last flight's EEPROM record.
 */
static void checkFirstUse()
{
  // check for initial MCU programming
  bool initPgm = false;
  for (byte i = 0; i < altDigits; i++)
  {
    if (EEPROM.read(eeprom.maxFltAlt + i) > 10)
    {
      initPgm = true;
    }
  }
  // Write initial values into EEPROM where needed
  if (initPgm)
  {
    if (settings.testMode)
    {
      Serial.println(F("Initial Use Detected: Writing Initial EEPROM values for prior flight"));
    }
    for (byte j = 0; j < 6; j++)
    {
      EEPROM.update(eeprom.maxFltAlt + j, j);
    }
    EEPROM.update(eeprom.totalFlights, 0);
    EEPROM.update(eeprom.lastEvent, 27);
  }
}

/**
 * @brief Beep out the last flight's maximum altitude.
 */
static void reportLastFlight()
{
  // Beep out the last flight's altitude
  if (settings.testMode)
  {
    Serial.print(F("Reporting last flight: "));
  }
  for (uint8_t i = 0; i < 6; i++)
  {
    maxAltDigits[i] = EEPROM.read(eeprom.maxFltAlt + i);
  }
  while (maxAltDigits[altDigits - 1] == 0 && altDigits > 0)
  {
    altDigits--;
  }
  for (uint8_t i = altDigits; i > 0; i--)
  {
    delay(800);
    if (settings.testMode)
    {
      if (maxAltDigits[i - 1] < 10)
      {
        Serial.print(maxAltDigits[i - 1]);
      }
      else
      {
        Serial.print('0');
      }
    }
    for (uint8_t j = maxAltDigits[i - 1]; j > 0; j--)
    {
      digitalWrite(pins.beep, HIGH);
      delay(100);
      digitalWrite(pins.beep, LOW);
      delay(100);
    }
  }
  altDigits = 6;
  delay(2000);
}

/**
 * @brief Beep out the battery voltage.
 */
static void reportBatteryVoltage()
{
  // Beep out the battery voltage
  if (settings.testMode)
  {
    Serial.println(" ");
    Serial.print(F("Reporting Battery Voltage: "));
    Serial.println(voltage, 1);
  }
  parseBeep((uint32_t)(voltage * 10), voltageDigits, 2);
  for (uint8_t i = 0; i < 2; i++)
  {
    delay(800);
    for (uint8_t j = voltageDigits[1 - i]; j > 0; j--)
    {
      digitalWrite(pins.beep, HIGH);
      delay(100);
      digitalWrite(pins.beep, LOW);
      delay(100);
    }
  }
  delay(2000);
}

/**
 * @brief Set the serial debug outputs.
 */
static void setSerialDebug()
{
  // set the Serial debug ouputs
  switch (settings.serialDebug)
  {
  case 0:
    gpsEcho = false;
    radioDebug = false;
    break;
  case 1:
    gpsEcho = true;
    radioDebug = false;
    break;
  case 2:
    gpsEcho = false;
    radioDebug = true;
    break;
  case 3:
    gpsEcho = true;
    radioDebug = true;
    break;
  }
}

/**
 * @brief Initializes the flight computer: hardware and user settings, sensors, radio, pyro
 * channels, calibration, and the preflight reports.
 */
void setup(void)
{

  Serial.begin(38400);

  delay(500);

  // check to see if we are restarting from a power loss in flight: the last recorded flight event
  // is read before anything overwrites it. If the flight can be resumed, rapidReset() brings the
  // system back up and setup() exits straight into the loop.
  const uint8_t lastEvent = EEPROM.read(eeprom.lastEvent);
  settings.inflightRecover = EEPROM.read(eeprom.inflightRecover);
  if (settings.inflightRecover > 0 && isRecoverableEvent(lastEvent))
  {
    // execute the rapid recovery routine
    Serial.print("Rapid Reset Initiated, setting: ");
    Serial.println(settings.inflightRecover);
    if (rapidReset(lastEvent))
    {
      // exit setup and immediately re-enter the loop
      return;
    }
    Serial.println("Not in flight, starting normally");
  }
  if (lastEvent != 27)
  {
    EEPROM.update(eeprom.lastEvent, 27);
  }

  // Start SD card
  beginSd();

  // if an EEPROM settings file exists on the SD card, read it and store it into EEPROM
  parseEepromSettingsSd();

  readHardwareSettingsEeprom();

  // check for unitialized EEPROM
  if (sensors.accel == 255)
  {
    Serial.println("WARNING: SYSTEM HARDWARE SETTINGS ARE UNITIALIZED! SYSTEM CANNOT CONTINUE UNTIL HARDWARE DEVICES, BUSES, AND PIN VALUES ARE SET");
    Serial.println("PLEASE LOAD EEPROMsettings.txt ONTO THE SD CARD ROOT TO PROCEED");
    while (1)
    {
    }
  }

  setPinModes();

  beginGnssSerial();

  // check if the test mode button is being held
  digitalWrite(pins.testGnd, LOW);
  delay(50);
  if (digitalRead(pins.testRead) == LOW)
  {
    settings.testMode = true;
    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F("Bench-Test Mode Confirmed"));
    Serial.println(F("Press the button again to calibrate the accelerometers."));
    Serial.println(F("Enter 'b' into the serial monitor to calibrate the barometer"));
    Serial.println(F("Enter 'c' into the serial monitor to adjust the canard trims"));
    Serial.println(F("------------------------------------------------------------"));
  }

  setAnalogResolution();

  // read the flight settings from the SD card
  readFlightSettingsSd();

  beginSensors();

  // check to see if a critical senor failed the startup test - if so, give an audible warning
  bool sensorsOk = sensors.statusAccel & sensors.statusGyro & sensors.statusHighG & sensors.statusBaro;
  // halt progress and give an audible warning once per second
  if (!sensorsOk)
  {
    Serial.println("Sensor Failure! Cannot proceed to flight");
  }
  while (!sensorsOk)
  {
    for (uint8_t i = 0; i < 20; i++)
    {
      digitalWrite(pins.beep, HIGH);
      delay(12);
      digitalWrite(pins.beep, LOW);
      delay(13);
    }
    delay(1000);
  }

  scaleSensorGains();

  beginRadioLink();

  limitSettings();

  writeInflightRecoverEeprom();

  configurePyroChannels();

  if (settings.testMode)
  {
    Serial.print(F("Flight Profile: "));
    Serial.println(settings.fltProfile);
    Serial.print(F("Pyro 4 Function: "));
    Serial.println(pyros[3].func);
    Serial.print(F("Pyro 3 Function: "));
    Serial.println(pyros[2].func);
    Serial.print(F("Pyro 2 Function: "));
    Serial.println(pyros[1].func);
    Serial.print(F("Pyro 1 Function: "));
    Serial.println(pyros[0].func);
  }

  // check for silent mode
  if (settings.testMode && settings.silentMode)
  {
    pins.beep = pins.nullCont;
    Serial.println(F("Silent Mode Confirmed"));
  }

  configureRadio();

  // setup servos if enabled
  if (settings.stableRotn || settings.stableVert)
  {

    if (settings.testMode)
    {
      Serial.println(F("ACTIVE STABILIZATION ENABLED"));
    }

    // enable serial plotting
    gpsEcho = false;
    radioDebug = false;

    attachServos();
    if (settings.testMode)
    {
      Serial.print(F("Servo1 Trim: "));
      Serial.println(servoTrim[0]);
      Serial.print(F("Servo2 Trim: "));
      Serial.println(servoTrim[1]);
      Serial.print(F("Servo3 Trim: "));
      Serial.println(servoTrim[2]);
      Serial.print(F("Servo4 Trim: "));
      Serial.println(servoTrim[3]);
    }

    // Test canards
    for (uint8_t i = 0; i < 4; i++)
    {
      servos[i].write(120 - servoTrim[i]);
    }
    delay(1000);
    for (uint8_t i = 0; i < 4; i++)
    {
      servos[i].write(60 - servoTrim[i]);
    }
    delay(1000);
    for (uint8_t i = 0; i < 4; i++)
    {
      servos[i].write(90 - servoTrim[i]);
    }
  }

  // otherwise disable the servos and ensure that stray voltages do not cause any attached servos to move
  else
  {
    pinMode(pins.servo1, OUTPUT);
    pinMode(pins.servo2, OUTPUT);
    disableServos();
  }

  // signal if in test-mode
  if (settings.testMode)
  {

    Serial.println(F("Signaling Test Mode"));
    beepCounter = 0;
    beepDelay = LONG_BEEP_DELAY;

    // initiate the 7 beeps to signal test mode
    while (beepCounter < 8)
    {

      fltTime.tmClock = micros();

      // Look for the user to release the button
      if (digitalRead(pins.testRead) == HIGH)
      {
        settings.testMode = false;
        delay(50);
      }

      // Look for the user to put it into accelerometer calibration mode or barometer calibration mode
      if (digitalRead(pins.testRead) == LOW && !settings.testMode)
      {
        delay(50); // necessary because sometimes when the button is released it bounces back
        if (digitalRead(pins.testRead) == LOW)
        {
          settings.calibrationMode = true;
          settings.testMode = true;
          beepCounter = 8;
          digitalWrite(pins.beep, LOW);
          beep = false;
        }
      }

      // Look for the user to enter barometer calibration mode by entering into the serial monitor
      if (Serial.available() > 0)
      {
        userInput = Serial.read();
        if (userInput == 'b')
        {
          baroCalibrate();
        }
        if (userInput == 'c')
        {
          setCanardTrim();
        }
        while (Serial.available() > 0)
        {
          userInput = Serial.read();
        }
      }

      // starts the beep
      if (!beep && fltTime.tmClock - timeLastBeepEnd > beepDelay)
      {
        digitalWrite(pins.beep, HIGH);
        timeBeepStart = fltTime.tmClock;
        beep = true;
        beepCounter++;
      }

      // stops the beep
      if (beep && (fltTime.tmClock - timeBeepStart > 500000UL))
      {
        digitalWrite(pins.beep, LOW);
        timeBeepStart = 0UL;
        timeLastBeepEnd = fltTime.tmClock;
        beep = false;
      }
    } // end while

    // Reset variables
    beepCounter = 0;
    timeBeepStart = 0UL;
    timeLastBeepEnd = 0UL;
    fltTime.tmClock = 0UL;
    if (!settings.calibrationMode)
    {
      settings.testMode = true;
    }
  } // end testMode

  // calibration mode
  if (settings.calibrationMode)
  {
    accelCalibrate();
  } // end calibration mode

  // Calibrate the magnetometer
  if (magCalibrateMode == 1)
  {
    magCalibrate();
  }
  // set the trigger for the magnetic switch
  magTrigger = 3000;

  readCalibrationEeprom();

  // output the read values
  if (settings.testMode)
  {
    Serial.println(F("Calibrataion values read from EEPROM:"));
    Serial.print(F("accel.biasX: "));
    Serial.println(accel.biasX);
    Serial.print(F("accel.biasY: "));
    Serial.println(accel.biasY);
    Serial.print(F("accel.biasZ: "));
    Serial.println(accel.biasZ);
    Serial.print(F("gyro.biasX: "));
    Serial.println(gyro.biasX);
    Serial.print(F("gyro.biasY: "));
    Serial.println(gyro.biasY);
    Serial.print(F("gyro.biasZ: "));
    Serial.println(gyro.biasZ);
    Serial.print(F("highG.biasX: "));
    Serial.println(highG.biasX);
    Serial.print(F("highG.biasY: "));
    Serial.println(highG.biasY);
    Serial.print(F("highG.biasZ: "));
    Serial.println(highG.biasZ);
    Serial.print(F("mag.biasX: "));
    Serial.println(mag.biasX);
    Serial.print(F("mag.biasY: "));
    Serial.println(mag.biasY);
    Serial.print(F("mag.biasZ: "));
    Serial.println(mag.biasZ);
    Serial.print(F("baro.tempOffset: "));
    Serial.println(baro.tempOffset, 1);
    Serial.print(F("baro.pressOffset: "));
    Serial.println(baro.pressOffset, 2);
  }

  // read the orientation variables from EEPROM
  readOrientation();

  // display orientation values from EEPROM
  if (settings.testMode)
  {
    Serial.print(F("accel.X is pointed to real world: "));
    Serial.print((accel.dirX == 1) ? '+' : '-');
    Serial.println(accel.orientX);
    Serial.print(F("accel.Y is pointed to real world: "));
    Serial.print((accel.dirY == 1) ? '+' : '-');
    Serial.println(accel.orientY);
    Serial.print(F("accel.Z is pointed to real world: "));
    Serial.print((accel.dirZ == 1) ? '+' : '-');
    Serial.println(accel.orientZ);
    Serial.print(F("gyro.X is pointed to real world: "));
    Serial.print((gyro.dirX == 1) ? '+' : '-');
    Serial.println(gyro.orientX);
    Serial.print(F("gyro.Y is pointed to real world: "));
    Serial.print((gyro.dirY == 1) ? '+' : '-');
    Serial.println(gyro.orientY);
    Serial.print(F("gyro.Z is pointed to real world: "));
    Serial.print((gyro.dirZ == 1) ? '+' : '-');
    Serial.println(gyro.orientZ);
    Serial.print(F("mag.X is pointed to real world: "));
    Serial.print((mag.dirX == 1) ? '+' : '-');
    Serial.println(mag.orientX);
    Serial.print(F("mag.Y is pointed to real world: "));
    Serial.print((mag.dirY == 1) ? '+' : '-');
    Serial.println(mag.orientY);
    Serial.print(F("mag.Z is pointed to real world: "));
    Serial.print((mag.dirZ == 1) ? '+' : '-');
    Serial.println(mag.orientZ);
    Serial.print(F("highG.X is pointed to real world: "));
    Serial.print((highG.dirX == 1) ? '+' : '-');
    Serial.println(highG.orientX);
    Serial.print(F("highG.Y is pointed to real world: "));
    Serial.print((highG.dirY == 1) ? '+' : '-');
    Serial.println(highG.orientY);
    Serial.print(F("highG.Z is pointed to real world: "));
    Serial.print((highG.dirZ == 1) ? '+' : '-');
    Serial.println(highG.orientZ);
  }

  // Overrides for bench test mode
  if (settings.testMode)
  {
    applyBenchTestOverrides();
  }

  // Create and open the next file on the SD card
  createNextFileSd();

  // check continuity
  checkPyroContinuity();

  // report continuity results
  if (settings.testMode)
  {
    Serial.print("Pyro4 Continuity: ");
    Serial.println((pyros[3].contStatus) ? "Y" : "N");
    Serial.print("Pyro3 Continuity: ");
    Serial.println((pyros[2].contStatus) ? "Y" : "N");
    Serial.print("Pyro2 Continuity: ");
    Serial.println((pyros[1].contStatus) ? "Y" : "N");
    Serial.print("Pyro1 Continuity: ");
    Serial.println((pyros[0].contStatus) ? "Y" : "N");
    Serial.print(F("Reporting continuity code: "));
    Serial.println(cont.beepCode);
  }

  // set the beep delay and preflight beep code
  beepDelay = LONG_BEEP_DELAY;
  beepCode = cont.beepCode;

  // if the magnetic switch is enabled, beep the continuity code until the magnet is sensed
  if (settings.magSwitchEnable)
  {
    waitForMagneticSwitch();
  }

  calibrateOnPad();
  readBatteryVoltage();

  // Read main deploy setting into its beep array
  if (settings.reportStyle == 'P')
  {
    reportMainDeployAlt();
  }
  checkFirstUse();
  if (settings.reportStyle == 'P')
  {
    reportLastFlight();
    reportBatteryVoltage();
  }
  setSerialDebug();

  if (settings.testMode)
  {
    Serial.println(F("Setup Complete. Awaiting simulated launch."));
    Serial.print(F("Beginning Serial Output and Continuity Reporting: "));
    Serial.println(beepCode);
    delay(3000);
  }

  // initialize the radio timing
  radioTimer.begin(timerBuildPkt, pktInterval.preLiftoff);
  if (settings.fltProfile == 'B')
  {
    radioTelemetry.event = BOOSTER_PREFLIGHT;
  }
  Serial.println("End Setup");
} // end setup

/**
 * @brief Note a pyro function whose channel has continuity.
 */
static void markContinuity(char func)
{
  switch (func)
  {
  case 'M':
    cont.main = true;
    break;
  case 'A':
    cont.apogee = true;
    break;
  case 'I':
    cont.upperStage = true;
    break;
  case 'B':
    cont.boosterSep = true;
    break;
  case '1':
    cont.airStart1 = true;
    break;
  case '2':
    cont.airStart2 = true;
    break;
  case 'N':
    cont.noFunc = true;
    break;
  }
}

/**
 * @brief Read each pyro channel's continuity and, before liftoff, set the continuity flags, the
 * error and the preflight beep and report codes.
 */
void checkPyroContinuity()
{

  // check continuity
  for (uint8_t i = 0; i < 4; i++)
  {
    pyros[i].contStatus = ((digitalRead(pyros[i].contPin) == HIGH) ? true : false);
  }

  if (!events.liftoff)
  {
    // note which functions have continuity
    for (uint8_t i = 0; i < 4; i++)
    {
      if (pyros[i].contStatus)
      {
        markContinuity(pyros[i].func);
      }
    }

    // if the flight profile is complex, but there is no continuity on BOTH of the extra pyros, then reset to a single stage flight
    if ((settings.fltProfile == '2' || settings.fltProfile == 'A') && !cont.upperStage && !cont.boosterSep && !cont.airStart1 && !cont.airStart2)
    {
      if (settings.testMode)
      {
        Serial.println(F("Complex Pyros Not Detected! Flight Profile set to single stage"));
      }
      settings.fltProfile = 'S';
      for (uint8_t i = 0; i < 4; i++)
      {
        if (pyros[i].func == 'I' || pyros[i].func == 'B' || pyros[i].func == '1' || pyros[i].func == '2')
        {
          pyros[i].func = 'N';
        }
      }
    }

    // Look for continuity problems
    bool anyContinuity = false;
    for (uint8_t i = 0; i < 4; i++)
    {
      if (!pyros[i].contStatus && pyros[i].func != 'N')
      {
        cont.error = true;
        cont.reportCode = cont.beepCode = i + 1;
      }
      anyContinuity = anyContinuity || pyros[i].contStatus;
    }
    if (!anyContinuity)
    {
      cont.reportCode = 0;
      cont.beepCode = 1;
    }

    // Report single-stage pre-flight status
    if (!cont.error && (settings.fltProfile == 'S' || settings.fltProfile == 'B'))
    {
      if (cont.main && cont.apogee)
      {
        cont.beepCode = 3;
        cont.reportCode = 9;
      }
      else if (cont.main)
      {
        cont.beepCode = 2;
        cont.reportCode = 8;
      }
      else if (cont.apogee)
      {
        cont.beepCode = 1;
        cont.reportCode = 7;
      }
      postFlightCode = 1;
    }

    // Report two-stage pre-flight status
    if (!cont.error && settings.fltProfile == '2')
    {
      if (cont.boosterSep && cont.upperStage && cont.apogee && cont.main)
      {
        cont.beepCode = 4;
        cont.reportCode = 6;
      }
      else if (cont.upperStage && cont.apogee && cont.main)
      {
        cont.beepCode = 3;
        cont.reportCode = 5;
      }
    }

    // Report airstart pre-flight status
    if (!cont.error && settings.fltProfile == 'A')
    {
      if (cont.airStart1 && cont.airStart2 && cont.apogee && cont.main)
      {
        cont.beepCode = 4;
        cont.reportCode = 6;
      }
      else if (cont.airStart1 && cont.apogee && cont.main)
      {
        cont.beepCode = 3;
        cont.reportCode = 5;
      }
    }

    // Change if Marsa reporting style is desired
    if (settings.reportStyle == 'M')
    {
      cont.beepCode = 2;
      if (cont.error)
      {
        cont.beepCode = 1;
      }
    }
  } // end if liftoff
} // end check continuity
