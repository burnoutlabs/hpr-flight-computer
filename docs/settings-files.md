# Settings files on the SD card

The flight computer reads two text files from the root of the SD card:

- `EEPROMsettings.txt`: the board's hardware (device IDs, buses, orientation, pins, call sign,
  hardware ID). Read once by `parseEepromSettingsSd()`, copied into EEPROM, then deleted from the
  card. Sample: [`firmware/flightcomp/src/EEPROMsettings.txt`](../firmware/flightcomp/src/EEPROMsettings.txt).
- `Settings.txt`: the flight settings. Read on every start-up by `readFlightSettingsSd()`.
  Sample: [`firmware/flightcomp/src/Settings.txt`](../firmware/flightcomp/src/Settings.txt).

Source: `firmware/flightcomp/src/SD.cpp` (`parseNextVariable()`, `parseEepromSettingsSd()`,
`readFlightSettingsSd()`). The host test `firmware/flightcomp/test/test_settings_parser` reads both
sample files through the real parser and checks every value they give.

## How a value is read

Both files are read **by position**. The key names are never compared, so a value's meaning is
set by how many values come before it. `parseNextVariable()`, once per value:

1. reads characters until it reads an `=`. Lines with no `=` (headings, comments) are skipped
   this way;
2. reads and discards the one character after the `=`. In `key = value` that is the space, so
   `key=value` loses the first character of the value;
3. takes every character after that, up to the next `\n` or `\r`, as the value. A `\r\n` line end
   works: the `\r` ends the value and the `\n` is skipped by the next search for `=`.

Consequences worth knowing:

- Do not add, remove or reorder lines that contain an `=` above the last value. Lines below the
  last value (the legends at the end of both samples) are never read.
- A line `key =` followed directly by the line end gives no value: the line end is the
  discarded character, nothing is copied, and the value read is the previous one's text.
- The value is copied with an 8-bit index, so in a value longer than 256 characters the
  characters after the 256th overwrite the start.
- Numbers are converted with `atof()`, so text after the number is ignored and text with no
  number reads as 0.
- The parser does not check for the end of the file (issue #25).

## EEPROMsettings.txt

Read in this order. "Byte" is the value converted with `atof()` and cast to a byte. The EEPROM
addresses are in [eeprom-map.md](eeprom-map.md).

| # | Sample line | Stored as |
| - | ----------- | --------- |
| 1-8 | `accel`, `mag`, `gyro`, `highG`, `baro`, `radio`, `GPS`, `SD` | device ID, byte (codes below) |
| 9-16 | buses, same device order | 2 characters: the bus letter, and the second character minus `'0'` |
| 17-28 | `accelX`..`accelZ`, `gyroX`..`gyroZ`, `highGx`..`highGz`, `magX`..`magZ` | sensor axis to board axis, 2 characters: sign (`-` stores -1, anything else 1) and axis letter |
| 29-52 | `pyro1cont`, `pyro1fire`, `pyro2cont`, `pyro2fire`, `pyro3cont`, `pyro3fire`, `pyro4cont`, `pyro4fire`, `nullPin`, `beepPin`, `battVoltPin`, `testGnd`, `testRead`, `radioIRQ`, `radioRST`, `radioEN`, `radioCS`, `accelCS`, `gyroCS`, `highG-CS`, `magCS`, `baroCS`, `sdCS` | pin number, byte |
| 53-60 | `servo 1 pin` .. `servo 8 pin` | pin number, byte |
| 61 | `callSign` | the first 6 characters |
| 62 | `calibrateMode` | not stored: sets `magCalibrateMode` for this start-up |
| 63 | `HWid` | 3 characters: generation, revision, unit number |

Note the order of the buses (accel, mag, gyro, highG, baro, radio, GPS, SD) differs from the order
of the chip-select pins. The sample's `SD = IO` is the letter O, so its bus number is stored as
`'O' - '0'` = 31; nothing reads the SD bus number.

### Bus codes

The bus letter is `I` (I2C), `S` (SPI) or `U` (hardware serial), and the digit selects the bus and
pins as listed in the legend at the end of the sample file. `firmware/flightcomp/src/Bus_Mgmt.cpp`
and the drivers in `Device_Drivers.cpp` are what interpret them.

### Device ID codes

Source: `setPolyMorphs()` in `firmware/flightcomp/src/Device_Mgmt.cpp`. An ID with no entry gets
the default in the last column.

| Device | Codes | Any other ID |
| ------ | ----- | ------------ |
| Accelerometer | 1 LSM303, 2 LSM9DS1, 3 LSM6DS33, 4 MPU6050, 5 MPU9250, 6 LSM6DSOX, 7 external library, 8 LSM6DS3TR | external library stub (its begin fails until a library is written in, so start-up stops) |
| Gyro | 0 none, 1 L3GD20H, 7 external library. 2-6 and 8 (the accelerometers with a gyro on the chip) are read with the accelerometer | no separate gyro read |
| High-G accelerometer | 0 none, 1 ADS1115/ADXL377, 2 H3LIS331DL, 3 Teensy ADC/ADXL377, 4 external library | none |
| Magnetometer | 0 none, 1 LSM303, 2 LSM9DS1, 3 LIS3MDL, 4 MPU9250, 5 external library | none |
| Barometer | 1 BMP180, 2 MPL3115A2, 3 BMP280, 4 BMP388, 5 MS5611, 6 MS5607, 7 LPS25H, 8 external library | external library stub |
| Radio | 0 none, 1 SX127x (SX1276/7/8/9), 2 RFD900, 3 external library | none |
| GNSS | 0 none, 1 NEO-M8N, 2 SAM/MAX/NEO-M8Q/W, 3 NEO-M9N (all u-blox), 4 Adafruit Ultimate GPS or external library | none |
| SD | 0 none, 1 present (stored; nothing reads it) | |

Gyro ID 6 also skips the gyro bias calibration on the pad (`calibrateOnPad()`, `Startup.cpp`).
An accelerometer ID of 255 (erased EEPROM) stops start-up with a message asking for
`EEPROMsettings.txt`.

## Settings.txt

Read in this order by `readFlightSettingsSd()`. "Char" is the first character of the value. After
reading, `limitSettings()` (`firmware/flightcomp/src/Startup.cpp`) clamps the values in the last
column.

| # | Sample key | Field | Conversion | Clamped by `limitSettings()` |
| - | ---------- | ----- | ---------- | ---------------------------- |
| 1 | `Rocket Name` | `rocketName[20]` | text, copied up to its end | |
| 2 | `flightProfile` | `fltProfile` | char: `S` single stage, `2` two stage, `A` airstart, `B` booster | |
| 3 | `units` | `units` | char: `M` sets metres for the values below, anything else feet | |
| 4 | `reportStyle` | `reportStyle` | char: `P` Perfectflite, `M` Marsa | |
| 5 | `startupTime` | `setupTime` | seconds × 1000 (ms) | 3000 to 60000 ms |
| 6 | `mainDeployAlt` | `mainDeployAlt` | feet or metres, stored in metres | 30 to 458 m |
| 7 | `apogeeDelay` | `apogeeDelay` | seconds × 10^6 (µs) | at most 5 s |
| 8 | `maxRecordTime` | `rcdTime` | seconds × 10^6 (µs) | at least 300 s |
| 9 | `silentTestMode` | `silentMode` | 0 or 1 | |
| 10 | `magSwitchEnable` | `magSwitchEnable` | 0 or 1 | |
| 11 | `inflightRecovery` | `inflightRecover` | 0 off, 1 without pyros, 2 with pyros | |
| 12 | `GPSlogFile` | `gpsLog` | 0 or 1 | |
| 13 | `pyroFiringTime` | `fireTime` | seconds × 10^6 (µs) | 0.2 to 1.0 s |
| 14-17 | `pyro4function` .. `pyro1function` (4 first) | `pyro4Func` .. `pyro1Func` | char: `M`, `A`, `I`, `B`, `1`, `2`, `N` | |
| 18 | `TXenable` | `txEnable` | 0 or 1 | |
| 19 | `TXpwr` | `txPwr` | byte | 2 to 14 |
| 20 | `TXfreq` | `txFreq` | MHz | below 420 → 421; 450 to 902.3 → 902.2; above 928 → 914.9 |
| 21 | `FHSS` | `fhss` | 0 or 1 | turned off below 900 MHz |
| 22 | `boosterSeparationDelay` | `boosterSeparationDelay` | seconds × 10^6 (µs) | at most 3 s |
| 23 | `sustainerIgnitionDelay` | `sustainerFireDelay` | seconds × 10^6 (µs) | at most 8 s |
| 24 | `1stAirstartEvent` | `airStart1Event` | char: `I` ignition, `B` burnout | |
| 25 | `1stAirStartDelayTime` | `airStart1Delay` | seconds × 10^6 (µs) | at most 2 s |
| 26 | `2ndAirStartEvent` | `airStart2Event` | char | |
| 27 | `2ndAirStartDelayTime` | `airStart2Delay` | seconds × 10^6 (µs) | at most 2 s |
| 28 | `altitudeThreshold` | `altThreshold` | feet or metres, stored in whole metres | at least 91 m |
| 29 | `maxAngleOffVertical` | `maxAngle` | whole degrees × 10 (tenths) | at most 450 (45°) |
| 30 | `stableRotn` | `stableRotn` | 0 or 1 | |
| 31 | `stableVert` | `stableVert` | 0 or 1 | |
| 32 | `FlyBack` | `flyBack` | 0 or 1 | |
| 33 | `serialDebug` | `serialDebug` | 0 to 3 | |

The `units` setting only changes how the values in feet or metres are read (`unitConvert`, 3.2808
for feet). The pyro functions are read in the order 4, 3, 2, 1.
