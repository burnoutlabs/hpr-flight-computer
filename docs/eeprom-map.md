# Flight computer EEPROM map

Source: `struct EepromMap` in `firmware/flightcomp/include/declarations.h` (the addresses), and the
reads and writes named in each row. Multi-byte values are stored low byte first (the Teensy is
little-endian and the code copies a union's bytes in order). An erased byte reads 255.

These addresses are a contract with every board already in the field: a board keeps its EEPROM
across firmware updates, so an address cannot move without the stored value moving with it.

## Hardware settings and calibration

| Address | Bytes | Field | Contents | Written by | Read by |
| ------- | ----- | ----- | -------- | ---------- | ------- |
| 0-5 | 6 | `maxFltAlt` | last flight's maximum barometric altitude in the display units, one decimal digit per byte, least significant first; a zero digit inside the number is stored as 10 (`parseBeep()`) | `loop()` at the end of the flight, not in test mode (`flightcomp.cpp`); `checkFirstUse()` writes 0-5 on first use | `checkFirstUse()` (any byte above 10 means first use), `reportLastFlight()` (`Startup.cpp`) |
| 6-11 | 2 each | `accelBiasX/Y/Z` | int16 | `accelCalibrate()` (`Calibration.cpp`) | `readCalibrationEeprom()` (`Startup.cpp`) |
| 12-17 | 2 each | `highGBiasX/Y/Z` | int16 | `accelCalibrate()` | `readCalibrationEeprom()` |
| 18-23 | 2 each | `magBiasX/Y/Z` | int16 | `magCalibrate()` | `readCalibrationEeprom()` |
| 24-47 | 1 each | `accelXSign`, `accelXPtr`, ... `highGZPtr` | sensor to world orientation, per sensor (accel, gyro, mag, high-G) and axis: a sign byte then an axis byte | `setOrientation()` (`Calibration.cpp`) | `readOrientation()` (`Calibration.cpp`) |
| 48-71 | 1 each | `accel2BoardXAxis`, `accel2BoardXSign`, ... `highG2BoardZSign` | sensor to board orientation from `EEPROMsettings.txt`: per sensor (accel, gyro, mag, high-G) and axis, the axis letter then the sign (1, or 255 for -1) | `parseEepromSettingsSd()` (`SD.cpp`) | `setOrientation()` |
| 72-79 | 1 each | `accelBusType`, `accelBusNum`, `gyroBus*`, `magBus*`, `highGBus*` | bus letter, then bus number | `parseEepromSettingsSd()` | `readHardwareSettingsEeprom()` (`Startup.cpp`) |
| 80-83 | 4 | `baroPressOffset` | float, hPa | `baroCalibrate()` (`Calibration.cpp`) | `readCalibrationEeprom()` (all four bytes 255 means never written: offset 0) |
| 84-87 | 4 | `baroTempOffset` | float | `baroCalibrate()` | `readCalibrationEeprom()` |
| 88-95 | 1 each | `baroBus*`, `radioBus*`, `sdBus*`, `gpsBus*` | bus letter, then bus number | `parseEepromSettingsSd()` | `readHardwareSettingsEeprom()` |
| 96-118 | 1 each | `pyro1ContPin` .. `baroCsPin` | pin numbers, in the order of `struct EepromMap` | `parseEepromSettingsSd()` | `readHardwareSettingsEeprom()` |
| 119-126 | 1 each | `servo1Pin` .. `servo8Pin` | pin numbers | `parseEepromSettingsSd()` | `readHardwareSettingsEeprom()` |
| 127-134 | 1 each | `accelId`, `gyroId`, `highGId`, `baroId`, `radioId`, `gpsId`, `magId`, `sdId` | device ID codes ([settings-files.md](settings-files.md#device-id-codes)); 255 in `accelId` stops start-up | `parseEepromSettingsSd()` | `readHardwareSettingsEeprom()` (`sdId` is never read) |
| 135-137 | 1 each | `hwVersion`, `hwSubVersion`, `hwUnitNum` | hardware ID characters | `parseEepromSettingsSd()` | `readHardwareSettingsEeprom()` |
| 138-143 | 6 | `callSign` | ham radio call sign, 6 characters, no terminator | `parseEepromSettingsSd()` | `readHardwareSettingsEeprom()` |

The order of the IDs (127-134) is not the order `EEPROMsettings.txt` lists them in; the parser
writes each to its own address.

## Flight settings copy (144-227)

`setup()` writes a copy of the flight settings here (`writeSettingsEeprom()`, `Startup.cpp`), after
reading `Settings.txt` and applying `limitSettings()`. It was meant for in-flight recovery; since
#28, in-flight recovery reads `Settings.txt` as a normal start does. Of this block only
`inflightRecover` (186) is read: `setup()` reads it first thing, before the SD card is started, to
decide whether to try an in-flight recovery.

The addresses overlap. The writes happen in this order, and a later write replaces an earlier
one's bytes:

| Field | Address | Bytes written | Overwritten by |
| ----- | ------- | ------------- | -------------- |
| `fltProfile`, `units`, `reportStyle` | 164, 165, 166 | 1 each | |
| `setupTime` | 167-170 | 4 | |
| `mainDeployAlt` | 177-180 | 4 (float; the map's comment says 2) | `rcdTime` at 179-180 |
| `apogeeDelay` | 183-186 | 4 | `silentMode` 184, `magSwitchEnable` 185, `inflightRecover` 186 |
| `rcdTime` | 179-182 | 4 | |
| `silentMode`, `magSwitchEnable`, `inflightRecover`, `gpsLogFile` | 184, 185, 186, 187 | 1 each | |
| `fireTime` | 188-191 | 4 | |
| `pyro1Func` .. `pyro4Func` | 195, 194, 193, 192 | 1 each | |
| `txEnable`, `txPwr` | 196, 197 | 1 each | |
| `txFreq` | 198-201 | 4 | |
| `boosterSeparationDelay` | 203-206 | 4 | |
| `sustainerFireDelay` | 207-210 | 4 | |
| `airStart1Event`, `airStart1Delay` | 211, 212-215 | 1, 4 | |
| `airStart2Event`, `airStart2Delay` | 216, 217-220 | 1, 4 (the map's comment says 2) | `altThreshold` at 219-220 |
| `altThreshold` | 219-220 | 2 | |
| `maxAngle` | 221-222 | 2 | `stableRotn` at 222 |
| `stableRotn`, `stableVert` | 222, 223 | 1 each | |

So reading the copy back gives wrong values for `mainDeployAlt`, `apogeeDelay` (its upper three bytes
are `silentMode`, `magSwitchEnable` and `inflightRecover`; with those 0, 0 and 1 it reads
16777216 µs more than its low byte),
`airStart2Delay` and `maxAngle`. Fields in the map that nothing writes: `rocketName` (144-163),
`gTrigger` (171-172), `detectLiftoffTime` (173-176), `fhss` (202, reads 255 on an erased part),
`flyback` (224), `serialDebug` (225), `lastFile` (226-227).

## Flight state and stabilization

| Address | Bytes | Field | Contents | Written by | Read by |
| ------- | ----- | ----- | -------- | ---------- | ------- |
| 228-231 | 4 | `baseAlt` | float, pad altitude at liftoff (m) | the flight step at liftoff, with in-flight recovery on (`FlightStep.h`, `EEPROM_BASE_ALT`) | `rapidReset()` (`InflightRecover.cpp`) |
| 232 | 1 | `lastEvent` | last flight event, a `FlightCode` ([telemetry-packets.md](telemetry-packets.md#flight-event-codes)); 27 (touchdown) when no flight is in progress | the flight step at each event, with in-flight recovery on (`EEPROM_LAST_EVENT`); `setup()` sets 27 once it has read it; `checkFirstUse()` | `setup()`, first thing, for in-flight recovery |
| 233-240 | 1 each | `servo1Trim` .. `servo8Trim` | int8 servo trims | `setCanardTrim()` (`Calibration.cpp`), all 8 | `attachServos()` (`Startup.cpp`), servos 1-4 |
| 241-252 | 4 each | `kp`, `ki`, `kd` | reserved; nothing reads or writes them | | |
| 253 | 1 | `totalFlights` | flight counter, one byte (the map's comment says 2); 254 is not used | `loop()` at the end of the flight, not in test mode; `checkFirstUse()` writes 0 | `loop()` |
| 255-260 | 2 each | `gyroBiasX/Y/Z` | int16 | `accelCalibrate()` | `readCalibrationEeprom()` |

Everything above 260 is unused.
