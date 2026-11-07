# SD card flight log

Source: `createNextFileSd()` and `writeSdFooterClose()` in `firmware/flightcomp/src/SD.cpp`,
`writeSdFlightData()` in `firmware/flightcomp/include/SD_Card.h`, and `renameFileSd()` in
`firmware/flightcomp/src/flightcomp.cpp`.

## Files

- `FLIGHTnn.txt`: the flight data, `nn` the first free number from 01. After the footer is
  written and the file closed, it is renamed `YYYYMMDDHHMM <rocket name>.csv` from the GNSS date
  and time at liftoff (UTC), except in test mode or when no GNSS date was captured; a rename that
  fails leaves the old name.
- `GPSlognn.nmea`: every NMEA sentence from the GNSS receiver, when `GPSlogFile = 1` in
  `Settings.txt`. It starts with the line `GPS Log`.

## Data rows

The header line is written when the file is created; then one row for each pass of the flight
loop (`loop()`, `flightcomp.cpp`) that has anything new: a sensor sample, a GNSS update, a battery
reading or a radio packet.
Every field, the last included, is followed by a comma, so each row ends `,\r\n`. A field with no
new value this pass is left empty.

| Column | Source | Notes |
| ------ | ------ | ----- |
| `timeStamp` | `fltTime.timeCurrent` | µs, the sum of the loop's time steps since liftoff detection |
| `accelX`, `accelY`, `accelZ` | `accel.x/y/z` | oriented accelerometer counts |
| `gyroX`, `gyroY`, `gyroZ` | `gyro.x/y/z` | oriented gyro counts |
| `highGx`, `highGy`, `highGz` | `highG.x/y/z` | oriented high-G counts |
| `cyclesBtwn` | `cyclesBtwn` | **test mode only** (these four columns) |
| `writeFlags` | 9 digits | `1`/`0` for a new accel, gyro, high-G, mag, baro and temperature sample, a radio packet, a GNSS update, a battery reading |
| `sampleTime`, `writeTime` | `sampleTime`, `writeTime` | µs |
| `smoothHighGz` | `highGSmooth` | |
| `rollZ` | `rollZ` | |
| `yawY`, `pitchX` | `yawY`, `pitchX` | |
| `offVert` | `offVert` | tenths of a degree |
| `intVel`, `intAlt` | `accelVel`, `accelAlt` | integrated from the accelerometer, 2 decimals |
| `fusionVel`, `fusionAlt` | `fusionVel`, `fusionAlt` | 2 decimals |
| `fltEvents` | event bit string | see below |
| `radioCode` | `radioTelemetry.event` | the current flight event code ([telemetry-packets.md](telemetry-packets.md#flight-event-codes)) |
| `pyroCont` | `C` then 4 digits | continuity of pyro 1 to 4 |
| `pyroFire` | `F` then 4 digits | firing state of pyro 1 to 4 |
| `pyroPin` | `pins.firePin` | |
| `baroAlt`, `altMoveAvg`, `baroVel`, `baroPress` | `baro.alt`, `baro.smoothAlt`, `baro.vel`, `baro.pressure` | only on a new barometer sample |
| `baroTemp` | `baro.temperature` | only on a new temperature sample |
| `battVolt` | `voltage` | only on a new battery reading |
| `magX`, `magY`, `magZ` | `mag.x/y/z` | only on a new magnetometer sample |
| `gnssLat`, `gnssLon` | `gnss.latitude`, `gnss.longitude` | only on a new GNSS fix (these six columns), 6 decimals |
| `gnssSpeed` | `gps.speed.mph()` | mph |
| `gnssAlt` | `gnss.alt` | m above the pad |
| `gnssAngle` | `gps.course.deg()` | degrees |
| `gnssSatellites` | `radioTelemetry.satNum` | |
| `radioPacketNum` | `radioTelemetry.packetNum` | only when a packet was sent this pass |

### The `fltEvents` bit string

One `1` or `0` per flag, with no separators, in this order. The middle part depends on the flight
profile, so the string is 8, 13 or 16 characters long.

| Profile | Characters |
| ------- | ---------- |
| all | `liftoff`, `boosterBurnout`, `boosterBurnoutCheck` |
| `2` only | `boosterSeparation`, `sustainerFireCheck`, `sustainerFire`, `sustainerIgnition`, `sustainerBurnout` |
| `A` only | `airStart1Check`, `airStart1Fire`, `airStart1Ignition`, `airStart1Burnout`, `airStart2Check`, `airStart2Fire`, `airStart2Ignition`, `airStart2Burnout` |
| all | `apogee`, `apogeeFire`, `mainDeploy`, `touchdown`, `timeOut` |

## Footer

Written once the flight ends (touchdown or the record time limit), as header and value line pairs:

1. `Max Baro Alt, Max GPS Alt, Max Speed, Max Gs, baseAlt, padTime, initial Y ang, initial X ang,`
   the initial sensor values, the sensor biases and the barometer offsets. The maxima are in the
   display units; `Max Gs` is in g.
2. Launch date (`day/month/year`), UTC time, launch altitude, latitude and longitude.
3. Landing date, UTC time, altitude, latitude and longitude. The landing date printed is the
   liftoff date (`gnss.liftoff`).
4. The settings of the flight: rocket name, call sign, hardware ID, code version, flight profile,
   units, in-flight recovery, pyro 4 to 1 functions, apogee delay, main deploy altitude, setup
   time, record time, firing time, TX enable, power, frequency, FHSS, sea level pressure; then for
   profile `2` the ignition and separation delays, altitude threshold and maximum angle, or for
   profile `A` the two airstart events and delays, altitude threshold and maximum angle.
