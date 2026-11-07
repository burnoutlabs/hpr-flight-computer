# Telemetry packets

The byte layout is defined once, in the library both firmware images build:
`firmware/common/lib/TelemetryProtocol/src/TelemetryProtocol.h` (the layout table in its header
comment, and the encoders and decoders in `TelemetryProtocol.cpp`). That file is the source of
truth; the table below repeats it with what the flight computer puts in each field
(`buildTelemetryPkt()` in `firmware/flightcomp/src/Telemetry.cpp`, unless a row says otherwise).
The host tests `test_telemetry_protocol` and `test_telemetry_packets` hold golden packets.

Every packet starts with the 6-character call sign from EEPROM. Integers are little-endian two's
complement, floats IEEE-754 single precision, little-endian. Altitudes are in metres and
velocities in m/s: the firmware keeps both in metric and converts only for display.

## Pre-flight, 42 bytes (sent until liftoff)

| Offset | Type | Field | Value |
| ------ | ---- | ----- | ----- |
| 0-5 | char[6] | call sign | |
| 6 | uint8 | event | flight event code (below) |
| 7 | uint8 | GNSS fix | 1 after a position or altitude update, 0 when none for 2 s (`flightcomp.cpp`) |
| 8 | uint8 | continuity code | `cont.reportCode` from `checkPyroContinuity()` (`Startup.cpp`) |
| 9-28 | char[20] | rocket name | `settings.rocketName`, all 20 bytes: up to 19 characters and a terminator |
| 29-30 | int16 | base altitude | `baro.baseAlt`, m |
| 31-32 | int16 | GPS altitude | GNSS altitude less `baro.baseAlt`, m |
| 33-36 | float | latitude | degrees |
| 37-40 | float | longitude | degrees |
| 41 | uint8 | satellites | |

## In-flight, 62 bytes

Four samples of 11 bytes each, sample `n` (0-3) at offset `6 + 11n`, then a 12-byte trailer.

| Offset in sample | Type | Field | Value |
| ---------------- | ---- | ----- | ----- |
| 0 | uint8 | event | flight event code |
| 1-2 | int16 | velocity | `baro.vel`, m/s |
| 3-4 | int16 | altitude | `baro.smoothAlt`, m |
| 5-6 | int16 | roll | `rollZ` |
| 7-8 | int16 | off vertical | `offVert`, tenths of a degree, 0-1800 |
| 9-10 | int16 | acceleration | `accelNow × 33.41406087` (`accelNow` in m/s²; 33.41406087 = 32768 / 9.80665 / 100) |

| Offset | Type | Field | Value |
| ------ | ---- | ----- | ----- |
| 50-51 | int16 | packet number | counts from 1 after liftoff |
| 52-53 | int16 | GPS altitude | m above the pad |
| 54-57 | float | latitude | degrees |
| 58-61 | float | longitude | degrees |

## Post-flight, 26 bytes

| Offset | Type | Field | Value (set in `loop()`, `flightcomp.cpp`, as the flight ends) |
| ------ | ---- | ----- | ----- |
| 6 | uint8 | event | flight event code |
| 7-8 | int16 | max altitude | `baro.maxAlt`, m |
| 9-10 | int16 | max velocity | `maxVelocity`, m/s |
| 11-12 | int16 | max G | `maxG / 0.980665`, tenths of a g |
| 13-14 | int16 | max GPS altitude | m above the pad |
| 15 | uint8 | GNSS fix | as above |
| 16-17 | int16 | GPS altitude | m above the pad |
| 18-21 | float | latitude | degrees |
| 22-25 | float | longitude | degrees |

## Sync, 10 bytes (FHSS only)

| Offset | Type | Field |
| ------ | ---- | ----- |
| 6 | uint8 | 255 (`SYNC_MARKER`), in place of the event |
| 7 | uint8 | current channel, an index into `FHSS_CHANNEL_MHZ` |
| 8-9 | int16 | index into `HOP_SEQUENCE` of the next hop |

The 915 MHz channel plan (130 channels, 902.1 to 927.9 MHz in 200 kHz steps) and the
2000-entry hop sequence are in the same library.

## Flight event codes

Source: `enum FlightCode` in `firmware/flightcomp/include/declarations.h`. The same codes go to
the SD log (`radioCode`) and EEPROM (`lastEvent`).

| Code | Name | Code | Name |
| ---- | ---- | ---- | ---- |
| 0 | `PREFLIGHT` | 17 | `AIRSTART2_BURNOUT` |
| 1 | `LIFTOFF` | 18 | `NO_FIRE_ROTN_LIMIT` |
| 2 | `BOOSTER_BURNOUT` | 19 | `NO_FIRE_ALT_LIMIT` |
| 3 | `APOGEE` | 20 | `NO_FIRE_ROTN_ALT_LIMIT` |
| 4 | `FIRE_APOGEE` | 21 | `BOOSTER_APOGEE` |
| 5 | `SEPARATION` | 22 | `FIRE_BOOSTER_APOGEE` |
| 6 | `FIRE_MAINS` | 23 | `BOOSTER_SEPARATION` |
| 7 | `UNDER_CHUTE` | 24 | `FIRE_BOOSTER_MAINS` |
| 8 | `EJECT_BOOSTER` | 25 | `BOOSTER_UNDER_CHUTE` |
| 9 | `FIRE_SUSTAINER` | 26 | `TIME_LIMIT` |
| 10 | `SUSTAINER_IGNITION` | 27 | `TOUCHDOWN` |
| 11 | `SUSTAINER_BURNOUT` | 28 | `POWER_LOSS_RESTART` |
| 12 | `FIRE_AIRSTART1` | 29 | `BOOSTER_TOUCHDOWN` |
| 13 | `AIRSTART1_IGNITION` | 30 | `BOOSTER_PREFLIGHT` |
| 14 | `AIRSTART1_BURNOUT` | 31 | `BOOSTER_TIME_LIMIT` |
| 15 | `FIRE_AIRSTART2` | 32 | `BOOSTER_POWER_LOSS` |
| 16 | `AIRSTART2_IGNITION` | | |

## Continuity codes

Source: `checkPyroContinuity()` in `firmware/flightcomp/src/Startup.cpp`, before liftoff.

| Code | Meaning |
| ---- | ------- |
| 1-4 | that pyro channel has a function but no continuity |
| 0 | no channel has continuity |
| 9, 8, 7 | single stage or booster: main and apogee, main only, apogee only |
| 6, 5 | two stage: booster separation, upper stage, apogee and main; or all but booster separation. Airstart: both airstarts, apogee and main; or all but the second airstart |
