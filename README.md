
**Builds on [HPR Rocket Flight Computer](https://github.com/SparkyVT/HPR-Rocket-Flight-Computer) by SparkyVT**  
The code in this repository is based on upstream V4.7.2 ([`57c1224`](https://github.com/SparkyVT/HPR-Rocket-Flight-Computer/tree/57c122420cbadd5b4ad754281a0be82aa6ef7d2b)).  

## DESCRIPTION

This code base is the software to run an advanced flight computer in high-power amateur rockets.  
Hardware setups vary, but typically can fit within a 38mm airframe.  
It has similar capability as a TeleMega, but without a smart-phone app or advanced interfaces  
Flight-tested on over 250 successful flights, including M-and-N powered supersonic flights to over 34K feet and Mach 2.3.  
For large or high-power projects, a commercially available backup computer is strongly recommended.  

## FEATURES

- Full-featured dual deploy/multi-stage/airstart rocket flight computer capable to 100,000ft or more
- Tilt-sensing lockout for ignition of second stages and/or airstarts
- Live telemetry over 433MHz or 915MHz LoRa (433MHz: USA amateur 70cm band, EUR license free) (915MHz: USA license free) 
- 4 high-current pyro outputs with continuity checks
- Advanced MEMS sensor package: GNSS, accelerometers, gyroscope, magnetometer, barometer, and LoRa radio
- High Data-Capture rate: approximately 50,000 samples per second recorded to SD card
  - 1000Hz 3-axis digital 24G and 100G accelerometer data logging
  - 1000Hz 3-axis digital 2000dps gyroscope data logging
  - 1000Hz of flight events & continuity data logging
  - 1000Hz of sensor-fuzed speed & altitude
  - 100Hz of pitch, yaw, roll rotation
  - 40Hz of magnetic data logging and magnetic roll
  - 30Hz-100Hz of digital barometric data logging (Altitude, pressure, temperature)
  - 30Hz of main battery voltage (1400Hz during pyro events)
  - 20Hz of LoRa telemetry output (time, event, acceleration, speed, altitude, rotation, GNSS position, signal strength)
  - 5Hz-25Hz of GNSS data logging (chip-dependent data rates & constellations)
  - Separate data file for each flight up to 100 flights
  - Optional separate GNSS NMEA capture file for plotting in Google Earth, UBLOX NEO-M9N configuration approved for TRA altitude record attempts
- Simple, easy-to-use configuration interface through the SD card
  - User Selectable Flight Mode: Single-Stage, Two-Stage, Airstart, or Booster
  - Configurable Apogee delay
  - Optional Audible Battery Voltage report at startup
  - Optional Magnetic Switch Startup & Shut-down
  - Preflight audible reporting options: Perfectflight or Marsa
  - User selectable telemetry frequency & power settings
  - 8 configurable servo outputs (4 powered, 4 un-powered)
  - User selectable inflight brownout recovery
- Mach immune, sensor-fusion based apogee event
- Barometric based main deploy event
- Audible pre-flight continuity report
- Audible Post-flight max altitude & speed report
- Mount in any orientation, automatic orientation detection with built-in self calibration mode
- Bench-test mode activated w/ tactile button; user configurable status messages over USB Serial
- Report in SI or Metric units
- Compatible with Teensy 3.2, 3.5, 3.6, 4.0, 4.1
  - Connect any sensor to any available I2C or SPI bus
  - Create your own custom hardware setup with configurable pins for continuity, firing, and servos
  - Connect UBLOX GPS unit to any available HW Serial port

## FLIGHT COMPUTER COMPONENTS

- **Microprocessor**: Teensy 4.1, 3.5, or 3.6 (compatible with provided PCB file) or Teensy 4.0/3.2 (no PCB file provided)
- **IMU**: LSM6DSOX (recommended), LSM6DS3TR, LSM9DS1, LSM6DS33, MPU6050, MPU9250, or LSM303 & L3GD20H combo
- **Magnetometer**: LIS3MDL (recommended), LSM9DS1, MPU9250, or LSM303
- **100G Accelerometer**: H3LIS331DL (recommended), ADXL377, or ADXL377 & ADS1115 combo
- **Pressure Sensor**: MS5611 (recommended), MS5607, MPL3115A2, BMP180, BMP280, or BMP388 (Note: BMP280 & BMP388 incompatible with telemetry due to EMI)
- **GNSS**: UBLOX M6, M7, M8, M9 (NEO footprint supported in PCB file) or Adafruit Ultimate GPS
- **LoRa Telemetry**: RFM96W or RFM95W, both capable of TX on 433MHz or 915MHz (433MHz USA: Amateur License Required, EUR: License Free), or an RFD900 serial radio
- **Other devices**: any accelerometer, gyroscope, magnetometer, high-G accelerometer, barometer, GNSS receiver, or packet radio without a built-in driver can be used through an external library (see `firmware/flightcomp/src/Device_Drivers_External.cpp`)

## GROUND STATION COMPONENTS

- **MCU**: Arduino Nano 33 BLE
- **Telemetry**: RFM96W 433MHz (USA: Ham License Required, EUR: License Free) or RFM95W 915MHz (USA: License Free)
- **GPS**: Ublox SAM-M8Q (optional)
- **IMU**: Sparkfun LSM9DS1 breakout
- **LCD**: 20X4 Sparkfun LCD
- **SD Card**: Adafruit SD Card breakout board
- **Antenna**: 433Mhz 5-element Yagi (Ham Only) or 900Mhz (License Free)

## BUILDING

The firmware builds with [PlatformIO](https://platformio.org). There are two PlatformIO projects, one per firmware image, each with an env per supported board:

| Project | Path | Envs |
| --- | --- | --- |
| Flight computer | `firmware/flightcomp` | `teensy41`, `teensy40`, `teensy36`, `teensy35`, `teensy31` (Teensy 3.2) |
| Ground station | `firmware/gndstation` | `teensy41`, `teensy40` |

```sh
pip install platformio==6.1.19
pio run -d firmware/flightcomp              # every flight computer env
pio run -d firmware/flightcomp -e teensy41  # one board
pio run -d firmware/flightcomp -e teensy41 -t upload
pio test -d firmware/flightcomp -e native   # host unit tests
```

With [just](https://just.systems) installed, `just build` builds every env of both projects, `just test` runs the host unit tests, and `just check` runs the same build and tests as CI. `just --list` shows the other recipes.

The flight computer's unit tests (`firmware/flightcomp/test`, [Unity](https://www.throwtheswitch.org/unity)) build with the host's own C++ compiler in the `native` env, which `pio run` leaves out. They compile the real flight computer sources under test against small stand-ins for the Teensy core and libraries in `test/shim`. The rotation tests run twice: once on the Teensy 4.x double-precision path, and once on the Teensy 3.x single-precision path, built with `-fsingle-precision-constant` as those boards are.

The flight step (`FlightStep.h`) has two suites. `test_flight_step_neutral` runs it side by side with the V4.7.2 code it was extracted from (verbatim copies in `test/legacy_v4_7_2/`, except the accelerometer apogee detector, removed for #14) on synthetic and logged flights, and requires the same state and the same I/O on every loop pass. `test_flight_replay` replays three logged flights (`test/replay/`, data CC BY 4.0, see its `ATTRIBUTION.md`) and checks liftoff, burnout, apogee, main and touchdown against the logged events within stated tolerances; `test/replay/make_fixtures.py` rebuilds the fixtures from post-flight-analysis.

`test_setup_neutral` runs `setup()` (`Startup.cpp`) side by side with the `setup()` it was refactored from (verbatim copies in `test/legacy_setup/`), with the board's I/O logged through `test/startup_harness.h`, and requires the same state and the same I/O call for call over scenarios that take its branches; it also compares `setPolyMorphs()` with its original for every device ID.

`test_inflight_recovery` flies a synthetic flight with in-flight recovery on, resets the board at a chosen moment (keeping only the EEPROM), runs `setup()` with the sensors reading the same flight, and flies on: it checks that a charge already fired does not fire again, that the remaining ones do, that apogee after a reset in the coast comes when it would have without the reset, that recovery setting 1 fires nothing, and that a board standing on the pad with a stale flight record starts normally.

Versions are pinned in each `platformio.ini`: the `teensy` platform 6.0.0, which bundles Teensyduino 1.62 (`framework-arduinoteensy` 1.162.0), TinyGPSPlus 1.0.3 and, for the ground station, SparkFun SerLCD 1.0.9. SdFat, SD, SPI, Wire, i2c_t3, PWMServo and EEPROM come with Teensyduino. The Teensy 3.2 env runs at 96 MHz, Teensyduino's default for that board.

The flight computer is plain C++ in PlatformIO's layout: each module is a `.cpp` in `src/` with its declarations in a header of the same name in `include/`. One exception: the SD card module's header is `SD_Card.h`, since `SD.h` is Teensyduino's SD library. `FlightStep.h` holds the flight step: `flightStep()` (liftoff detection, velocity and altitude integration, the rotation update and the event logic, `checkEvents()`) is defined there, always inline, for the flight loop, as is `writeSdFlightData()` in `SD_Card.h`. It does no I/O: it returns the pin, EEPROM, radio-timer and SD actions it wants, in order, and `loop()` carries them out with `performFlightActions()`. `FlightStep.cpp` holds the barometer processing that feeds it and its out-of-line helpers, so the host tests can build the whole step. `Startup.cpp` holds `setup()` and its initialization steps, which the in-flight recovery (`InflightRecover.cpp`) shares. `declarations.h` declares the shared types and globals, and `globals.cpp` defines them; `board.h` holds the declarations that differ between Teensy boards. The ground station follows the same layout: `gndstation.cpp` holds `setup()` and `loop()`, each other module has a `.cpp` and a header of the same name, and `declarations.h`/`globals.cpp` declare and define its globals. Building from the Arduino IDE or arduino-cli is no longer supported: the sketch folders no longer match their main `.ino` names, and CI builds only with PlatformIO.

The telemetry protocol, which the flight computer encodes and the ground station decodes, is a PlatformIO library in `firmware/common/lib/TelemetryProtocol`: the packet layout and its encoders and decoders, the 915 MHz channel list, the hop sequence and the hailing-channel function. It touches no hardware. Projects use it through `lib_deps = symlink://../common/lib/TelemetryProtocol`. The host tests include a check that the flight computer still sends the V4.7.2 packets byte for byte (`test_telemetry_packets`) and a round trip through the shared decoders (`test_telemetry_protocol`).

`firmware/gndstation/archive` holds earlier ground station sketches for reference. They are not built.

CI builds every env and runs the host unit tests on each push to `master` and each pull request, and writes the flash and RAM use per board to the job summary. The Teensy 3.2 build uses about 78% of its flash, so watch that figure.
