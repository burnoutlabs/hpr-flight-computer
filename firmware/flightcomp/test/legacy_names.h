/**
 * @file legacy_names.h
 * @brief The old names the verbatim reference copies (legacy_v4_7_2/, legacy_setup/) use, mapped to
 * the current names of the same variables, fields, constants and functions.
 *
 * @details
 * The naming pass (#24) renamed them; the copies stay verbatim, so they are compiled with these
 * macros in force. Include this file immediately before the copies and legacy_names_end.h
 * immediately after them, so nothing else in the test sees the macros.
 */
#define A2D gainRatio
#define accelCS accelCs
#define accelCSpin accelCsPin
#define accelID accelId
#define ADCmax adcMax
#define ADCmidValue adcMidValue
#define baroCS baroCs
#define baroCSpin baroCsPin
#define baroID baroId
#define FHSS fhss
#define GNSS gnss
#define GPSecho gpsEcho
#define GPSID gpsId
#define GPSlog gpsLog
#define gyroCS gyroCs
#define gyroCSpin gyroCsPin
#define gyroID gyroId
#define highG_CS highGCs
#define highGbiasX highGBiasX
#define highGbiasY highGBiasY
#define highGbiasZ highGBiasZ
#define highGcsPin highGCsPin
#define highGID highGId
#define HWid hwId
#define HWsubVersion hwSubVersion
#define HWunitNum hwUnitNum
#define HWversion hwVersion
#define magCS magCs
#define magCSpin magCsPin
#define magID magId
#define pyroPWM pyroPwm
#define radioCS radioCs
#define radioCSpin radioCsPin
#define radioEN radioEn
#define radioID radioId
#define radioIRQ radioIrq
#define radioIRQpin radioIrqPin
#define radioRST radioRst
#define SD_CS sdCs
#define sdCSpin sdCsPin
#define sensorsOK sensorsOk
#define servo1pin servo1Pin
#define servo1trim servo1Trim
#define servo2pin servo2Pin
#define servo2trim servo2Trim
#define servo3pin servo3Pin
#define servo3trim servo3Trim
#define servo4pin servo4Pin
#define servo4trim servo4Trim
#define servo5pin servo5Pin
#define servo6pin servo6Pin
#define servo7pin servo7Pin
#define servo8pin servo8Pin
#define testModeRdPin testModeReadPin
#define TXenable txEnable
#define TXfreq txFreq
#define TXpwr txPwr
