/**
 * @file Device_Mgmt.cpp
 * @brief Device management: function-pointer setup and sensor orientation.
 *
 * @details
 * setPolyMorphs() points the generic function pointers declared in declarations.h at the
 * optimized driver, the external-library stub, or a dummy function, according to the device
 * IDs read from EEPROM, for every sensor, the packet radio, and the GNSS receiver.
 * The orient functions remove the calibration bias from raw sensor data and map it onto the
 * rocket's axes; the dummy functions stand in for absent devices.
 */

#include <Arduino.h>

#include "Device_Mgmt.h"
#include "declarations.h"
#include "Device_Drivers.h"
#include "Device_Drivers_External.h"
#include "RFDx_Driver.h"
#include "SX127X_Driver.h"
#include "UBLOX_GNSS_Config.h"

//-----------------------------------------------
// Written by SparkyVT, TRA #12111, NAR #85720, L3
//-----------Change Log--------------------------
// 22 OCT 23: Initial Build
// 19 NOV 23: Increased devices supported
// 29 SEP 24: Established polymorphic variables for function pointers
//***************************************************************************
// The system interfaces with the sensors through generic "begin" and "get" functions, i.e. beginAccel(), getAccel(),...
// Each function can either use an external library or use optimized drivers provided in Sensor_Drivers.ino
// The "begin" function is used to initialize the sensor and put it at the necessary gain and oupt data rates
// The "get" function is used to retrieve the raw data from the sensor.  Raw sensor data is necessary to use the built-in calilbration
// The "orient" function is used to remove bias from the sensor data and orient it to the proper axes. Called after "get"
//***************************************************************************
//----------------------------
// LIST OF FUNCTIONS
//----------------------------
// setPolyMorphs(): sets the function pointers for the devices
// orientAccel(): removes bias from raw sensor data and orients corrected data to the proper axes
// orientGyro(): removes bias from raw sensor data and orients corrected data to the proper axes
// orientMag(): removes bias from raw sensor data and orients corrected data to the proper axes
// orientHighG(): removes bias from raw sensor data and orients corrected data to the proper axes
// gnssConfig(): configures the GNSS unit for flight conditions
// GNSSpowerSaveMode(): reduces power consumption post-flight
// dummyVoid(): executes no code when a routine or sensor is absent
// dummyBool(): executes no code and returns true when a routine or sensor is absent

/**
 * @brief Point the device function pointers at the drivers for the device IDs read from EEPROM.
 *
 * @details
 * An ID that names no device gets the same default for every device of a kind:
 * - the accelerometer and the barometer, which every flight needs, default to the external
 *   library stubs (Device_Drivers_External.cpp). Until a library is written into them, their
 *   begin function returns false, so setup() stops with the sensor failure alarm;
 * - the gyro, high-G accelerometer, magnetometer, radio and GNSS receiver, which a board may
 *   leave out, default to the no-op dummies, whose begin functions report success.
 * The accelerometers with a gyro on the same chip (IDs 2-6 and 8) read both in getAccel(), so
 * their gyro IDs (2-6) take the gyro default: no separate gyro read.
 * Every pointer is set for every ID from 0 to 255.
 */
void setPolyMorphs()
{

  // accelerometer pointers
  switch (sensors.accel)
  {
  case 1:
    beginAccel = beginLsm303Accel;
    getAccel = getLsm303Accel;
    break;
  case 2:
    beginAccel = beginLsm9ds1AccelGyro;
    getAccel = getLsm9ds1AccelGyro;
    break;
  case 3:
    beginAccel = beginLsm6ds33;
    getAccel = getLsm6ds33;
    break;
  case 4:
    beginAccel = beginMpu6050;
    getAccel = getMpu6050;
    break;
  case 5:
    beginAccel = beginMpu9250AccelGyro;
    getAccel = getMpu9250AccelGyro;
    break;
  case 6:
    beginAccel = beginLsm6dsox;
    getAccel = getLsm6dsox;
    break;
  case 8:
    beginAccel = beginLsm6ds3tr;
    getAccel = getLsm6ds3tr;
    break;
  case 7: // External Library
  default:
    beginAccel = beginExternalAccel;
    getAccel = getExternalAccel;
    break;
  }

  // gyroscope pointers
  switch (sensors.gyro)
  {
  case 1:
    beginGyro = beginL3gd20h;
    getGyro = getL3gd20h;
    break;
  case 7: // External Library
    beginGyro = beginExternalGyro;
    getGyro = getExternalGyro;
    break;
  default: // none, or read with the accelerometer
    beginGyro = dummyBool;
    getGyro = dummyVoid;
    break;
  }

  // high-G accelerometer pointers
  switch (sensors.highG)
  {
  case 1:
    beginHighG = beginAds1115;
    getHighG = getAds1115;
    break;
  case 2:
    beginHighG = beginH3lis331dl;
    getHighG = getH3lis331dl;
    break;
  case 3:
    beginHighG = beginAdxl377;
    getHighG = getAdxl377;
    break;
  case 4: // External Library
    beginHighG = beginExternalHighG;
    getHighG = getExternalHighG;
    break;
  default: // none
    beginHighG = dummyBool;
    getHighG = dummyVoid;
    break;
  }

  // magnetometer pointers
  switch (sensors.mag)
  {
  case 1:
    beginMag = beginLsm303Mag;
    getMag = getLsm303Mag;
    break;
  case 2:
    beginMag = beginLsm9ds1Mag;
    getMag = getLsm9ds1Mag;
    break;
  case 3:
    beginMag = beginLis3mdl;
    getMag = getLis3mdl;
    break;
  case 4:
    beginMag = beginMpu9250Mag;
    getMag = getMpu9250Mag;
    break;
  case 5: // External Library
    beginMag = beginExternalMag;
    getMag = getExternalMag;
    break;
  default: // none
    beginMag = dummyBool;
    getMag = dummyVoid;
    break;
  }

  // barometer pointers
  switch (sensors.baro)
  {
  case 1:
    beginBaro = beginBmp180;
    getBaro = getBmp180;
    break;
  case 2:
    beginBaro = beginMpl3115a2;
    getBaro = getMpl3115a2;
    break;
  case 3:
    beginBaro = beginBmp280;
    getBaro = getBmp280;
    break;
  case 4:
    beginBaro = beginBmp388;
    getBaro = getBmp388;
    break;
  case 5: // MS5611
  case 6: // MS5607
    beginBaro = beginMs56xx;
    getBaro = getMs56xx;
    break;
  case 7:
    beginBaro = beginLps25h;
    getBaro = getLps25h;
    break;
  case 8: // External Library
  default:
    beginBaro = beginExternalBaro;
    getBaro = getExternalBaro;
    break;
  }

  // radio pointers
  switch (sensors.radio)
  {
  case 1:
    beginRadio = beginSx127x;
    setRadioFreq = setFreqSx127x;
    setRadioPwr = setPwrSx127x;
    radioSleep = sleepSx127x;
    radioSendPkt = sendPktSx127x;
    break;
  case 2: // the RFD900 has no frequency, power or sleep commands here
    beginRadio = beginRfd900;
    setRadioFreq = dummyBool1;
    setRadioPwr = dummyBool2;
    radioSleep = dummyVoid;
    radioSendPkt = sendPktRfd900;
    break;
  case 3: // External Library
    beginRadio = beginExternalRadio;
    setRadioFreq = setExternalRadioFreq;
    setRadioPwr = setExternalRadioPwr;
    radioSleep = externalRadioSleep;
    radioSendPkt = sendExternalRadioDataPkt;
    break;
  default: // none
    beginRadio = dummyBool;
    setRadioFreq = dummyBool1;
    setRadioPwr = dummyBool2;
    radioSleep = dummyVoid;
    radioSendPkt = dummyBool3;
    break;
  }

  // GNSS receiver pointers
  switch (sensors.gnss)
  {
  case 1: // NEO-M8N
  case 2: // SAM/MAX/NEO-M8Q/W
  case 3: // NEO-M9N
    gnssRestoreDefaults = ubloxRestoreDefaults;
    gnssConfig = ubloxConfig;
    gnssPowerSave = ubloxPowerSave;
    break;
  case 4: // Adafruit Ultimate GPS or External Library
    gnssRestoreDefaults = dummyVoid;
    gnssConfig = externalGnssConfig;
    gnssPowerSave = externalGnssPowerSave;
    break;
  default: // none
    gnssRestoreDefaults = dummyVoid;
    gnssConfig = dummyVoid;
    gnssPowerSave = dummyVoid;
    break;
  }

} // end setPolyMorphs()

void orientAccel()
{

  // after getting the raw sensor data, we need to remove the bias and orient the raw data to the real-world axes

  // indicate new sample
  accel.newSamp = true;

  // remove bias
  accel.rawX -= accel.biasX;
  accel.rawY -= accel.biasY;
  accel.rawZ -= accel.biasZ;

  // orient sensor data
  accel.x = *accel.ptrX * *accel.ptrXSign;
  accel.y = *accel.ptrY * *accel.ptrYSign;
  accel.z = *accel.ptrZ * *accel.ptrZSign;
}

void orientMag()
{

  // indicate new sample
  mag.newSamp = true;

  // remove bias
  mag.rawX -= mag.biasX;
  mag.rawY -= mag.biasY;
  mag.rawZ -= mag.biasZ;

  // translate sensor data
  mag.x = *mag.ptrX * *mag.ptrXSign;
  mag.y = *mag.ptrY * *mag.ptrYSign;
  mag.z = *mag.ptrZ * *mag.ptrZSign;
}

void orientGyro()
{

  // indicate new sample
  gyro.newSamp = true;

  // remove bias
  gyro.rawX -= gyro.biasX;
  gyro.rawY -= gyro.biasY;
  gyro.rawZ -= gyro.biasZ;

  // orient sensor data
  gyro.x = *gyro.ptrX * *gyro.ptrXSign;
  gyro.y = *gyro.ptrY * *gyro.ptrYSign;
  gyro.z = *gyro.ptrZ * *gyro.ptrZSign;
}

void orientHighG()
{

  // indicate new sample
  highG.newSamp = true;

  // remove bias
  highG.rawX -= highG.biasX;
  highG.rawY -= highG.biasY;
  highG.rawZ -= highG.biasZ;

  // orient sensor data
  highG.x = *highG.ptrX * *highG.ptrXSign;
  highG.y = *highG.ptrY * *highG.ptrYSign;
  highG.z = *highG.ptrZ * *highG.ptrZSign;
}

void dummyVoid() {}
bool dummyBool() { return true; }
bool dummyBool1(float freq) { return true; }
bool dummyBool2(int8_t pwr) { return true; }
bool dummyBool3(uint8_t *data, uint8_t len) { return true; }