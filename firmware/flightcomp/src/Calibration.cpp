/**
 * @file Calibration.cpp
 * @brief Calibration routines for various sensors.
 *
 * This file contains functions and routines for calibrating different sensors,
 * as well as managing their calibration data.
 *
 * @details
 * - `accelCalibrate()`: Calibrates the accelerometers.
 * - `magCalibrate()`: Calibrates the magnetometer.
 * - `writeCalibration()`: Stores calibration data in EEPROM.
 * - `setOrientation()`: Determines and stores the sensor orientation.
 * - `readOrientation()`: Reads orientation data from EEPROM.
 * - `baroCalibrate()`: Calibrates the barometric pressure sensor.
 * - `setCanardTrim()`: Calibrates canard trim settings.
 * - `wiggleServo()`: Moves a specified servo for calibration purposes.
 *
 * @section changelog Change Log
 * - 17 JUL 21: Initial breakout created.
 * - 10 SEP 21: Barometer calibration routine created.
 * - 30 DEC 21: Updated magnetometer calibration to account for resetMagGain routine.
 * - 18 APR 22: Major overhaul to account for independent sensors.
 */

#include <Arduino.h>
#include <EEPROM.h>

#include "Calibration.h"
#include "declarations.h"
#include "flightcomp.h"
#include "SD_Card.h"

// serial input characters for setCanardTrim()
char trimServoChar;
char trimDirectionChar;
char trimEndChar;

/**
 * @brief Calibration routine for IMU and High-G accelerometers.
 */
void accelCalibrate()
{

  Serial.println(F("Accelerometer Calibration Mode Confirmed. Ensure the altimeter is pointed vertical and held steady on a level surface"));

  for (uint8_t i = 1; i < 20; i++)
  {
    digitalWrite(pins.beep, HIGH);
    delay(250);
    digitalWrite(pins.beep, LOW);
    delay(250);
  }

  digitalWrite(pins.beep, HIGH);

  // reset the axis pointers
  accel.ptrX = &accel.rawX;
  accel.ptrY = &accel.rawY;
  accel.ptrZ = &accel.rawZ;
  gyro.ptrX = &gyro.rawX;
  gyro.ptrY = &gyro.rawY;
  gyro.ptrZ = &gyro.rawZ;
  mag.ptrX = &mag.rawX;
  mag.ptrY = &mag.rawY;
  mag.ptrZ = &mag.rawZ;
  highG.ptrX = &highG.rawX;
  highG.ptrY = &highG.rawY;
  highG.ptrZ = &highG.rawZ;
  // reset the direction pointers
  accel.ptrXSign = &accel.dirX;
  accel.ptrYSign = &accel.dirY;
  accel.ptrZSign = &accel.dirZ;
  gyro.ptrXSign = &gyro.dirX;
  gyro.ptrYSign = &gyro.dirY;
  gyro.ptrZSign = &gyro.dirZ;
  mag.ptrXSign = &mag.dirX;
  mag.ptrYSign = &mag.dirY;
  mag.ptrZSign = &mag.dirZ;
  highG.ptrXSign = &highG.dirX;
  highG.ptrYSign = &highG.dirY;
  highG.ptrZSign = &highG.dirZ;
  // reset the direction values
  accel.dirX = accel.dirY = accel.dirZ = 1;
  gyro.dirX = gyro.dirY = gyro.dirZ = 1;
  mag.dirX = mag.dirY = mag.dirZ = 1;
  highG.dirX = highG.dirY = highG.dirZ = 1;

  Serial.println(F("Calibrating..."));

  accel.biasX = accel.biasY = accel.biasZ = 0;
  accel.sumX0 = accel.sumY0 = accel.sumZ0 = 0;
  highG.biasX = highG.biasY = highG.biasZ = 0;
  highG.sumX0 = highG.sumY0 = highG.sumZ0 = 0;

  int32_t accelSamps = 0;
  int32_t highGsamps = 0;
  int32_t gyroSamps = 0;
  uint32_t sampTime = 3000000;
  uint32_t calibrationStart = micros();
  bool samplePrint = false;
  while (micros() - calibrationStart < sampTime)
  {
    // accelerometer
    if (micros() - accel.timeLastSamp > accel.timeBtwnSamp)
    {
      getAccel();
      accel.timeLastSamp = micros();
      accel.sumX0 += accel.rawX;
      accel.sumY0 += accel.rawY;
      accel.sumZ0 += accel.rawZ;
      accelSamps++;
      samplePrint = true;
    }
    // high-G accelerometer
    if (sensors.highG != 0 && micros() - highG.timeLastSamp > highG.timeBtwnSamp)
    {
      getHighG();
      highG.timeLastSamp = micros();
      highG.sumX0 += highG.rawX;
      highG.sumY0 += highG.rawY;
      highG.sumZ0 += highG.rawZ;
      highGsamps++;
    }
    // gyroscope
    if (micros() - gyro.timeLastSamp > gyro.timeBtwnSamp)
    {
      getGyro();
      gyro.timeLastSamp = micros();
      gyro.sumX0 += gyro.rawX;
      gyro.sumY0 += gyro.rawY;
      gyro.sumZ0 += gyro.rawZ;
      gyroSamps++;
    }

    if (accelSamps % 100 == 0 && samplePrint)
    {
      Serial.print("Accel: ");
      Serial.print(accel.rawX);
      Serial.print(',');
      Serial.print(accel.rawY);
      Serial.print(',');
      Serial.println(accel.rawZ);
      Serial.print("HighG: ");
      Serial.print(highG.rawX);
      Serial.print(',');
      Serial.print(highG.rawY);
      Serial.print(',');
      Serial.println(highG.rawZ);
      Serial.print("Gyro: ");
      Serial.print(gyro.rawX);
      Serial.print(',');
      Serial.print(gyro.rawY);
      Serial.print(',');
      Serial.println(gyro.rawZ);
      samplePrint = false;
    }
  } // end sample period

  // calculate the bias
  accel.biasX = (int16_t)(accel.sumX0 / accelSamps);
  accel.biasY = (int16_t)(accel.sumY0 / accelSamps);
  accel.biasZ = (int16_t)(accel.sumZ0 / accelSamps);
  // no samples without a high-G accelerometer: its bias stays 0
  if (highGsamps > 0)
  {
    highG.biasX = (int16_t)(highG.sumX0 / highGsamps);
    highG.biasY = (int16_t)(highG.sumY0 / highGsamps);
    highG.biasZ = (int16_t)(highG.sumZ0 / highGsamps);
  }
  gyro.biasX = (int16_t)(gyro.sumX0 / gyroSamps);
  gyro.biasY = (int16_t)(gyro.sumY0 / gyroSamps);
  gyro.biasZ = (int16_t)(gyro.sumZ0 / gyroSamps);

  // reset the counters
  accel.sumX0 = accel.sumY0 = accel.sumZ0 = 0;
  highG.sumX0 = highG.sumY0 = highG.sumZ0 = 0;
  gyro.sumX0 = gyro.sumY0 = gyro.sumZ0 = 0;

  // determine the altimeter orientation
  setOrientation();

  // Store bias in EEPROM
  writeCalibration(accel.biasX, eeprom.accelBiasX);
  writeCalibration(accel.biasY, eeprom.accelBiasY);
  writeCalibration(accel.biasZ, eeprom.accelBiasZ);
  writeCalibration(highG.biasX, eeprom.highGBiasX);
  writeCalibration(highG.biasY, eeprom.highGBiasY);
  writeCalibration(highG.biasZ, eeprom.highGBiasZ);
  writeCalibration(gyro.biasX, eeprom.gyroBiasX);
  writeCalibration(gyro.biasY, eeprom.gyroBiasY);
  writeCalibration(gyro.biasZ, eeprom.gyroBiasZ);

  digitalWrite(pins.beep, LOW);
  Serial.println(F("Calculated values are:"));
  Serial.print(F("accel.biasX: "));
  Serial.println(accel.biasX);
  Serial.print(F("accel.biasY: "));
  Serial.println(accel.biasY);
  Serial.print(F("accel.biasZ: "));
  Serial.println(accel.biasZ);
  Serial.print(F("highG.biasX: "));
  Serial.println(highG.biasX);
  Serial.print(F("highG.biasY: "));
  Serial.println(highG.biasY);
  Serial.print(F("highG.biasZ: "));
  Serial.println(highG.biasZ);
  Serial.print(F("gyro.biasX: "));
  Serial.println(gyro.biasX);
  Serial.print(F("gyro.biasY: "));
  Serial.println(gyro.biasY);
  Serial.print(F("gyro.biasZ: "));
  Serial.println(gyro.biasZ);

} // end accelerometer/gyro calibration

/**
 * @brief Calibration routine for the IMU magnetometer.
 */
void magCalibrate()
{

  // if the magnetic switch was enabled by the user, we need to reset the mag gain
  bool originalSettings = settings.magSwitchEnable;
  if (settings.magSwitchEnable)
  {
    settings.magSwitchEnable = false;
    beginMag();
  }

  // Clear out the FIFO
  getMag();
  delay(100);
  getMag();

  Serial.println(F("Magnetometer Calibration Mode"));
  // give a 20 second audible warning
  for (uint8_t i = 0; i < 20; i++)
  {
    digitalWrite(pins.beep, HIGH);
    delay(250);
    digitalWrite(pins.beep, LOW);
    delay(750);
  }

  // initialize the offsets
  int16_t maxMagX, maxMagY, maxMagZ;
  int16_t minMagX, minMagY, minMagZ;
  bool initMag = true;
  mag.biasX = mag.biasY = mag.biasZ = 0;

  Serial.println(F("Sampling Magnetometer"));
  // sample at 10Hz for 30seconds
  for (int16_t i = 0; i < 300; i++)
  {
    getMag();
    Serial.print(mag.rawX);
    Serial.print(',');
    Serial.print(mag.rawY);
    Serial.print(',');
    Serial.println(mag.rawZ);
    if (initMag)
    {
      maxMagX = minMagX = mag.rawX;
      maxMagY = minMagY = mag.rawY;
      maxMagZ = minMagZ = mag.rawZ;
      initMag = false;
    }
    if (mag.rawX > maxMagX)
    {
      maxMagX = mag.rawX;
    }
    if (mag.rawY > maxMagY)
    {
      maxMagY = mag.rawY;
    }
    if (mag.rawZ > maxMagZ)
    {
      maxMagZ = mag.rawZ;
    }
    if (mag.rawX < minMagX)
    {
      minMagX = mag.rawX;
    }
    if (mag.rawY < minMagY)
    {
      minMagY = mag.rawY;
    }
    if (mag.rawZ < minMagZ)
    {
      minMagZ = mag.rawZ;
    }
    delay(100);
  }

  // Calculate the bias
  Serial.print("minMagX: ");
  Serial.print(minMagX);
  Serial.print(", maxMagX: ");
  Serial.println(maxMagX);
  Serial.print("minMagY: ");
  Serial.print(minMagY);
  Serial.print(", maxMagY: ");
  Serial.println(maxMagY);
  Serial.print("minMagZ: ");
  Serial.print(minMagZ);
  Serial.print(", maxMagZ: ");
  Serial.println(maxMagZ);
  mag.biasX = (int16_t)(minMagX + (maxMagX - minMagX) / 2);
  mag.biasY = (int16_t)(minMagY + (maxMagY - minMagY) / 2);
  mag.biasZ = (int16_t)(minMagZ + (maxMagZ - minMagZ) / 2);

  // Write to EEPROM
  writeCalibration(mag.biasX, eeprom.magBiasX);
  writeCalibration(mag.biasY, eeprom.magBiasY);
  writeCalibration(mag.biasZ, eeprom.magBiasZ);

  // set the variables to proceed to bench-test mode
  settings.testMode = true;
  settings.magSwitchEnable = false;

  Serial.println(F("Magnetometer Calibration Complete!"));

  // beep three times to signal calibration complete
  digitalWrite(pins.beep, HIGH);
  delay(200);
  digitalWrite(pins.beep, LOW);
  delay(200);
  digitalWrite(pins.beep, HIGH);
  delay(200);
  digitalWrite(pins.beep, LOW);
  delay(200);
  digitalWrite(pins.beep, HIGH);
  delay(200);
  digitalWrite(pins.beep, LOW);
  delay(2000);

  if (originalSettings)
  {
    settings.magSwitchEnable = originalSettings;
    beginMag();
  }
} // end magnetometer calibration

/**
 * @brief Writes calibration values to EEPROM.
 * @param inValue The value to write to EEPROM.
 * @param eepromStart The starting position in EEPROM to write.
 */
void writeCalibration(int16_t inValue, byte eepromStart)
{

  calUnion.calValue = inValue;
  EEPROM.update(eepromStart, calUnion.calByte[0]);
  EEPROM.update(eepromStart + 1, calUnion.calByte[1]);
}

/**
 * @brief Determines the orientation of the flight computer and stores it in EEPROM.
 */
void setOrientation()
{

  byte orientCase = 0;

  // read from EEPROM the sensor orientations relative to the board
  accel.orientX = (char)EEPROM.read(eeprom.accel2BoardXAxis);
  accel.dirX = (uint8_t)EEPROM.read(eeprom.accel2BoardXSign);
  accel.orientY = (char)EEPROM.read(eeprom.accel2BoardYAxis);
  accel.dirY = (uint8_t)EEPROM.read(eeprom.accel2BoardYSign);
  accel.orientZ = (char)EEPROM.read(eeprom.accel2BoardZAxis);
  accel.dirZ = (uint8_t)EEPROM.read(eeprom.accel2BoardZSign);
  gyro.orientX = (char)EEPROM.read(eeprom.gyro2BoardXAxis);
  gyro.dirX = (uint8_t)EEPROM.read(eeprom.gyro2BoardXSign);
  gyro.orientY = (char)EEPROM.read(eeprom.gyro2BoardYAxis);
  gyro.dirY = (uint8_t)EEPROM.read(eeprom.gyro2BoardYSign);
  gyro.orientZ = (char)EEPROM.read(eeprom.gyro2BoardZAxis);
  gyro.dirZ = (uint8_t)EEPROM.read(eeprom.gyro2BoardZSign);
  mag.orientX = (char)EEPROM.read(eeprom.mag2BoardXAxis);
  mag.dirX = (uint8_t)EEPROM.read(eeprom.mag2BoardXSign);
  mag.orientY = (char)EEPROM.read(eeprom.mag2BoardYAxis);
  mag.dirY = (uint8_t)EEPROM.read(eeprom.mag2BoardYSign);
  mag.orientZ = (char)EEPROM.read(eeprom.mag2BoardZAxis);
  mag.dirZ = (uint8_t)EEPROM.read(eeprom.mag2BoardZSign);
  highG.orientX = (char)EEPROM.read(eeprom.highG2BoardXAxis);
  highG.dirX = (uint8_t)EEPROM.read(eeprom.highG2BoardXSign);
  highG.orientY = (char)EEPROM.read(eeprom.highG2BoardYAxis);
  highG.dirY = (uint8_t)EEPROM.read(eeprom.highG2BoardYSign);
  highG.orientZ = (char)EEPROM.read(eeprom.highG2BoardZAxis);
  highG.dirZ = (uint8_t)EEPROM.read(eeprom.highG2BoardZSign);

  // use the accelerometer to determine which of the 6 possible system orientations is present
  // Accelerometer Z-axis is pointed up
  if (abs(accel.biasZ) > abs(accel.biasY) && abs(accel.biasZ) > abs(accel.biasX) && accel.biasZ > 0)
  {
    if (accel.orientZ == 'Z')
    {
      orientCase = 1;
    } // Flight Computer Z-axis is pointed to the nose
    if (accel.orientY == 'Z')
    {
      orientCase = 3;
    } // Flight Computer Y-axis is pointed to the nose
    if (accel.orientX == 'Z')
    {
      orientCase = 5;
    }
  } // Flight Computer X-axis is pointed to the nose
  // Accelerometer Z-axis is pointed down
  if (abs(accel.biasZ) > abs(accel.biasY) && abs(accel.biasZ) > abs(accel.biasX) && accel.biasZ < 0)
  {
    if (accel.orientZ == 'Z')
    {
      orientCase = 2;
    } // Flight Computer Z-axis is pointed to the ground
    if (accel.orientY == 'Z')
    {
      orientCase = 4;
    } // Flight Computer Y-axis is pointed to the ground
    if (accel.orientX == 'Z')
    {
      orientCase = 6;
    }
  } // Flight Computer X-axis is pointed to the ground
  // Accelerometer Y-axis is pointed up
  if (abs(accel.biasY) > abs(accel.biasZ) && abs(accel.biasY) > abs(accel.biasX) && accel.biasY > 0)
  {
    if (accel.orientZ == 'Y')
    {
      orientCase = 1;
    } // Flight Computer Z-axis is pointed to the nose
    if (accel.orientY == 'Y')
    {
      orientCase = 3;
    } // Flight Computer Y-axis is pointed to the nose
    if (accel.orientX == 'Y')
    {
      orientCase = 5;
    }
  } // Flight Computer X-axis is pointed to the nose
  // Accelerometer Y-axis is pointed down
  if (abs(accel.biasY) > abs(accel.biasZ) && abs(accel.biasY) > abs(accel.biasX) && accel.biasY < 0)
  {
    if (accel.orientZ == 'Y')
    {
      orientCase = 2;
    } // Flight Computer Z-axis is pointed to the ground
    if (accel.orientY == 'Y')
    {
      orientCase = 4;
    } // Flight Computer Y-axis is pointed to the ground
    if (accel.orientX == 'Y')
    {
      orientCase = 6;
    }
  } // Flight Computer X-axis is pointed to the ground
  // Accelerometer X-axis is pointed up
  if (abs(accel.biasX) > abs(accel.biasZ) && abs(accel.biasX) > abs(accel.biasY) && accel.biasX > 0)
  {
    if (accel.orientZ == 'X')
    {
      orientCase = 1;
    } // Flight Computer Z-axis is pointed to the nose
    if (accel.orientY == 'X')
    {
      orientCase = 3;
    } // Flight Computer Y-axis is pointed to the nose
    if (accel.orientX == 'X')
    {
      orientCase = 5;
    }
  } // Flight Computer X-axis is pointed to the nose
  // Accelerometer X-axis is pointed down
  if (abs(accel.biasX) > abs(accel.biasZ) && abs(accel.biasX) > abs(accel.biasY) && accel.biasX < 0)
  {
    if (accel.orientZ == 'X')
    {
      orientCase = 2;
    } // Flight Computer Z-axis is pointed to the ground
    if (accel.orientY == 'X')
    {
      orientCase = 4;
    } // Flight Computer Y-axis is pointed to the ground
    if (accel.orientX == 'X')
    {
      orientCase = 6;
    }
  } // Flight Computer X-axis is pointed to the ground

  // set the orientation variables
  switch (orientCase)
  {

  case 1: // Flight Computer Z-axis is pointed to the nose

    Serial.println("Flight Computer Z-axis is pointed to the nose");

    // Do nothing, the axes are correctly aligned

    break;

  case 2: // Flight Computer Z-axis is pointed to the tail

    Serial.println("Flight Computer Z-axis is pointed to the tail");

    // all axes are still aligned

    // flip the direction of Z
    flipAxis('Z');

    // flip the direction of Y
    flipAxis('Y');

    break;

  case 3: // Flight Computer Y-axis is pointed to the nose

    Serial.println("Flight Computer Y-axis is pointed to the nose");

    // flip the direction of Z
    flipAxis('Z');

    // exchange Y and Z axes
    exchangeAxes('Y', 'Z');

    break;

  case 4: // Flight Computer Y-axis is pointed to the tail

    Serial.println("Flight Computer Y-axis is pointed to the tail");

    // flip the direction of Y
    flipAxis('Y');

    // exchange Y and Z axes
    exchangeAxes('Y', 'Z');

    break;

  case 5: // Flight Computer X-axis is pointed to the nose

    Serial.println("Flight Computer IMU X-axis is pointed to the nose");

    // flip the direction of Z
    flipAxis('Z');

    // exchange X and Z axes
    exchangeAxes('X', 'Z');

    break;

  case 6: // Flight Computer X-axis is pointed to the tail

    Serial.println("Flight Computer X-axis is pointed to the tail");

    // flip the direction of X
    flipAxis('X');

    // exchange X and Z axes
    exchangeAxes('X', 'Z');

    break;

  default: // error has occured
    Serial.println("Error in determining flight computer orientation");
    break;
  } // end case

  // correct IMU bias for 1G
  if (accel.orientX == 'Z')
  {
    accel.biasX -= accel.dirX * accel1G;
  }
  if (accel.orientY == 'Z')
  {
    accel.biasY -= accel.dirY * accel1G;
  }
  if (accel.orientZ == 'Z')
  {
    accel.biasZ -= accel.dirZ * accel1G;
  }

  // correct high-G bias for 1G, if there is a high-G accelerometer (high1G is set only then)
  if (sensors.highG != 0)
  {
    if (highG.orientX == 'Z')
    {
      highG.biasX -= highG.dirX * high1G;
    }
    if (highG.orientY == 'Z')
    {
      highG.biasY -= highG.dirY * high1G;
    }
    if (highG.orientZ == 'Z')
    {
      highG.biasZ -= highG.dirZ * high1G;
    }
  }

  // write to EEPROM
  // Accelerometer
  EEPROM.update(eeprom.accelXSign, accel.dirX);
  EEPROM.update(eeprom.accelXPtr, accel.orientX);
  EEPROM.update(eeprom.accelYSign, accel.dirY);
  EEPROM.update(eeprom.accelYPtr, accel.orientY);
  EEPROM.update(eeprom.accelZSign, accel.dirZ);
  EEPROM.update(eeprom.accelZPtr, accel.orientZ);
  // Gyroscope
  EEPROM.update(eeprom.gyroXSign, gyro.dirX);
  EEPROM.update(eeprom.gyroXPtr, gyro.orientX);
  EEPROM.update(eeprom.gyroYSign, gyro.dirY);
  EEPROM.update(eeprom.gyroYPtr, gyro.orientY);
  EEPROM.update(eeprom.gyroZSign, gyro.dirZ);
  EEPROM.update(eeprom.gyroZPtr, gyro.orientZ);
  // Magnetometer
  EEPROM.update(eeprom.magXSign, mag.dirX);
  EEPROM.update(eeprom.magXPtr, mag.orientX);
  EEPROM.update(eeprom.magYSign, mag.dirY);
  EEPROM.update(eeprom.magYPtr, mag.orientY);
  EEPROM.update(eeprom.magZSign, mag.dirZ);
  EEPROM.update(eeprom.magZPtr, mag.orientZ);
  // High-G Accelerometer
  EEPROM.update(eeprom.highGXSign, highG.dirX);
  EEPROM.update(eeprom.highGXPtr, highG.orientX);
  EEPROM.update(eeprom.highGYSign, highG.dirY);
  EEPROM.update(eeprom.highGYPtr, highG.orientY);
  EEPROM.update(eeprom.highGZSign, highG.dirZ);
  EEPROM.update(eeprom.highGZPtr, highG.orientZ);

  Serial.println("Calibration Complete!");

} // end void

void exchangeAxes(char axisA, char axisB)
{

  // accelerometer
  if (accel.orientX == axisA)
  {
    accel.orientX = axisB;
    if (accel.orientY == axisB)
    {
      accel.orientY = axisA;
    }
    if (accel.orientZ == axisB)
    {
      accel.orientZ = axisA;
    }
  }
  else if (accel.orientY == axisA)
  {
    accel.orientY = axisB;
    if (accel.orientX == axisB)
    {
      accel.orientX = axisA;
    }
    if (accel.orientZ == axisB)
    {
      accel.orientZ = axisA;
    }
  }
  else if (accel.orientZ == axisA)
  {
    accel.orientZ = axisB;
    if (accel.orientX == axisB)
    {
      accel.orientX = axisA;
    }
    if (accel.orientY == axisB)
    {
      accel.orientY = axisA;
    }
  }

  // gyroscope
  if (gyro.orientX == axisA)
  {
    gyro.orientX = axisB;
    if (gyro.orientY == axisB)
    {
      gyro.orientY = axisA;
    }
    if (gyro.orientZ == axisB)
    {
      gyro.orientZ = axisA;
    }
  }
  else if (gyro.orientY == axisA)
  {
    gyro.orientY = axisB;
    if (gyro.orientX == axisB)
    {
      gyro.orientX = axisA;
    }
    if (gyro.orientZ == axisB)
    {
      gyro.orientZ = axisA;
    }
  }
  else if (gyro.orientZ == axisA)
  {
    gyro.orientZ = axisB;
    if (gyro.orientX == axisB)
    {
      gyro.orientX = axisA;
    }
    if (gyro.orientY == axisB)
    {
      gyro.orientY = axisA;
    }
  }

  // magnetometer
  if (mag.orientX == axisA)
  {
    mag.orientX = axisB;
    if (mag.orientY == axisB)
    {
      mag.orientY = axisA;
    }
    if (mag.orientZ == axisB)
    {
      mag.orientZ = axisA;
    }
  }
  else if (mag.orientY == axisA)
  {
    mag.orientY = axisB;
    if (mag.orientX == axisB)
    {
      mag.orientX = axisA;
    }
    if (mag.orientZ == axisB)
    {
      mag.orientZ = axisA;
    }
  }
  else if (mag.orientZ == axisA)
  {
    mag.orientZ = axisB;
    if (mag.orientX == axisB)
    {
      mag.orientX = axisA;
    }
    if (mag.orientY == axisB)
    {
      mag.orientY = axisA;
    }
  }

  // high-G accelerometer
  if (highG.orientX == axisA)
  {
    highG.orientX = axisB;
    if (highG.orientY == axisB)
    {
      highG.orientY = axisA;
    }
    if (highG.orientZ == axisB)
    {
      highG.orientZ = axisA;
    }
  }
  else if (highG.orientY == axisA)
  {
    highG.orientY = axisB;
    if (highG.orientX == axisB)
    {
      highG.orientX = axisA;
    }
    if (highG.orientZ == axisB)
    {
      highG.orientZ = axisA;
    }
  }
  else if (highG.orientZ == axisA)
  {
    highG.orientZ = axisB;
    if (highG.orientX == axisB)
    {
      highG.orientX = axisA;
    }
    if (highG.orientY == axisB)
    {
      highG.orientY = axisA;
    }
  }
} // end exchangeAxes

void flipAxis(char axis)
{

  // accelerometer
  if (accel.orientX == axis)
  {
    accel.dirX *= -1;
  }
  if (accel.orientY == axis)
  {
    accel.dirY *= -1;
  }
  if (accel.orientZ == axis)
  {
    accel.dirZ *= -1;
  }
  // gyroscope
  if (gyro.orientX == axis)
  {
    gyro.dirX *= -1;
  }
  if (gyro.orientY == axis)
  {
    gyro.dirY *= -1;
  }
  if (gyro.orientZ == axis)
  {
    gyro.dirZ *= -1;
  }
  // magnetometer
  if (mag.orientX == axis)
  {
    mag.dirX *= -1;
  }
  if (mag.orientY == axis)
  {
    mag.dirY *= -1;
  }
  if (mag.orientZ == axis)
  {
    mag.dirZ *= -1;
  }
  // high-G accelerometer
  if (highG.orientX == axis)
  {
    highG.dirX *= -1;
  }
  if (highG.orientY == axis)
  {
    highG.dirY *= -1;
  }
  if (highG.orientZ == axis)
  {
    highG.dirZ *= -1;
  }

} // end flipAxis

/**
 * @brief Reads the orientation pointers from EEPROM.
 */
void readOrientation()
{

  // EEPROM stores the pointers and directions for the real-world axes as determined from calibration
  // accel.orientZ contains a character that represents the accelerometer axis that is aligned to the realworld Z-axis (rocket direction of travel)
  // thus if accel.orientZ == 'X' and accel.dirX == -1, then we know that the accelerometer X-axis is pointed towards the ground
  //*****************************************************
  // Accelerometer
  //*****************************************************
  // X-Axis
  accel.orientX = (char)EEPROM.read(eeprom.accelXPtr);
  accel.dirX = (int8_t)EEPROM.read(eeprom.accelXSign);
  switch (accel.orientX)
  {
  case 'X':
    accel.ptrX = &accel.rawX;
    accel.ptrXSign = &accel.dirX;
    break;
  case 'Y':
    accel.ptrY = &accel.rawX;
    accel.ptrYSign = &accel.dirX;
    break;
  case 'Z':
    accel.ptrZ = &accel.rawX;
    accel.ptrZSign = &accel.dirX;
    break;
  }
  // Y-Axis
  accel.orientY = (char)EEPROM.read(eeprom.accelYPtr);
  accel.dirY = (int8_t)EEPROM.read(eeprom.accelYSign);
  switch (accel.orientY)
  {
  case 'X':
    accel.ptrX = &accel.rawY;
    accel.ptrXSign = &accel.dirY;
    break;
  case 'Y':
    accel.ptrY = &accel.rawY;
    accel.ptrYSign = &accel.dirY;
    break;
  case 'Z':
    accel.ptrZ = &accel.rawY;
    accel.ptrZSign = &accel.dirY;
    break;
  }
  // Z-Axis
  accel.orientZ = (char)EEPROM.read(eeprom.accelZPtr);
  accel.dirZ = (int8_t)EEPROM.read(eeprom.accelZSign);
  switch (accel.orientZ)
  {
  case 'X':
    accel.ptrX = &accel.rawZ;
    accel.ptrXSign = &accel.dirZ;
    break;
  case 'Y':
    accel.ptrY = &accel.rawZ;
    accel.ptrYSign = &accel.dirZ;
    break;
  case 'Z':
    accel.ptrZ = &accel.rawZ;
    accel.ptrZSign = &accel.dirZ;
    break;
  }

  //*****************************************************
  // Gyroscope
  //*****************************************************
  // X-Axis
  gyro.orientX = (char)EEPROM.read(eeprom.gyroXPtr);
  gyro.dirX = (int8_t)EEPROM.read(eeprom.gyroXSign);
  switch (gyro.orientX)
  {
  case 'X':
    gyro.ptrX = &gyro.rawX;
    gyro.ptrXSign = &gyro.dirX;
    break;
  case 'Y':
    gyro.ptrY = &gyro.rawX;
    gyro.ptrYSign = &gyro.dirX;
    break;
  case 'Z':
    gyro.ptrZ = &gyro.rawX;
    gyro.ptrZSign = &gyro.dirX;
    break;
  }
  // Y-Axis
  gyro.orientY = (char)EEPROM.read(eeprom.gyroYPtr);
  gyro.dirY = (int8_t)EEPROM.read(eeprom.gyroYSign);
  switch (gyro.orientY)
  {
  case 'X':
    gyro.ptrX = &gyro.rawY;
    gyro.ptrXSign = &gyro.dirY;
    break;
  case 'Y':
    gyro.ptrY = &gyro.rawY;
    gyro.ptrYSign = &gyro.dirY;
    break;
  case 'Z':
    gyro.ptrZ = &gyro.rawY;
    gyro.ptrZSign = &gyro.dirY;
    break;
  }
  // Z-Axis
  gyro.orientZ = (char)EEPROM.read(eeprom.gyroZPtr);
  gyro.dirZ = (int8_t)EEPROM.read(eeprom.gyroZSign);
  switch (gyro.orientZ)
  {
  case 'X':
    gyro.ptrX = &gyro.rawZ;
    gyro.ptrXSign = &gyro.dirZ;
    break;
  case 'Y':
    gyro.ptrY = &gyro.rawZ;
    gyro.ptrYSign = &gyro.dirZ;
    break;
  case 'Z':
    gyro.ptrZ = &gyro.rawZ;
    gyro.ptrZSign = &gyro.dirZ;
    break;
  }

  //*****************************************************
  // Magnetometer
  //*****************************************************
  // X-Axis
  mag.orientX = (char)EEPROM.read(eeprom.magXPtr);
  mag.dirX = (int8_t)EEPROM.read(eeprom.magXSign);
  switch (mag.orientX)
  {
  case 'X':
    mag.ptrX = &mag.rawX;
    mag.ptrXSign = &mag.dirX;
    break;
  case 'Y':
    mag.ptrY = &mag.rawX;
    mag.ptrYSign = &mag.dirX;
    break;
  case 'Z':
    mag.ptrZ = &mag.rawX;
    mag.ptrZSign = &mag.dirX;
    break;
  }
  // Y-Axis
  mag.orientY = (char)EEPROM.read(eeprom.magYPtr);
  mag.dirY = (int8_t)EEPROM.read(eeprom.magYSign);
  switch (mag.orientY)
  {
  case 'X':
    mag.ptrX = &mag.rawY;
    mag.ptrXSign = &mag.dirY;
    break;
  case 'Y':
    mag.ptrY = &mag.rawY;
    mag.ptrYSign = &mag.dirY;
    break;
  case 'Z':
    mag.ptrZ = &mag.rawY;
    mag.ptrZSign = &mag.dirY;
    break;
  }
  // Z-Axis
  mag.orientZ = (char)EEPROM.read(eeprom.magZPtr);
  mag.dirZ = (int8_t)EEPROM.read(eeprom.magZSign);
  switch (mag.orientZ)
  {
  case 'X':
    mag.ptrX = &mag.rawZ;
    mag.ptrXSign = &mag.dirZ;
    break;
  case 'Y':
    mag.ptrY = &mag.rawZ;
    mag.ptrYSign = &mag.dirZ;
    break;
  case 'Z':
    mag.ptrZ = &mag.rawZ;
    mag.ptrZSign = &mag.dirZ;
    break;
  }

  //*****************************************************
  // High-G Accelerometer
  //*****************************************************
  // X-Axis
  highG.orientX = (char)EEPROM.read(eeprom.highGXPtr);
  highG.dirX = (int8_t)EEPROM.read(eeprom.highGXSign);
  switch (highG.orientX)
  {
  case 'X':
    highG.ptrX = &highG.rawX;
    highG.ptrXSign = &highG.dirX;
    break;
  case 'Y':
    highG.ptrY = &highG.rawX;
    highG.ptrYSign = &highG.dirX;
    break;
  case 'Z':
    highG.ptrZ = &highG.rawX;
    highG.ptrZSign = &highG.dirX;
    break;
  }
  // Y-Axis
  highG.orientY = (char)EEPROM.read(eeprom.highGYPtr);
  highG.dirY = (int8_t)EEPROM.read(eeprom.highGYSign);
  switch (highG.orientY)
  {
  case 'X':
    highG.ptrX = &highG.rawY;
    highG.ptrXSign = &highG.dirY;
    break;
  case 'Y':
    highG.ptrY = &highG.rawY;
    highG.ptrYSign = &highG.dirY;
    break;
  case 'Z':
    highG.ptrZ = &highG.rawY;
    highG.ptrZSign = &highG.dirY;
    break;
  }
  // Z-Axis
  highG.orientZ = (char)EEPROM.read(eeprom.highGZPtr);
  highG.dirZ = (int8_t)EEPROM.read(eeprom.highGZSign);
  switch (highG.orientZ)
  {
  case 'X':
    highG.ptrX = &highG.rawZ;
    highG.ptrXSign = &highG.dirZ;
    break;
  case 'Y':
    highG.ptrY = &highG.rawZ;
    highG.ptrYSign = &highG.dirZ;
    break;
  case 'Z':
    highG.ptrZ = &highG.rawZ;
    highG.ptrZSign = &highG.dirZ;
    break;
  }
} // end readOrientation

/**
 * @brief User-guided calibration for the barometric pressure sensor.
 */
void baroCalibrate()
{

  char serialInput;

  // exit the beeping
  digitalWrite(pins.beep, LOW);
  beepCounter = 8;

  // Send user message
  Serial.println(F("Barometer Calibration mode initiated"));

  // clear the user input
  while (Serial.available() > 0)
  {
    serialInput = Serial.read();
  }

  // Display the user temperature instructions
  Serial.println(F("Enter the current ambient room temperature in degrees and tenths of a degree C, or enter e to exit: "));

  // wait for user input, then enter into the array
  while (Serial.available() == 0)
  {
    delay(100);
  }
  strPosn = 0;
  while (Serial.available() > 0)
  {
    serialInput = Serial.read();
    dataString[strPosn] = serialInput;
    strPosn++;
    if (serialInput == 'e')
    {
      Serial.println(F("Exiting calibration"));
      return;
    }
  }
  dataString[strPosn] = '\0';

  // parse the data
  float userInput = atof(dataString);
  Serial.print(F("User input: "));
  Serial.println(userInput, 1);
  Serial.println(F("Calibrating..."));

  // Sample Temperature Sensor
  float tempSum = 0.0F;
  baro.tempOffset = 0.0F;
  for (int i = 0; i < 300; i++)
  {
    getBaro();
    if (baro.newTemp)
    {
      tempSum += baro.temperature;
      baro.newTemp = false;
      Serial.println(baro.temperature, 1);
    }
    else
    {
      i--;
    }
    delayMicroseconds(baro.timeBtwnSamp);
  }

  baro.tempOffset = (tempSum / 300) - userInput;
  Serial.println(baro.tempOffset, 1);

  // Print the sample conditions
  Serial.println(F("Temperature calibration complete"));
  Serial.print(F("Sampled atmospheric temperature: "));
  Serial.println(tempSum / 300, 1);
  Serial.print("Temperature Offset: ");
  Serial.println(baro.tempOffset, 1);

  // Display the user temperature instructions
  Serial.println(F("Enter the current barometric pressure in hPa: "));

  // wait for user input, then enter into the array
  while (Serial.available() == 0)
  {
    delay(100);
  }
  strPosn = 0;

  while (Serial.available() > 0)
  {
    char input = Serial.read();
    dataString[strPosn] = input;
    strPosn++;
  }
  dataString[strPosn] = '\0';

  // parse the data
  userInput = atof(dataString);
  Serial.print(F("User input: "));
  Serial.println(userInput, 2);
  Serial.println(F("Calibrating..."));

  // Sample Barometer
  baro.pressOffset = 0.0F;
  float baroSum = 0.0F;
  for (int i = 0; i < 300; i++)
  {
    getBaro();
    if (baro.newSamp)
    {
      baroSum += baro.pressure;
      baro.newSamp = false;
      Serial.println(baro.pressure, 2);
    }
    else
    {
      i--;
    };
    delayMicroseconds(baro.timeBtwnSamp);
  }

  baro.pressOffset = (baroSum / 300) - userInput;

  // Print the sample conditions
  Serial.println(F("Barometer calibration Complete"));
  Serial.print(F("Sampled barometric pressure: "));
  Serial.println(baroSum / 300, 2);
  Serial.print("Barometric Offset: ");
  Serial.print(baro.pressOffset, 2);
  Serial.println(F(" hPa"));

  // write to eeprom
  floatUnion.val = baro.tempOffset;
  for (byte i = 0; i < 4; i++)
  {
    EEPROM.update(eeprom.baroTempOffset + i, floatUnion.bytes[i]);
  }
  floatUnion.val = baro.pressOffset;
  for (byte i = 0; i < 4; i++)
  {
    EEPROM.update(eeprom.baroPressOffset + i, floatUnion.bytes[i]);
  }

  delay(2000);

} // End of barometer calibration

/**
 * @brief User-guided calibration for active stabilization and return capability.
 */
void setCanardTrim()
{

  // Send instructions to the user
  Serial.println(F("Enter the servo number and direction of rotation.  Each entry will add/subtract 1 degree of Trim."));
  Serial.println(F("Ex: 2+ will move Servo #2 one degree clockwise, 6- will move Servo #6 one degree counter clockwise"));
  Serial.println(F("Send ee to exit"));
  // clear the buffer
  while (Serial.available() > 0)
  {
    trimServoChar = Serial.read();
  }

  // Set loop flags
  boolean errorFlag = false;
  boolean exitFlag = false;

  // loop and wait for the user to enter a value
  while (!exitFlag)
  {
    if (Serial.available() == 3)
    {

      // read data
      trimServoChar = Serial.read();
      trimDirectionChar = Serial.read();
      trimEndChar = Serial.read();

      // exit condition
      if (trimServoChar == 'e' || trimDirectionChar == 'e')
      {
        exitFlag = true;
      }

      // error check
      if (trimServoChar != '1' && trimServoChar != '2' && trimServoChar != '3' && trimServoChar != '4' && trimServoChar != '5' && trimServoChar != '6' && trimServoChar != '7' && trimServoChar != '8')
      {
        errorFlag = true;
      }
      if (trimDirectionChar != '+' && trimDirectionChar != '-')
      {
        errorFlag = true;
      }

      // update trim settings
      if (trimServoChar >= '1' && trimServoChar <= '8')
      {
        const byte servo = trimServoChar - '0';
        servoTrim[servo - 1] += (trimDirectionChar == '+' ? 1 : -1);
        Serial.print(F("Servo"));
        Serial.print(servo);
        Serial.print(F(" Trim: "));
        Serial.println(servoTrim[servo - 1]);
        wiggleServo(servo);
      }
    }

    else if (Serial.available() > 0)
    {
      Serial.print("Sent Bytes: ");
      Serial.println(Serial.available());
      while (Serial.available() > 0)
      {
        trimServoChar = (char)Serial.read();
        Serial.print(trimServoChar);
      }
      Serial.println(' ');
      errorFlag = true;
    }

    // report error message if needed
    if (!exitFlag && errorFlag)
    {
      Serial.println(F("Error: Send a valid two character string. Ex: 3+"));
      errorFlag = false;
    }

    delay(100);
  } // end trim update loop

  // update the EEPROM
  // servo 1's trim is at eeprom.servo1Trim, the others after it
  for (byte i = 0; i < 8; i++)
  {
    EEPROM.update(eeprom.servo1Trim + i, servoTrim[i]);
  }

  // display settings to user
  Serial.println(F("FINAL SERVO TRIM SETTINGS"));
  for (byte i = 0; i < 8; i++)
  {
    Serial.print(F("Servo"));
    Serial.print(i + 1);
    Serial.print(F(" Trim: "));
    Serial.println(servoTrim[i]);
  }
}

/**
 * @brief Wiggle the servo to calibrate.
 * @param servo The servo number to calibrate.
 */
void wiggleServo(byte servo)
{
  if (servo < 1 || servo > 8)
  {
    return;
  }
  PWMServo &wiggled = servos[servo - 1];
  wiggled.write(80);
  delay(250);
  wiggled.write(100);
  delay(250);
  wiggled.write(90 - servoTrim[servo - 1]);
} // end void wiggle servo+