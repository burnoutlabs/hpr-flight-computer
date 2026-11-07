/**
 * @file flight_state.h
 * @brief Every global the flight step can touch, as one list, for saving and comparing state.
 *
 * @details
 * FLIGHT_STATE_OBJECTS lists every object globals.cpp defines, in its order, and the file-scope
 * variables of Rotation.cpp (which flight_harness.h compiles into the test). It was generated from
 * globals.cpp; a global added there should be added here too.
 */
#pragma once

#include <string.h>

#include <string>
#include <vector>

#define FLIGHT_STATE_OBJECTS(X) \
  X(sensors) X(settings) X(accel) X(mag) X(gyro) X(highG) X(pyros) \
  X(events) X(resetEvents) X(fltTime) X(resetFltTime) X(pins) X(dataPacket) X(liftoffSync) \
  X(TXstartTime) X(implicitHdr) X(syncFreq) X(pktPosn) X(pktSize) X(sampNum) X(packetSamples) \
  X(gpsTransmit) X(TX) X(SDradioTX) X(radioMode) X(clearTX) X(buildPkt) X(syncFlag) X(sendPkt) \
  X(pktInterval) X(radio) X(cont) X(rotnOK) X(altOK) X(beep) X(pyroFire) X(fileClose) \
  X(boosterBurpTime) X(unitConvert) X(g) X(accelNow) X(maxG) X(gTrigger) X(high1G) X(filterFull) \
  X(highGsum) X(highGfilter) X(sizeHighGfilter) X(filterPosn) X(highGsmooth) X(adcConvert) \
  X(adcMidValue) X(calUnion) X(baro) X(baroTouchdown) X(touchdownTrigger) X(pressureAvg) \
  X(pressureAvg5) X(pressureSum) X(pressurePosn) X(magTrigger) X(magCalibrateMode) X(magRoll) \
  X(magOffVert) X(magPitch) X(magYaw) X(dx) X(dy) X(dz) X(yawY0) X(pitchX0) X(pitchX) X(yawY) \
  X(rollZ) X(offVert) X(rotationFault) X(lastRotn) X(rotnRate) X(accelVel) X(accelAlt) \
  X(maxVelocity) X(fusionVel) X(fusionAlt) X(thresholdVel) X(thresholdAlt) X(clearRailTime) \
  X(beep_counter) X(beepPosn) X(beep_delay) X(beepCode) X(beep_len) X(timeBeepStart) \
  X(timeLastBeepEnd) X(beepAlt) X(beepVel) X(beepCont) X(beepAlarm) X(livePyroAlarm) \
  X(writeStart) X(writeTime) X(maxWriteTime) X(writeThreshold) X(writeThreshCount) X(syncApogee) \
  X(syncMains) X(maxAltDigits) X(maxVelDigits) X(voltageDigits) X(altDigits) X(velDigits) X(n) \
  X(voltage) X(writeVolt) X(voltRate) X(voltReading) X(lastVolt) X(reportCode) X(postFlightCode) \
  X(gnss) X(ulongUnion) X(intUnion) X(floatUnion) X(userInput) X(servoTrim) \
  X(timeLastControl) X(kp) X(ki) X(kd) X(gpsEcho) X(radioDebug) X(txDataFile) X(cyclesBtwn) \
  X(sampleTime) X(sampleStart) X(sampleTimeCheck) X(debugTimer) X(radioFnctn) X(beginAccel) \
  X(beginGyro) X(beginMag) X(beginHighG) X(beginBaro) X(beginRadio) X(getAccel) X(getGyro) \
  X(getMag) X(getHighG) X(getBaro) X(setRadioFreq) X(setRadioPWR) X(radioSleep) X(radioSendPkt) \
  X(GNSSrestorDefaults) X(GNSSconfig) X(GNSSpowerSave) X(Quat) X(prevRollZ) X(quatRollZ) \
  X(fullRollZ) X(rawX) X(rawY) X(rawZ) X(cosZ) X(sinZ) \
  X(rollError) X(pitchError) X(yawError) X(rollPosn)

/** The bytes of every object in FLIGHT_STATE_OBJECTS, in order. */
inline void saveFlightState(std::vector<uint8_t> &bytes)
{
  size_t size = 0;
#define SIZE_OBJECT(object) size += sizeof(object);
  FLIGHT_STATE_OBJECTS(SIZE_OBJECT)
#undef SIZE_OBJECT
  bytes.resize(size);
  size_t at = 0;
#define SAVE_OBJECT(object)                                       \
  {                                                               \
    memcpy(bytes.data() + at, (const void *)&object, sizeof(object)); \
    at += sizeof(object);                                         \
  }
  FLIGHT_STATE_OBJECTS(SAVE_OBJECT)
#undef SAVE_OBJECT
}

inline std::vector<uint8_t> saveFlightState()
{
  std::vector<uint8_t> bytes;
  saveFlightState(bytes);
  return bytes;
}

inline void restoreFlightState(const std::vector<uint8_t> &bytes)
{
  size_t at = 0;
#define RESTORE_OBJECT(object)                            \
  {                                                       \
    memcpy((void *)&object, bytes.data() + at, sizeof(object)); \
    at += sizeof(object);                                 \
  }
  FLIGHT_STATE_OBJECTS(RESTORE_OBJECT)
#undef RESTORE_OBJECT
}

/** The name of the first object whose bytes differ between two saved states, or "" if none. */
inline std::string firstDifference(const std::vector<uint8_t> &a, const std::vector<uint8_t> &b)
{
  size_t at = 0;
  std::string name;
#define COMPARE_OBJECT(object)                                                   \
  if (name.empty() && memcmp(a.data() + at, b.data() + at, sizeof(object)) != 0) \
  {                                                                              \
    name = #object;                                                              \
  }                                                                              \
  at += sizeof(object);
  FLIGHT_STATE_OBJECTS(COMPARE_OBJECT)
#undef COMPARE_OBJECT
  return name;
}
