/**
 * @file telemetry_scenarios.h
 * @brief Drives the real Telemetry.cpp on the host and records every packet it sends.
 *
 * Compiles globals.cpp and Telemetry.cpp against the shims in test/shim. The radio
 * driver pointers are pointed at stubs that record the frequency each packet goes out
 * on and its bytes. Include it from exactly one .cpp per test suite: it defines globals.
 *
 * runTelemetryScenario() resets the telemetry state, then sends what a flight sends:
 * preflight packets, inflight packets built one sample at a time, postflight packets,
 * and the sync packets that each phase asks for. The inputs are fixed, so the packets
 * are too; test_telemetry_packets compares them with the bytes recorded from the
 * flight computer before the protocol moved into the shared library.
 */
#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <PWMServo.h>
#include <TinyGPS++.h>

#include <string.h>

#include "../src/globals.cpp"

// Objects that flightcomp.cpp defines and the sources under test use.
TinyGPSPlus GPS;
PWMServo servos[8];

#include "../src/Telemetry.cpp"

// One transmission: the frequency the radio was set to when it was sent, and the bytes.
struct SentPacket
{
  float freq;
  uint8_t len;
  uint8_t bytes[64];
};

static const int MAX_SENT = 64;
static SentPacket sent[MAX_SENT];
static int sentCount = 0;
static float radioFreqNow = 0.0F;

static bool stubSetRadioFreq(float f)
{
  radioFreqNow = f;
  return true;
}

static bool stubRadioSendPkt(uint8_t *data, uint8_t len)
{
  if (sentCount < MAX_SENT)
  {
    SentPacket &p = sent[sentCount];
    p.freq = radioFreqNow;
    p.len = len;
    memset(p.bytes, 0, sizeof(p.bytes));
    memcpy(p.bytes, data, len < sizeof(p.bytes) ? len : sizeof(p.bytes));
  }
  sentCount++;
  return true;
}

// One pass of the flight loop's radio handling: build, send, then sync if asked for.
static void telemetryTick()
{
  buildPkt = true;
  buildTelmetryPkt();
  if (sendPkt)
  {
    sendTelemetryPkt();
    TX = false; // the TX-done interrupt
  }
  if (syncFreq)
  {
    syncPkt();
  }
}

// Returns the telemetry module and the globals it reads to their start-up values.
static void resetTelemetryState()
{
  FHSS.hopNow = true;
  FHSS.hopNum = 0;
  FHSS.nextHop = 1;
  FHSS.currentChnl = 0;
  FHSS.nextChnl = 0;
  FHSS.pktNum = 0;
  FHSS.gndPktNum = 0;
  FHSS.hailChnl = 0;
  memset(dataPacket, 0, sizeof(dataPacket));
  radio = RadioTelemetry();
  events = eventList();
  sampNum = 0;
  packetSamples = 4;
  pktPosn = 0;
  pktSize = 0;
  syncFreq = false;
  syncFlag = false;
  sendPkt = false;
  buildPkt = false;
  TX = false;
  radioDebug = false;
  settings.testMode = false;
  sentCount = 0;
  radioFreqNow = 0.0F;
  setRadioFreq = stubSetRadioFreq;
  radioSendPkt = stubRadioSendPkt;
  shimSetMicros(1000000UL);
}

// Sends 5 preflight, 13 inflight and 2 postflight packets, with the sync packets each phase
// asks for. txFreq is the configured frequency; FHSS applies only when fhss is true.
static void runTelemetryScenario(bool fhss, float txFreq)
{
  resetTelemetryState();
  strcpy(settings.callSign, "KK4ELF");
  memset(settings.rocketName, 0, sizeof(settings.rocketName));
  strcpy(settings.rocketName, "Golden Test Rocket");
  settings.fhss = fhss;
  settings.txFreq = txFreq;
  beginTelemetry();

  // Preflight
  events.preLiftoff = true;
  radio.event = 0;
  gnss.fix = 1;
  cont.reportCode = 6;
  radio.baseAlt = -123;
  for (int i = 0; i < 5; i++)
  {
    radio.GPSalt = (int16_t)(1234 + i);
    radio.satNum = (uint8_t)(7 + i);
    GPS.location.latitude = 34.123456 + i * 0.0001;
    GPS.location.longitude = -117.654321 - i * 0.0001;
    telemetryTick();
    shimAdvanceMicros(1000000UL);
  }

  // Inflight, built one sample at a time as the loop does
  events.preLiftoff = false;
  events.liftoff = true;
  events.inFlight = true;
  sampNum = 0;
  for (int s = 0; s < 13 * 4; s++)
  {
    radio.event = (uint8_t)(s < 20 ? 1 : (s < 40 ? 2 : 3));
    radio.vel = (int16_t)(s * 37 - 300);
    radio.alt = (int16_t)(s * 113 - 50);
    rollZ = s * 29 - 700;
    offVert = s * 7;
    accelNow = 9.80665F * (s % 9) - 20.0F;
    radio.GPSalt = (int16_t)(1300 + s * 11);
    GPS.location.latitude = 34.123456 + s * 0.00001;
    GPS.location.longitude = -117.654321 + s * 0.00002;
    telemetryTick();
    shimAdvanceMicros(50000UL);
  }

  // Postflight; the touchdown timer asks for a sync packet after each one
  events.inFlight = false;
  events.postFlight = true;
  radio.event = 27;
  radio.maxAlt = 4321;
  radio.maxVel = 876;
  radio.maxG = -1234;
  radio.maxGPSalt = 4300;
  for (int i = 0; i < 2; i++)
  {
    radio.GPSalt = (int16_t)(1240 - i);
    GPS.location.latitude = 34.2 + i * 0.001;
    GPS.location.longitude = -117.7 - i * 0.001;
    telemetryTick();
    shimAdvanceMicros(10000000UL);
  }
}
