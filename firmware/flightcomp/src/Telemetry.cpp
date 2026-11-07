/**
 * @file Telemetry.cpp
 *
 * @brief Telemetry packet building, transmission, and 915MHz FHSS frequency hopping.
 */

#include <Arduino.h>

#include <TelemetryProtocol.h>

#include "Telemetry.h"
#include "declarations.h"
#include "flightcomp.h"

/**
 * @section List of Functions & Routines
 *
 * - beginTelemetry(): Sets the FHSS hailing channel and writes the callsign into the packet header.
 * - buildTelmetryPkt(): Builds the pre-flight, in-flight, or post-flight telemetry packet.
 * - sendTelemetryPkt(): Transmits the built packet to the radio.
 * - hopTXfreq(): Hops to the next channel in the hop sequence when FHSS is active.
 * - syncPkt(): Sends a sync packet on the FHSS hailing frequency.
 *
 * The packet layout, the encoders, the channel list, the hop sequence and the hailing-channel
 * function are in the shared TelemetryProtocol library (firmware/common/lib), which the ground
 * station decodes with.
 */

/**
 * @section Radio Drivers
 *
 * The radio drivers are in SX127X_Driver.cpp, RFDx_Driver.cpp, and Device_Drivers_External.cpp.
 * Telemetry calls them through the function pointers declared in declarations.h, which
 * setPolyMorphs() (Device_Mgmt.cpp) points at the driver for the configured radio:
 * - beginRadio(): Starts the radio and checks that it responds.
 * - setRadioPWR(int8_t pwr): Sets output power (dBm).
 * - setRadioFreq(float freq): Sets the radio frequency (expressed in MHz, e.g., 434.250).
 * - radioSendPkt(uint8_t* data, uint8_t len): Sends a radio packet from a byte array with a specified length.
 * - radioSleep(): Puts the radio to sleep.
 */

/**
 * @section Change Log
 *
 * - 17 JUL 21: Initial breakout created.
 * - 18 AUG 22: Eliminated RadioHead library and wrote independent radio drivers.
 * - 20 NOV 22: Added callsign to the ham radio packet.
 * - 14 FEB 26: Changed the FHSS hopping strategy to pass the hop-sequence location in the sync packet, added mandatory callsign verification.
 */

/**
 * @section 900MHz FHSS Strategy
 *
 * This section describes the frequency-hopping spread spectrum (FHSS) strategy for 900MHz radios,
 * as implemented by the functions in this file:
 *
 * - **All flight phases**:
 *   - Every packet starts with the 6-character callsign, so the receiver can reject any packet that doesn't match it.
 *   - Channels are taken from a 130-channel list (902.1MHz to 927.9MHz in 200kHz steps).
 *   - The hailing channel is derived from the user-set frequency. Data packets follow a 2000-entry
 *     pseudo-random hop sequence, which skips the hailing channel.
 *   - Each data packet is sent on the next channel in the hop sequence.
 *   - The sync packet (10 bytes) is sent on the hailing channel. After the callsign it carries a
 *     sync marker (255), the current channel, and the hop-sequence location of the next hop, which both
 *     the transmitter and the receiver know.
 *
 * - **Preflight**:
 *   - Send a data packet every 1000ms (default), hopping to the next channel before each one.
 *   - Send a sync packet after every 4th data packet.
 *
 * - **Inflight**:
 *   - Send 5 packets per second (4 samples per packet).
 *   - Send 3 packets on one channel, then hop: a shift every 600ms.
 *   - Send a sync packet after every 12th packet: every 4th shift, once per 2.4s.
 *
 * - **Postflight**:
 *   - Send a data packet every 10 seconds, hopping to the next channel before each one.
 *   - A separate timer, started at touchdown or timeout, sends a sync packet every 10 seconds.
 *   - Continue until the system is turned off.
 *
 * The ground station (firmware/gndstation) decodes these packets and follows the hop sequence with
 * the same TelemetryProtocol library.
 */

using telemetry::fhssChannelMHz;
using telemetry::hopSequence;

struct
{
  bool hopNow = true;
  int16_t hopNum = 0;
  int16_t nextHop = 1;
  uint8_t currentChnl = 0;
  uint8_t nextChnl;
  int16_t pktNum = 0;
  int16_t gndPktNum = 0;
  uint8_t hailChnl = 0;
} FHSS;
float freq;
union
{
  uint32_t val = 0;
  uint8_t bytes[4];
} idUnion;

void beginTelemetry()
{

  // set the hailing frequency channel
  if (settings.fhss)
  {
    FHSS.hailChnl = telemetry::hailChannel(settings.txFreq);
    // we need to reset the user defined frequency to be the closest LoRa channel
    settings.txFreq = fhssChannelMHz[FHSS.hailChnl];
  }

  // set the packet header to the callsign
  telemetry::writeCallsign(dataPacket, settings.callSign);
}

void buildTelmetryPkt()
{

  //------------------------------------------------------------------
  //                  PRE-FLIGHT PACKET
  //------------------------------------------------------------------
  // send the preflight packet, 42 bytes
  if (events.preLiftoff)
  {

    // hop frequency
    if (settings.fhss)
    {
      hopTXfreq();
    }

    // build the data packet after the static callsign header
    telemetry::PreflightPacket pkt;
    pkt.event = radio.event;
    pkt.gnssFix = gnss.fix;
    pkt.contCode = cont.reportCode;
    memcpy(pkt.rocketName, settings.rocketName, sizeof(pkt.rocketName));
    pkt.baseAlt = radio.baseAlt;
    pkt.gpsAlt = radio.GPSalt;
    pkt.latitude = GPS.location.lat();
    pkt.longitude = GPS.location.lng();
    pkt.satNum = radio.satNum;

    // send the packet
    sendPkt = true;
    pktSize = telemetry::encodePreflight(dataPacket, pkt);
    if (radioDebug && settings.testMode)
    {
      if (settings.serialDebug == 3)
      {
        Serial.println("");
      }
      Serial.print(F("PreFlight Packet Sent"));
    }

    // set the FHSS flags
    if (settings.fhss)
    {
      FHSS.gndPktNum++;
      if (FHSS.gndPktNum % 4 == 0)
      {
        syncFreq = true;
        FHSS.gndPktNum = 0;
      }
    }
  }

  //------------------------------------------------------------------
  //                  IN-FLIGHT PACKET
  //------------------------------------------------------------------
  // build and send inflight packet
  // packet structure: 6 bytes callsign, 11 bytes per sample, 4 samples per packet, 12 bytes GPS & pktnum
  // packet requirements: 11 x 4 + 12 + 6 = 62 bytes per packet
  else if (events.inFlight)
  {

    // check to see if an SD card latency made us miss a sample
    uint32_t sampleTime = 0UL;
    sampleTime = micros();
    if (sampNum > 0 && sampleTime - radio.lastSampTime > 100000UL)
    {
      Serial.println("Sample Missed");
    }

    radio.lastSampTime = sampleTime;

    // update sample number
    sampNum++;
    pktPosn = (sampNum - 1) * 11 + 6;

    // hop frequency if needed
    if (settings.fhss && FHSS.hopNow && sampNum >= packetSamples)
    {
      hopTXfreq();
    }

    // Roll and off vertical data
    radio.roll = rollZ;
    radio.offVert = offVert;
    // Acceleration
    radio.accel = (int16_t)(accelNow * 33.41406087); // 33.41406087 = 32768 / 9.80665 / 100

    // write the sample into the packet
    telemetry::InflightSample sample;
    sample.event = radio.event;
    sample.velocity = radio.vel;
    sample.altitude = radio.alt;
    sample.roll = radio.roll;
    sample.offVert = radio.offVert;
    sample.accel = radio.accel;
    telemetry::encodeInflightSample(dataPacket, sampNum - 1, sample);

    // GPS & packet data collected once per packet at 12 bytes
    if (sampNum >= packetSamples)
    {

      // reset the sample number
      sampNum = 0;

      // update packet number
      radio.packetnum++;
      // packet number and GPS data
      pktSize = telemetry::encodeInflightTrailer(dataPacket, radio.packetnum, radio.GPSalt, GPS.location.lat(), GPS.location.lng());

      // send packet
      sendPkt = true;
      // debug output
      if (radioDebug && settings.testMode)
      {
        if (settings.serialDebug == 3)
        {
          Serial.println("");
        }
        Serial.print(F("InFlight Packet Sent "));
        Serial.print(radio.packetnum);
      }

      // set FHSS flags
      if (settings.fhss)
      {
        if (radio.packetnum % 3 == 0)
        {
          FHSS.hopNow = true;
        }
        if (radio.packetnum % 12 == 0)
        {
          syncFreq = true;
        }
      }
    }
  }

  //------------------------------------------------------------------
  //                  POST-FLIGHT PACKET
  //------------------------------------------------------------------
  // send post flight packet: 26 bytes per packet
  else if (events.postFlight)
  {

    // hop frequency
    if (settings.fhss)
    {
      hopTXfreq();
    }

    telemetry::PostflightPacket pkt;
    pkt.event = radio.event;
    pkt.maxAlt = radio.maxAlt;
    pkt.maxVel = radio.maxVel;
    pkt.maxG = radio.maxG;
    pkt.maxGpsAlt = radio.maxGPSalt;
    pkt.gnssFix = gnss.fix;
    pkt.gpsAlt = radio.GPSalt;
    pkt.latitude = GPS.location.lat();
    pkt.longitude = GPS.location.lng();

    // send the packet
    sendPkt = true;
    pktSize = telemetry::encodePostflight(dataPacket, pkt);
    if (settings.fhss)
    {
      syncFreq = true;
    }
    // debug output
    if (radioDebug && settings.testMode)
    {
      if (settings.serialDebug == 3)
      {
        Serial.println("");
      }
      Serial.print(F("Post Flight Packet Sent"));
    }

  } // end postFlight packet

  // turn off the flag now that we've processed the packet command
  buildPkt = false;
} // end radioSendPacket

void sendTelemetryPkt()
{

  static uint32_t TXlastStart = 0UL;

  TX = radioSendPkt(dataPacket, pktSize);
  TXstartTime = micros();
  if (events.liftoff && TX)
  {
    sdRadioTx = true;
  }
  // Serial debug
  if (radioDebug && settings.testMode)
  {
    if (TX)
    {
      Serial.print(F("...Success! Packet size "));
      Serial.print(pktSize);
    }
    else if (!TX)
    {
      Serial.print(F("..Failed! Packet size "));
      Serial.print(pktSize);
    }
    Serial.print(", time between packets ");
    Serial.print(TXstartTime - TXlastStart);
  }
  TXlastStart = TXstartTime;
  // reset the send flag
  sendPkt = false;
}

void hopTXfreq()
{

  // Serial debug
  if (radioDebug && settings.testMode)
  {
    if (settings.serialDebug == 3)
    {
      Serial.println("");
    }
    Serial.print("Hopping Freq: ");
    Serial.print(fhssChannelMHz[FHSS.nextChnl], 3);
    Serial.print(", HopNum: ");
    Serial.print(FHSS.nextHop);
    Serial.print(", timeStamp; ");
    Serial.print(micros());
  }

  // set the radio to the new frequency
  setRadioFreq(fhssChannelMHz[FHSS.nextChnl]);
  FHSS.currentChnl = FHSS.nextChnl;

  // identify the next channel in the hop sequence, skipping the hail channel
  FHSS.hopNum = FHSS.nextHop;
  FHSS.nextHop = telemetry::nextHopIndex(FHSS.nextHop, FHSS.hailChnl);

  // set the next channel nunber
  FHSS.nextChnl = hopSequence[FHSS.nextHop];

  // reset the flag hopFreq
  FHSS.hopNow = false;
}

void syncPkt()
{

  // get the hailing frequency
  float freq = fhssChannelMHz[FHSS.hailChnl];

  // hop to the hailing channel
  setRadioFreq(freq);

  // Serial debug
  if (radioDebug && settings.testMode)
  {
    Serial.print("---Sending Sync Packet: ");
    Serial.println(freq, 3);
    Serial.print("---Sync nextHop: ");
    Serial.print(FHSS.nextHop);
    Serial.print(", nextChnl: ");
    Serial.print(FHSS.nextChnl);
    Serial.print(", timeStamp; ");
    Serial.print(micros());
  }

  // sync marker, current channel, and the hop-sequence location of the next hop
  telemetry::SyncPacket pkt;
  pkt.currentChannel = FHSS.currentChnl;
  pkt.nextHop = FHSS.nextHop;
  uint8_t len = telemetry::encodeSync(dataPacket, pkt);

  // send packet if the current or next channel are not the hail channel
  radioSendPkt(dataPacket, len);
  syncFreq = false;
  noInterrupts();
  syncFlag = false;
  interrupts();
}