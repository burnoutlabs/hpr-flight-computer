/**
 * @file Parse_Packet.cpp
 * @brief Handling of each telemetry packet the radios receive.
 *
 * The packets are decoded with the shared TelemetryProtocol library, the same code that
 * defines the format the flight computer sends (V4.7.2): a 6-character callsign, then the
 * event byte. loop() hands each received packet to processPacket(), which checks the
 * callsign and dispatches on the event to the parser for the packet type.
 */
#include <Arduino.h>
#include <TelemetryProtocol.h>

#include "declarations.h"
#include "LCD.h"
#include "Parse_Packet.h"
#include "Radio.h"
#include "SD_Card.h"

// Time between inflight samples, in hundredths of a second: the flight computer sends a
// packet every 200ms with 4 samples (pktInterval.inflight / pktInterval.samplesPerPkt).
static const uint16_t INFLIGHT_SAMPLE_INTERVAL_CS = 5;

bool processPacket()
{

  // every packet from our flight computer starts with its callsign
  if (!telemetry::callsignMatches(activeRadio->dataPacket, len, settings.callSign))
  {
    return false;
  }

  // set event flags
  event = (byte)activeRadio->dataPacket[telemetry::EVENT_OFFSET];
  if (settings.debugSerial)
  {
    if (event != telemetry::SYNC_MARKER)
    {
      Serial.println("-------Data Packet Received--------");
    }
    else
    {
      Serial.println("-------Sync Packet Received--------");
    }
    Serial.print("Packet on radio");
    Serial.print(activeRadio->num);
    Serial.print(", event: ");
    Serial.println(event);
    Serial.print("Length: ");
    Serial.println(len);
    Serial.print("TimeStamp: ");
    Serial.println(activeRadio->lastRx);
  }
  // determine if it is a sustainer or booster packet
  if (event < 21)
  {
    parseSustainer = true;
  }
  else if (event == 26 || event == 27 || event == 28)
  {
    parseSustainer = true;
  }
  else if (event != telemetry::SYNC_MARKER && event < 33)
  {
    parseBooster = true;
  }

  // Create the SD file
  if (parseSustainer && !sustainerFileCreated && sdInit)
  {
    createSustainerFile();
  }
  if (parseBooster && !boosterFileCreated && sdInit)
  {
    createBoosterFile();
  }

  // sync packet
  if (event == telemetry::SYNC_MARKER)
  {
    syncPkt(activeRadio->dataPacket);
  }

  // preflight packet
  else if (event == 0 || event == 30)
  {
    // parse the preFlight packet
    preflightPacket(activeRadio->dataPacket);
    // signal established
    signalEst = true;
    activeRadio->signalEst = true;
    // set lost signal time
    lostSignalTime = 2000000UL;
    // LCD
    preflightLcd();
  }

  // inflight packet
  else if (event < 26 && event != 28)
  {
    // set lost signal time
    lostSignalTime = 2000000UL;

    // write the last pre-flight packet to the SD card
    if (sdInit && sustainerPreFlightWrite && parseSustainer)
    {
      writePreflightData();
    }
    if (sdInit && boosterPreFlightWrite && parseBooster)
    {
      writePreflightData();
    }

    // parse the packet
    inflightPacket(activeRadio->dataPacket);

    // display to LCD
    inflightLcd();
  }

  // postflight packet
  else if (event == 26 || event == 27 || event == 29 || event == 31)
  {
    // set lost signal time
    lostSignalTime = 35000000UL;
    // parse the packet
    postflightPacket(activeRadio->dataPacket);
    // display to LCD
    postflightLcd();
  }

  // error packet
  else
  {
    errorLcd();
  }

  parseSustainer = false;
  parseBooster = false;
  return true;
}

void preflightPacket(byte rxPacket[])
{

  /*---------------------------------------------------
                PREFLIGHT PACKET
   -----------------------------------------------------*/

  // Read data from pre-flight packet
  telemetry::PreflightPacket pkt;
  if (!telemetry::decodePreflight(rxPacket, len, pkt))
  {
    return;
  }
  event = pkt.event;
  gpsLock = pkt.gnssFix;
  contCode = pkt.contCode;
  memcpy(rocketName, pkt.rocketName, sizeof(rocketName));
  baseAlt = pkt.baseAlt;
  baseGpsAlt = pkt.gpsAlt;
  gpsLatitude = pkt.latitude;
  gpsLongitude = pkt.longitude;
  satNum = pkt.satNum;

  // the flight computer hops before every preflight packet: follow it to the next channel
  if (activeRadio->fhss)
  {
    activeRadio->syncFreq = false;
    if (settings.debugSerial)
    {
      dispPktInfo();
      Serial.print(F("Hopping Freq: "));
    }
    activeRadio->chnlUsed = 0;
    hopFreq();
  }
  // capture the last good GPS coordinates to potentially store later in EEPROM
  if (gpsLock == 1)
  {
    lastGpsLat = gpsLatitude;
    lastGpsLon = gpsLongitude;
  }
}

void inflightPacket(byte rxPacket[])
{

  /*---------------------------------------------------
                INFLIGHT PACKET
   -----------------------------------------------------*/

  telemetry::InflightPacket pkt;
  if (!telemetry::decodeInflight(rxPacket, len, pkt))
  {
    return;
  }

  // the GPS data and packet number first, then the samples
  lastPacketNum = pkt.packetNum;

  if (debugSerial)
  {
    Serial.print(F("Inflight Packet Received, PktNum: "));
    Serial.println(lastPacketNum);
  }

  gpsAlt = pkt.gpsAlt;
  gpsLatitude = pkt.latitude;
  gpsLongitude = pkt.longitude;

  // determine GPS lock from coordinates
  if (fabs(gpsLongitude) > 0)
  {
    gpsLock = 1;
    lastGpsFix = micros();
  }
  else
  {
    gpsLock = 0;
  }

  // parse inflight packet of 4 samples
  strPosn = 0;
  for (byte i = 0; i < telemetry::INFLIGHT_SAMPLES; i++)
  {

    // parse the samples
    const telemetry::InflightSample &sample = pkt.samples[i];
    event = sample.event;
    if (!apogee && event >= 4 && event <= 6)
    {
      apogee = true;
    }
    // the packet carries no sample time: count samples from the first packet after liftoff
    sampleTime = (uint16_t)(((lastPacketNum - 1) * telemetry::INFLIGHT_SAMPLES + i) * INFLIGHT_SAMPLE_INTERVAL_CS);
    velocity = sample.velocity;
    if (!apogee && velocity > maxVelocity)
    {
      maxVelocity = velocity;
    }
    alt = sample.altitude;
    if (!apogee && alt > maxAltitude)
    {
      maxAltitude = alt;
    }
    spin = sample.roll;
    offVert = sample.offVert;
    accel = sample.accel;
    if (sdInit)
    {
      writeInflightData();
    }
    if (!apogee && accel > maxG)
    {
      maxG = accel;
    }
  }

  // write to the SD card
  if (parseSustainer)
  {
    sustainerFile.write(dataString, strPosn);
  }
  if (parseBooster)
  {
    boosterFile.write(dataString, strPosn);
  }

  // sync the SD card
  if (sdInit && parseSustainer)
  {
    sustainerFile.sync();
  }
  if (sdInit && parseBooster)
  {
    boosterFile.sync();
  }

  if (settings.debugSerial)
  {
    Serial.println("Inflight Data Written");
  }
  // set the next channel: the flight computer hops after every 3rd packet
  if (activeRadio->fhss)
  {
    activeRadio->syncFreq = false;
    if (debugSerial)
    {
      dispPktInfo();
    }
    activeRadio->lastHopTime = activeRadio->lastRx - (lastPacketNum % 3 * 200000UL);
    activeRadio->chnlUsed = 0;
    if (lastPacketNum % 3 == 0)
    {
      if (settings.debugSerial)
      {
        Serial.print(F("Hopping Freq: "));
      }
      hopFreq();
    }
  }

  // capture the last good GPS coordinates to potentially store later in EEPROM
  if (gpsLock == 1)
  {
    lastGpsLat = gpsLatitude;
    lastGpsLon = gpsLongitude;
  }
}

void postflightPacket(byte rxPacket[])
{

  /*---------------------------------------------------
                POSTFLIGHT PACKET
   -----------------------------------------------------*/
  telemetry::PostflightPacket pkt;
  if (!telemetry::decodePostflight(rxPacket, len, pkt))
  {
    return;
  }
  maxAltitude = pkt.maxAlt;
  maxVelocity = pkt.maxVel;
  maxG = pkt.maxG;
  maxGpsAlt = pkt.maxGpsAlt;
  gpsLock = pkt.gnssFix;
  gpsAlt = pkt.gpsAlt;
  gpsLatitude = pkt.latitude;
  gpsLongitude = pkt.longitude;

  // capture the last good GPS coordinates to potentially store later in EEPROM
  if (gpsLock == 1)
  {
    lastGpsLat = gpsLatitude;
    lastGpsLon = gpsLongitude;
  }

  // write to the SD card
  if (sdInit && parseSustainer && sustainerPostFlightWrite)
  {
    writePostflightData();
  }
  if (sdInit && parseBooster && boosterPostFlightWrite)
  {
    writePostflightData();
  }

  // sync the SD card
  if (sdInit && parseSustainer)
  {
    sustainerFile.flush();
  }
  if (sdInit && parseBooster)
  {
    boosterFile.flush();
  }

  // the flight computer hops before every postflight packet: follow it to the next channel
  if (activeRadio->fhss)
  {
    activeRadio->chnlUsed = 0;
    hopFreq();
  }
}
