/**
 * @file TelemetryProtocol.h
 * @brief The telemetry packet format and the 915MHz FHSS channel plan, shared by the flight
 * computer (which encodes) and the ground station (which decodes).
 *
 * @details
 * The wire format is the flight computer's, V4.7.2. Every packet starts with the 6-character
 * callsign; the byte after it is the flight event, or SYNC_MARKER for a sync packet. Integers
 * are little-endian two's complement and floats are IEEE-754 single precision, little-endian.
 *
 * | Packet     | Size | Bytes after the callsign (offsets from the start of the packet)        |
 * | ---------- | ---- | ---------------------------------------------------------------------- |
 * | Preflight  | 42   | 6 event, 7 GNSS fix, 8 continuity code, 9-28 rocket name,              |
 * |            |      | 29-30 base altitude, 31-32 GPS altitude, 33-36 latitude,               |
 * |            |      | 37-40 longitude, 41 satellites                                         |
 * | Inflight   | 62   | 4 samples of 11 bytes from 6: event, velocity, altitude, roll,         |
 * |            |      | off-vertical, acceleration (int16 each); then 50-51 packet number,     |
 * |            |      | 52-53 GPS altitude, 54-57 latitude, 58-61 longitude                    |
 * | Postflight | 26   | 6 event, 7-8 max altitude, 9-10 max velocity, 11-12 max G,             |
 * |            |      | 13-14 max GPS altitude, 15 GNSS fix, 16-17 GPS altitude,               |
 * |            |      | 18-21 latitude, 22-25 longitude                                        |
 * | Sync       | 10   | 6 SYNC_MARKER, 7 current channel, 8-9 hop-sequence index of next hop   |
 *
 * The encoders write only the bytes after the callsign; writeCallsign() sets the header once.
 * The decoders check the length and the callsign is checked separately by callsignMatches().
 *
 * Nothing here touches hardware, so it builds for the host tests as well as the Teensy boards.
 */
#pragma once

#include <stdint.h>

namespace telemetry
{

// Packet layout
const uint8_t CALLSIGN_LENGTH = 6;
const uint8_t EVENT_OFFSET = CALLSIGN_LENGTH;
const uint8_t SYNC_MARKER = 255;
const uint8_t ROCKET_NAME_LENGTH = 20;
const uint8_t INFLIGHT_SAMPLES = 4;
const uint8_t INFLIGHT_SAMPLE_SIZE = 11;
const uint8_t INFLIGHT_TRAILER_OFFSET = EVENT_OFFSET + INFLIGHT_SAMPLES * INFLIGHT_SAMPLE_SIZE;
const uint8_t PREFLIGHT_PACKET_SIZE = 42;
const uint8_t INFLIGHT_PACKET_SIZE = INFLIGHT_TRAILER_OFFSET + 12;
const uint8_t POSTFLIGHT_PACKET_SIZE = 26;
const uint8_t SYNC_PACKET_SIZE = 10;

// 915MHz FHSS channel plan
const uint8_t FHSS_CHANNELS = 130;
const uint16_t HOP_SEQUENCE_LENGTH = 2000;

/// Channel frequencies in MHz: 902.1 to 927.9 in 200kHz steps. Index n is a channel number.
extern const float FHSS_CHANNEL_MHZ[FHSS_CHANNELS];

/// Pseudo-random order in which data packets visit the channels.
extern const uint8_t HOP_SEQUENCE[HOP_SEQUENCE_LENGTH];

struct PreflightPacket
{
  uint8_t event;
  uint8_t gnssFix;
  uint8_t contCode;
  char rocketName[ROCKET_NAME_LENGTH];
  int16_t baseAlt;
  int16_t gpsAlt;
  float latitude;
  float longitude;
  uint8_t satNum;
};

struct InflightSample
{
  uint8_t event;
  int16_t velocity;
  int16_t altitude;
  int16_t roll;
  int16_t offVert;
  int16_t accel;
};

struct InflightPacket
{
  InflightSample samples[INFLIGHT_SAMPLES];
  int16_t packetNum;
  int16_t gpsAlt;
  float latitude;
  float longitude;
};

struct PostflightPacket
{
  uint8_t event;
  int16_t maxAlt;
  int16_t maxVel;
  int16_t maxG;
  int16_t maxGpsAlt;
  uint8_t gnssFix;
  int16_t gpsAlt;
  float latitude;
  float longitude;
};

struct SyncPacket
{
  uint8_t currentChannel;
  int16_t nextHop;
};

/// Writes the first CALLSIGN_LENGTH characters of callSign to the packet header.
void writeCallsign(uint8_t *pkt, const char *callSign);
/// True when the packet is long enough to carry an event and starts with callSign.
bool callsignMatches(const uint8_t *pkt, uint8_t len, const char *callSign);

/// Encoders: write the bytes after the callsign and return the packet size.
uint8_t encodePreflight(uint8_t *pkt, const PreflightPacket &p);
/// Writes sample `index` (0 to INFLIGHT_SAMPLES - 1) of an inflight packet.
void encodeInflightSample(uint8_t *pkt, uint8_t index, const InflightSample &s);
/// Writes the packet number and GPS data that close an inflight packet.
uint8_t encodeInflightTrailer(uint8_t *pkt, int16_t packetNum, int16_t gpsAlt, float latitude, float longitude);
uint8_t encodePostflight(uint8_t *pkt, const PostflightPacket &p);
uint8_t encodeSync(uint8_t *pkt, const SyncPacket &p);

/// Decoders: return false, leaving `out` unchanged, when len is shorter than the packet.
bool decodePreflight(const uint8_t *pkt, uint8_t len, PreflightPacket &out);
bool decodeInflight(const uint8_t *pkt, uint8_t len, InflightPacket &out);
bool decodePostflight(const uint8_t *pkt, uint8_t len, PostflightPacket &out);
bool decodeSync(const uint8_t *pkt, uint8_t len, SyncPacket &out);

/// The FHSS hailing channel for a configured frequency in MHz: the channel in FHSS_CHANNEL_MHZ
/// nearest to it, so a frequency on the list maps to its own channel.
uint8_t hailChannel(float freqMhz);
/// The hop-sequence index after `hop`, wrapping at the end and skipping one entry that is
/// the hailing channel, so data packets never go out on it.
int16_t nextHopIndex(int16_t hop, uint8_t hailChnl);

} // namespace telemetry
