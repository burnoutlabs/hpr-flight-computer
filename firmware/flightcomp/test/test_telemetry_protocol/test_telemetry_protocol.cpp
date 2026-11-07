/**
 * @file test_telemetry_protocol.cpp
 * @brief Round trip: what the flight computer sends, the shared decoders read back.
 *
 * runTelemetryScenario() drives the real Telemetry.cpp; each packet it sends is decoded
 * with the TelemetryProtocol decoders the ground station uses, and the fields must equal
 * the values the scenario fed in. With FHSS on, each sync packet must name the channel the
 * last data packet used and the hop that the next data packet goes out on, and a receiver
 * that follows the hop sequence from a sync packet must land on every later data channel.
 */
#include <unity.h>

#include <math.h>
#include <stdio.h>

#include <TelemetryProtocol.h>

#include "telemetry_scenarios.h"

using namespace telemetry;

void setUp() {}
void tearDown() {}

static const char *CALLSIGN = "KK4ELF";

// Channel index of a frequency taken from the table
static int channelOf(float freq)
{
  for (int i = 0; i < FHSS_CHANNELS; i++)
  {
    if (fhssChannelMHz[i] == freq)
    {
      return i;
    }
  }
  return -1;
}

static bool isSync(const SentPacket &p) { return p.len == SYNC_PACKET_SIZE && p.bytes[EVENT_OFFSET] == SYNC_MARKER; }

static void checkPreflight(bool fhss)
{
  runTelemetryScenario(fhss, fhss ? 915.100F : 433.250F);
  int n = 0;
  for (int i = 0; i < sentCount; i++)
  {
    if (sent[i].len != PREFLIGHT_PACKET_SIZE)
    {
      continue;
    }
    TEST_ASSERT_TRUE(callsignMatches(sent[i].bytes, sent[i].len, CALLSIGN));
    PreflightPacket p;
    TEST_ASSERT_TRUE(decodePreflight(sent[i].bytes, sent[i].len, p));
    TEST_ASSERT_EQUAL_UINT8(0, p.event);
    TEST_ASSERT_EQUAL_UINT8(1, p.gnssFix);
    TEST_ASSERT_EQUAL_UINT8(6, p.contCode);
    TEST_ASSERT_EQUAL_STRING_LEN("Golden Test Rocket\0\0", p.rocketName, ROCKET_NAME_LENGTH);
    TEST_ASSERT_EQUAL_INT16(-123, p.baseAlt);
    TEST_ASSERT_EQUAL_INT16(1234 + n, p.gpsAlt);
    TEST_ASSERT_TRUE((float)(34.123456 + n * 0.0001) == p.latitude);
    TEST_ASSERT_TRUE((float)(-117.654321 - n * 0.0001) == p.longitude);
    TEST_ASSERT_EQUAL_UINT8(7 + n, p.satNum);
    n++;
  }
  TEST_ASSERT_EQUAL_INT(5, n);
}

static void checkInflight(bool fhss)
{
  runTelemetryScenario(fhss, fhss ? 915.100F : 433.250F);
  int n = 0;
  for (int i = 0; i < sentCount; i++)
  {
    if (sent[i].len != INFLIGHT_PACKET_SIZE)
    {
      continue;
    }
    TEST_ASSERT_TRUE(callsignMatches(sent[i].bytes, sent[i].len, CALLSIGN));
    InflightPacket p;
    TEST_ASSERT_TRUE(decodeInflight(sent[i].bytes, sent[i].len, p));
    for (int k = 0; k < INFLIGHT_SAMPLES; k++)
    {
      int s = n * INFLIGHT_SAMPLES + k; // the scenario's sample number
      const InflightSample &q = p.samples[k];
      TEST_ASSERT_EQUAL_UINT8(s < 20 ? 1 : (s < 40 ? 2 : 3), q.event);
      TEST_ASSERT_EQUAL_INT16(s * 37 - 300, q.velocity);
      TEST_ASSERT_EQUAL_INT16(s * 113 - 50, q.altitude);
      TEST_ASSERT_EQUAL_INT16(s * 29 - 700, q.roll);
      TEST_ASSERT_EQUAL_INT16(s * 7, q.offVert);
      float accelNowS = 9.80665F * (s % 9) - 20.0F;
      TEST_ASSERT_EQUAL_INT16((int16_t)(accelNowS * 33.41406087), q.accel);
    }
    int last = n * INFLIGHT_SAMPLES + INFLIGHT_SAMPLES - 1;
    TEST_ASSERT_EQUAL_INT16(n + 1, p.packetNum);
    TEST_ASSERT_EQUAL_INT16(1300 + last * 11, p.gpsAlt);
    TEST_ASSERT_TRUE((float)(34.123456 + last * 0.00001) == p.latitude);
    TEST_ASSERT_TRUE((float)(-117.654321 + last * 0.00002) == p.longitude);
    n++;
  }
  TEST_ASSERT_EQUAL_INT(13, n);
}

static void checkPostflight(bool fhss)
{
  runTelemetryScenario(fhss, fhss ? 915.100F : 433.250F);
  int n = 0;
  for (int i = 0; i < sentCount; i++)
  {
    if (sent[i].len != POSTFLIGHT_PACKET_SIZE)
    {
      continue;
    }
    TEST_ASSERT_TRUE(callsignMatches(sent[i].bytes, sent[i].len, CALLSIGN));
    PostflightPacket p;
    TEST_ASSERT_TRUE(decodePostflight(sent[i].bytes, sent[i].len, p));
    TEST_ASSERT_EQUAL_UINT8(27, p.event);
    TEST_ASSERT_EQUAL_INT16(4321, p.maxAlt);
    TEST_ASSERT_EQUAL_INT16(876, p.maxVel);
    TEST_ASSERT_EQUAL_INT16(-1234, p.maxG);
    TEST_ASSERT_EQUAL_INT16(4300, p.maxGpsAlt);
    TEST_ASSERT_EQUAL_UINT8(1, p.gnssFix);
    TEST_ASSERT_EQUAL_INT16(1240 - n, p.gpsAlt);
    TEST_ASSERT_TRUE((float)(34.2 + n * 0.001) == p.latitude);
    TEST_ASSERT_TRUE((float)(-117.7 - n * 0.001) == p.longitude);
    n++;
  }
  TEST_ASSERT_EQUAL_INT(2, n);
}

void test_preflight_round_trip_fhss_off() { checkPreflight(false); }
void test_preflight_round_trip_fhss_on() { checkPreflight(true); }
void test_inflight_round_trip_fhss_off() { checkInflight(false); }
void test_inflight_round_trip_fhss_on() { checkInflight(true); }
void test_postflight_round_trip_fhss_off() { checkPostflight(false); }
void test_postflight_round_trip_fhss_on() { checkPostflight(true); }

void test_no_sync_packets_without_fhss()
{
  runTelemetryScenario(false, 433.250F);
  for (int i = 0; i < sentCount; i++)
  {
    TEST_ASSERT_FALSE(isSync(sent[i]));
  }
}

// Each sync names the last data channel and the hop the next data packet uses.
void test_sync_round_trip_names_the_next_hop()
{
  runTelemetryScenario(true, 915.100F);
  uint8_t hail = hailChannel(915.100F);
  int syncs = 0;
  for (int i = 1; i + 1 < sentCount; i++)
  {
    if (!isSync(sent[i]))
    {
      continue;
    }
    TEST_ASSERT_TRUE(callsignMatches(sent[i].bytes, sent[i].len, CALLSIGN));
    SyncPacket p;
    TEST_ASSERT_TRUE(decodeSync(sent[i].bytes, sent[i].len, p));
    TEST_ASSERT_TRUE(sent[i].freq == fhssChannelMHz[hail]);
    TEST_ASSERT_EQUAL_INT(channelOf(sent[i - 1].freq), p.currentChannel);
    TEST_ASSERT_EQUAL_INT(hopSequence[p.nextHop], channelOf(sent[i + 1].freq));
    syncs++;
  }
  TEST_ASSERT_EQUAL_INT(3, syncs); // the last one ends the scenario
}

// A receiver that starts from the first sync and steps with nextHopIndex() lands on the
// channel of every data packet that opens a new hop afterwards.
void test_receiver_following_the_hop_sequence_lands_on_every_hop()
{
  runTelemetryScenario(true, 915.100F);
  uint8_t hail = hailChannel(915.100F);
  int first = -1;
  for (int i = 0; i < sentCount && first < 0; i++)
  {
    if (isSync(sent[i]))
    {
      first = i;
    }
  }
  TEST_ASSERT_TRUE(first > 0);
  SyncPacket p;
  TEST_ASSERT_TRUE(decodeSync(sent[first].bytes, sent[first].len, p));
  int16_t hop = p.nextHop;
  float lastData = sent[first - 1].freq;
  int hops = 0;
  for (int i = first + 1; i < sentCount; i++)
  {
    if (isSync(sent[i]) || sent[i].freq == lastData)
    {
      continue; // sync on the hail channel, or another packet on the same hop
    }
    TEST_ASSERT_EQUAL_INT(hopSequence[hop], channelOf(sent[i].freq));
    hop = nextHopIndex(hop, hail);
    lastData = sent[i].freq;
    hops++;
  }
  TEST_ASSERT_TRUE(hops >= 6);
}

void test_callsign_check()
{
  uint8_t pkt[SYNC_PACKET_SIZE] = {0};
  writeCallsign(pkt, "KK4ELF");
  pkt[EVENT_OFFSET] = SYNC_MARKER;
  TEST_ASSERT_TRUE(callsignMatches(pkt, sizeof(pkt), "KK4ELF"));
  TEST_ASSERT_FALSE(callsignMatches(pkt, sizeof(pkt), "KK4ELG"));
  TEST_ASSERT_FALSE(callsignMatches(pkt, EVENT_OFFSET, "KK4ELF"));
}

void test_decoders_reject_short_packets()
{
  uint8_t pkt[64] = {0};
  pkt[EVENT_OFFSET] = SYNC_MARKER;
  PreflightPacket pre;
  InflightPacket in;
  PostflightPacket post;
  SyncPacket sync;
  TEST_ASSERT_FALSE(decodePreflight(pkt, PREFLIGHT_PACKET_SIZE - 1, pre));
  TEST_ASSERT_FALSE(decodeInflight(pkt, INFLIGHT_PACKET_SIZE - 1, in));
  TEST_ASSERT_FALSE(decodePostflight(pkt, POSTFLIGHT_PACKET_SIZE - 1, post));
  TEST_ASSERT_FALSE(decodeSync(pkt, SYNC_PACKET_SIZE - 1, sync));
  TEST_ASSERT_TRUE(decodeSync(pkt, SYNC_PACKET_SIZE, sync));
  pkt[EVENT_OFFSET] = 0;
  TEST_ASSERT_FALSE(decodeSync(pkt, SYNC_PACKET_SIZE, sync));
}

void test_layout_constants()
{
  TEST_ASSERT_EQUAL_UINT8(50, INFLIGHT_TRAILER_OFFSET);
  TEST_ASSERT_EQUAL_UINT8(62, INFLIGHT_PACKET_SIZE);
}

void test_channel_list_is_902_1_to_927_9_in_200khz_steps()
{
  for (int i = 0; i < FHSS_CHANNELS; i++)
  {
    TEST_ASSERT_FLOAT_WITHIN(1e-4, 902.1 + 0.2 * i, fhssChannelMHz[i]);
  }
}

void test_hop_sequence_stays_in_the_channel_list()
{
  for (int i = 0; i < HOP_SEQUENCE_LENGTH; i++)
  {
    TEST_ASSERT_TRUE(hopSequence[i] < FHSS_CHANNELS);
  }
}

void test_next_hop_index_wraps_and_skips_the_hail_channel()
{
  TEST_ASSERT_EQUAL_INT16(2, nextHopIndex(1, 200));                       // no channel 200: no skip
  TEST_ASSERT_EQUAL_INT16(3, nextHopIndex(1, hopSequence[2]));            // skips one entry
  TEST_ASSERT_EQUAL_INT16(0, nextHopIndex(HOP_SEQUENCE_LENGTH - 1, 200)); // wraps
  TEST_ASSERT_EQUAL_INT16(1, nextHopIndex(HOP_SEQUENCE_LENGTH - 1, hopSequence[0]));
}

// #13: the hailing channel is the configured frequency's own channel.
void test_hail_channel_of_each_channel_frequency_is_that_channel()
{
  for (int i = 0; i < FHSS_CHANNELS; i++)
  {
    TEST_ASSERT_EQUAL_UINT8(i, hailChannel(fhssChannelMHz[i]));
  }
}

void test_hail_channel_is_the_nearest_channel()
{
  TEST_ASSERT_EQUAL_UINT8(65, hailChannel(915.05F)); // 915.1
  TEST_ASSERT_EQUAL_UINT8(64, hailChannel(914.96F)); // 914.9
  TEST_ASSERT_EQUAL_UINT8(0, hailChannel(900.0F));
  TEST_ASSERT_EQUAL_UINT8(FHSS_CHANNELS - 1, hailChannel(930.0F));
}

// #13: the flight computer sends its sync packets on the frequency it was configured with,
// and keeps that frequency in its settings.
void test_sync_packets_go_out_on_the_configured_frequency()
{
  const float configured[] = {902.1F, 902.3F, 915.1F, 927.9F};
  for (float f : configured)
  {
    runTelemetryScenario(true, f);
    TEST_ASSERT_TRUE(settings.txFreq == f);
    int syncs = 0;
    for (int i = 0; i < sentCount; i++)
    {
      if (isSync(sent[i]))
      {
        TEST_ASSERT_TRUE(sent[i].freq == f);
        syncs++;
      }
    }
    TEST_ASSERT_EQUAL_INT(4, syncs);
  }
}

int main(int, char **)
{
  UNITY_BEGIN();
  RUN_TEST(test_preflight_round_trip_fhss_off);
  RUN_TEST(test_preflight_round_trip_fhss_on);
  RUN_TEST(test_inflight_round_trip_fhss_off);
  RUN_TEST(test_inflight_round_trip_fhss_on);
  RUN_TEST(test_postflight_round_trip_fhss_off);
  RUN_TEST(test_postflight_round_trip_fhss_on);
  RUN_TEST(test_no_sync_packets_without_fhss);
  RUN_TEST(test_sync_round_trip_names_the_next_hop);
  RUN_TEST(test_receiver_following_the_hop_sequence_lands_on_every_hop);
  RUN_TEST(test_callsign_check);
  RUN_TEST(test_decoders_reject_short_packets);
  RUN_TEST(test_layout_constants);
  RUN_TEST(test_channel_list_is_902_1_to_927_9_in_200khz_steps);
  RUN_TEST(test_hop_sequence_stays_in_the_channel_list);
  RUN_TEST(test_next_hop_index_wraps_and_skips_the_hail_channel);
  RUN_TEST(test_hail_channel_of_each_channel_frequency_is_that_channel);
  RUN_TEST(test_hail_channel_is_the_nearest_channel);
  RUN_TEST(test_sync_packets_go_out_on_the_configured_frequency);
  return UNITY_END();
}
