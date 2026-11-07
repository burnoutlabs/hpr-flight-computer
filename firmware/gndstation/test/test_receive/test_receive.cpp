/**
 * @file test_receive.cpp
 * @brief The ground station receiving what the V4.7.2 flight computer sends.
 *
 * The packets are the flight computer's own, byte for byte (telemetry_golden.h, which
 * firmware/flightcomp's test_telemetry_packets keeps in step with Telemetry.cpp). Each one
 * goes through the ground station's real receive path: the emulated radio's FIFO,
 * radioRecvPkt(), the callsign check, and processPacket(). The tests check the decoded
 * values, the SD log rows, and, with FHSS, that the ground station tunes to the channels the
 * flight computer transmits on.
 */
#include <unity.h>

#include <stdio.h>

#include <string>
#include <vector>

#include "../../../flightcomp/test/telemetry_golden.h"
#include "gndstation_subset.h"

static const int GOLDEN_OFF_COUNT = sizeof(GOLDEN_FHSS_OFF) / sizeof(GOLDEN_FHSS_OFF[0]);
static const int GOLDEN_ON_COUNT = sizeof(GOLDEN_FHSS_ON) / sizeof(GOLDEN_FHSS_ON[0]);

// Start-up state, then the radio set up as setup() does it, on radio 2.
static void resetGroundStation(bool fhss, float freq, const char *callSign)
{
  shimRadio.reset();
  shimSetMicros(5000000UL);
  settings = UserSettings();
  strcpy(settings.callSign, callSign);
  radio2 = Radio();
  radio2.num = 2;
  radio2.fhss = fhss;
  radio2.frq = freq;
  activeRadio = &radio2;
  memset(rocketName, 0, sizeof(rocketName));
  event = 0;
  apogee = false;
  maxAltitude = maxVelocity = maxG = maxGpsAlt = 0;
  lastPacketNum = 0;
  signalEst = false;
  strPosn = 0;
  sdInit = true;
  sustainerFile = FsFile();
  boosterFile = FsFile();
  sustainerFileCreated = boosterFileCreated = false;
  sustainerPreFlightWrite = boosterPreFlightWrite = true;
  sustainerPostFlightWrite = boosterPostFlightWrite = true;
  radio2.status = radioBegin();
  setRadioFreq(activeRadio->frq);
  radioSetMode(RX_MODE);
}

static bool receive(const GoldenPacket &p) { return receivePacket(p.bytes, p.len); }

// The comma-separated fields of line `n` (0-based) of a file.
static std::vector<std::string> csvLine(const std::string &text, int n)
{
  size_t start = 0;
  for (int i = 0; i < n; i++)
  {
    start = text.find("\r\n", start) + 2;
  }
  std::string line = text.substr(start, text.find("\r\n", start) - start);
  std::vector<std::string> fields;
  size_t pos = 0;
  while (true)
  {
    size_t comma = line.find(',', pos);
    fields.push_back(line.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos));
    if (comma == std::string::npos)
    {
      break;
    }
    pos = comma + 1;
  }
  return fields;
}

void setUp() {}
void tearDown() {}

void test_preflight_packet_is_decoded()
{
  resetGroundStation(false, 433.250F, "KK4ELF");
  TEST_ASSERT_TRUE(receive(GOLDEN_FHSS_OFF[0]));
  TEST_ASSERT_EQUAL_UINT8(0, event);
  TEST_ASSERT_EQUAL_UINT8(1, gpsLock);
  TEST_ASSERT_EQUAL_UINT8(6, contCode);
  TEST_ASSERT_EQUAL_STRING("Golden Test Rocket", rocketName);
  TEST_ASSERT_EQUAL_INT(-123, baseAlt);
  TEST_ASSERT_EQUAL_INT(1234, baseGpsAlt);
  TEST_ASSERT_TRUE((float)34.123456 == gpsLatitude);
  TEST_ASSERT_TRUE((float)-117.654321 == gpsLongitude);
  TEST_ASSERT_EQUAL_INT(7, satNum);
}

void test_inflight_packet_is_decoded_and_logged()
{
  resetGroundStation(false, 433.250F, "KK4ELF");
  for (int i = 0; i < 7; i++) // 5 preflight, inflight packets 1 and 2
  {
    TEST_ASSERT_TRUE(receive(GOLDEN_FHSS_OFF[i]));
  }
  // the last sample of inflight packet 2 is the scenario's sample 7
  TEST_ASSERT_EQUAL_INT16(2, lastPacketNum);
  TEST_ASSERT_EQUAL_UINT8(1, event);
  TEST_ASSERT_EQUAL_INT16(7 * 37 - 300, velocity);
  TEST_ASSERT_EQUAL_INT16(7 * 113 - 50, alt);
  TEST_ASSERT_EQUAL_INT16(7 * 29 - 700, spin);
  TEST_ASSERT_EQUAL_INT16(7 * 7, offVert);
  TEST_ASSERT_EQUAL_INT16((int16_t)((9.80665F * 7 - 20.0F) * 33.41406087), accel);
  TEST_ASSERT_EQUAL_INT(1300 + 7 * 11, gpsAlt);
  TEST_ASSERT_TRUE((float)(34.123456 + 7 * 0.00001) == gpsLatitude);

  // file: header, preflight row, inflight header, then 4 rows per inflight packet
  const std::string &log = sustainerFile.text;
  std::vector<std::string> pre = csvLine(log, 1);
  TEST_ASSERT_EQUAL_STRING("Golden Test Rocket", pre[1].c_str());
  TEST_ASSERT_EQUAL_STRING("-123", pre[4].c_str());
  const char *times[4] = {"0.20", "0.25", "0.30", "0.35"};
  for (int k = 0; k < 4; k++)
  {
    int s = 4 + k;
    std::vector<std::string> row = csvLine(log, 7 + k);
    TEST_ASSERT_EQUAL_STRING("1", row[0].c_str());                                   // event
    TEST_ASSERT_EQUAL_STRING(times[k], row[1].c_str());                              // time
    TEST_ASSERT_EQUAL_STRING(std::to_string(s * 37 - 300).c_str(), row[3].c_str());  // velocity
    TEST_ASSERT_EQUAL_STRING(std::to_string(s * 113 - 50).c_str(), row[4].c_str());  // altitude
    TEST_ASSERT_EQUAL_STRING(std::to_string(s * 29 - 700).c_str(), row[5].c_str());  // spin
    TEST_ASSERT_EQUAL_STRING(std::to_string(1300 + 7 * 11).c_str(), row[7].c_str()); // gpsAlt
    TEST_ASSERT_EQUAL_STRING("2", row[11].c_str());                                  // packetNum
  }
}

void test_postflight_packet_is_decoded()
{
  resetGroundStation(false, 433.250F, "KK4ELF");
  // a preflight packet first: writePostflightData() reads past the start of dataString
  // when the rocket name is empty
  TEST_ASSERT_TRUE(receive(GOLDEN_FHSS_OFF[0]));
  const GoldenPacket &post = GOLDEN_FHSS_OFF[GOLDEN_OFF_COUNT - 2];
  TEST_ASSERT_EQUAL_UINT8(telemetry::POSTFLIGHT_PACKET_SIZE, post.len);
  TEST_ASSERT_TRUE(receive(post));
  TEST_ASSERT_EQUAL_UINT8(27, event);
  TEST_ASSERT_EQUAL_INT(4321, maxAltitude);
  TEST_ASSERT_EQUAL_INT(876, maxVelocity);
  TEST_ASSERT_EQUAL_INT(-1234, maxG);
  TEST_ASSERT_EQUAL_INT(4300, maxGpsAlt);
  TEST_ASSERT_EQUAL_UINT8(1, gpsLock);
  TEST_ASSERT_EQUAL_INT(1240, gpsAlt);
  TEST_ASSERT_TRUE((float)34.2 == gpsLatitude);
}

void test_packet_with_another_callsign_is_ignored()
{
  resetGroundStation(false, 433.250F, "ABC123");
  TEST_ASSERT_FALSE(receive(GOLDEN_FHSS_OFF[0]));
  TEST_ASSERT_EQUAL_STRING("", rocketName);
  TEST_ASSERT_EQUAL_UINT32(0, sustainerFile.text.size());
}

// With FHSS the ground station waits on the hailing channel, where the flight computer
// sends its sync packets.
void test_ground_station_hails_where_the_flight_computer_syncs()
{
  resetGroundStation(true, 915.100F, "KK4ELF");
  int syncs = 0;
  for (int i = 0; i < GOLDEN_ON_COUNT; i++)
  {
    if (GOLDEN_FHSS_ON[i].len == telemetry::SYNC_PACKET_SIZE)
    {
      TEST_ASSERT_TRUE(GOLDEN_FHSS_ON[i].freq == radio2.frq);
      syncs++;
    }
  }
  TEST_ASSERT_EQUAL_INT(4, syncs);
  TEST_ASSERT_TRUE(fabs(shimRadioFreqMHz() - radio2.frq) < 0.0001);
}

// #13: the ground station hails on the frequency it was configured with, the frequency the
// flight computer syncs on when configured the same.
void test_hail_frequency_is_the_configured_frequency()
{
  const float configured[] = {902.1F, 902.3F, 915.1F, 927.9F};
  for (float f : configured)
  {
    resetGroundStation(true, f, "KK4ELF");
    TEST_ASSERT_TRUE(radio2.frq == f);
    TEST_ASSERT_TRUE(fabs(shimRadioFreqMHz() - f) < 0.0001);
  }
}

// Hands the flight computer's FHSS transmissions over in order; the ground station hears
// one only when its radio is tuned to that packet's frequency. Returns the indices heard.
static std::vector<int> followFlight()
{
  std::vector<int> heard;
  for (int i = 0; i < GOLDEN_ON_COUNT; i++)
  {
    const GoldenPacket &p = GOLDEN_FHSS_ON[i];
    if (fabs(shimRadioFreqMHz() - p.freq) < 0.0001 && radio2.frq == p.freq)
    {
      TEST_ASSERT_TRUE(receive(p));
      heard.push_back(i);
    }
  }
  return heard;
}

// From the first sync packet on, the ground station follows the hop sequence and hears every
// data packet, except inflight packets 1 to 3: the flight computer does not hop at liftoff,
// so those go out on the last preflight channel, which the ground station has hopped past.
// Before the first sync it cannot know the sequence. Missed-packet timing (loop()) is not
// simulated here, so nothing re-tunes the radio between packets; that leaves the postflight
// packets, which the flight computer hops for, out of reach.
void test_ground_station_follows_the_flight_computer_hops()
{
  resetGroundStation(true, 915.100F, "KK4ELF");
  std::vector<int> heard = followFlight();
  // transmissions: pre 0-3, sync 4, pre 5, inflight 6-17, sync 18, inflight 19, post 20, sync 21, post 22, sync 23
  std::vector<int> expected = {4, 5};
  for (int i = 9; i <= 17; i++)
  {
    expected.push_back(i); // inflight packets 4 to 12
  }
  expected.push_back(19); // inflight packet 13
  TEST_ASSERT_EQUAL_INT(expected.size(), heard.size());
  for (size_t k = 0; k < expected.size(); k++)
  {
    TEST_ASSERT_EQUAL_INT(expected[k], heard[k]);
  }
  TEST_ASSERT_EQUAL_INT16(13, lastPacketNum);
}

int main(int, char **)
{
  UNITY_BEGIN();
  RUN_TEST(test_preflight_packet_is_decoded);
  RUN_TEST(test_inflight_packet_is_decoded_and_logged);
  RUN_TEST(test_postflight_packet_is_decoded);
  RUN_TEST(test_packet_with_another_callsign_is_ignored);
  RUN_TEST(test_ground_station_hails_where_the_flight_computer_syncs);
  RUN_TEST(test_hail_frequency_is_the_configured_frequency);
  RUN_TEST(test_ground_station_follows_the_flight_computer_hops);
  return UNITY_END();
}
