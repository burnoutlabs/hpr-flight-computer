/**
 * @file gndstation_subset.h
 * @brief Compiles the ground station's packet handling, radio and SD sources for the host.
 *
 * globals.cpp, SD_Card.cpp, Radio.cpp and Parse_Packet.cpp are included as they are, against
 * the shims in test/shim; the SX127X behind the SPI shim is an emulated register file. The
 * LCD screens are stubs. Include it from exactly one .cpp per test suite: it defines globals.
 *
 * receivePacket() hands a packet to the ground station the way loop() does: the radio reads
 * it from the FIFO, then processPacket() checks the callsign and handles it.
 */
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <TelemetryProtocol.h>

#include "../src/globals.cpp"

#include "../src/SD_Card.cpp"

#include "../src/Radio.cpp"

#include "../src/Parse_Packet.cpp"

// The LCD screens (LCD.cpp) do nothing here.
void startupLcd() {}
void preflightLcd() {}
void inflightLcd() {}
void postflightLcd() {}
void signalLostLcd() {}
void errorLcd() {}
void changeFreqLcd() {}

// loop()'s receive path for one packet; returns whether it was handled.
static bool receivePacket(const uint8_t *bytes, uint8_t length)
{
  shimRadioReceive(bytes, length);
  return radioRecvPkt(activeRadio->dataPacket) && processPacket();
}
