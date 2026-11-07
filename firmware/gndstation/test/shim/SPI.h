/**
 * @file SPI.h
 * @brief Host stand-in for Teensyduino's SPI library, wired to an emulated SX127X register file.
 *
 * Each transaction's first byte is the register address, with bit 7 set for a write; the
 * bytes after it read or write consecutive registers, except register 0, the FIFO, which
 * reads and writes at REG_FIFO_ADDR_PTR (0x0D) and advances it, as the chip does. Writing
 * REG_IRQ_FLAGS (0x12) clears the flags written as 1. Once the LoRa bit of REG_OP_MODE is set
 * it stays set. shimRadioReceive() puts a packet in the FIFO as if it had just arrived.
 * One register file serves every chip select, which is enough for tests using one radio.
 */
#pragma once

#include <stdint.h>
#include <string.h>

#define MSBFIRST 1
#define SPI_MODE0 0

struct SPISettings
{
  SPISettings(uint32_t, uint8_t, uint8_t) {}
};

struct ShimSX127X
{
  uint8_t reg[128];
  uint8_t fifo[256];
  void reset()
  {
    memset(reg, 0, sizeof(reg));
    memset(fifo, 0, sizeof(fifo));
  }
  uint8_t read(uint8_t addr)
  {
    if (addr == 0x00)
    {
      return fifo[reg[0x0D]++];
    }
    return reg[addr];
  }
  void write(uint8_t addr, uint8_t val)
  {
    if (addr == 0x00)
    {
      fifo[reg[0x0D]++] = val;
    }
    else if (addr == 0x12)
    {
      reg[addr] &= (uint8_t)~val;
    }
    else if (addr == 0x01)
    {
      reg[addr] = (uint8_t)(val | (reg[addr] & 0x80) | (val & 0x80));
    }
    else
    {
      reg[addr] = val;
    }
  }
};
inline ShimSX127X shimRadio;

// A packet received on the current frequency: FIFO from 0, length, and RxDone set.
inline void shimRadioReceive(const uint8_t *data, uint8_t len)
{
  memcpy(shimRadio.fifo, data, len);
  shimRadio.reg[0x10] = 0x00; // REG_FIFO_RX_CURRENT_ADDR
  shimRadio.reg[0x13] = len;  // REG_RX_NB_BYTES
  shimRadio.reg[0x12] = 0x40; // REG_IRQ_FLAGS: RxDone
}

// The frequency the emulated radio is tuned to, from RegFrf (0x06-0x08), in MHz.
inline double shimRadioFreqMHz()
{
  uint32_t frf = ((uint32_t)shimRadio.reg[0x06] << 16) | ((uint32_t)shimRadio.reg[0x07] << 8) | shimRadio.reg[0x08];
  return frf * 32.0 / 524288.0;
}

struct SPIClass
{
  bool first = true;
  bool writing = false;
  uint8_t addr = 0;
  void begin() {}
  void beginTransaction(SPISettings) { first = true; }
  void endTransaction() {}
  uint8_t transfer(uint8_t b)
  {
    if (first)
    {
      first = false;
      writing = b & 0x80;
      addr = b & 0x7F;
      return 0;
    }
    uint8_t val = 0;
    if (writing)
    {
      shimRadio.write(addr, b);
    }
    else
    {
      val = shimRadio.read(addr);
    }
    if (addr != 0x00)
    {
      addr++;
    }
    return val;
  }
};
inline SPIClass SPI;
