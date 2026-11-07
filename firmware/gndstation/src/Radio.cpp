/**
 * @brief Functions and SPI transfer functions for radio control.
 */

/**
 * @section Functions
 *
 * - bool radioBegin(): Starts code and checks radio functionality.
 * - bool setRadioPwr(uint8_t pwr): Sets output power (0 - 20 dBm).
 * - bool setRadioFreq(float freq): Sets the radio frequency (expressed as MHz, e.g., 434.250).
 * - bool radioSendPkt(uint8_t* data, uint8_t len): Sends a radio packet from a byte array, with length len.
 * - bool radioRecvPkt(uint8_t* data): Receives a packet and places data from FIFO into array "data".
 * - bool radioSetMode(uint8_t mode): Sets the radio mode, mostly used for setting RX continuous mode in setup.
 *
 * @section SPI transfer functions
 *
 * - uint8_t read8(byte reg): Reads 8 bits from the specified register.
 * - bool write8(byte reg, byte data): Writes 8 bits of data to the specified register.
 * - bool burstRead(byte reg, uint8_t *data, byte len): Reads multiple bytes of data from consecutive registers.
 * - bool burstWrite(byte reg, uint8_t *data, byte len): Writes multiple bytes of data to consecutive registers.
 */

#include <Arduino.h>
#include <SPI.h>
#include <TelemetryProtocol.h>

#include "declarations.h"
#include "Radio.h"

//---------------------------
// 433 MHz Frequency List
//---------------------------
float radioFreq[9] = {433.250, 433.500, 433.625, 433.750, 433.875, 434.000, 434.125, 434.250, 434.400};

//---------------------------
// 915 MHz FHSS: the channel list, hop sequence and hailing channel are the flight
// computer's, from the shared TelemetryProtocol library
//---------------------------

byte radioMode = 0;
bool txInProgress = false;

bool radioBegin()
{

  uint8_t debugVal;
  boolean successFlag = true;

  delay(10);
  digitalWrite(activeRadio->rst, LOW);
  delayMicroseconds(100);
  digitalWrite(activeRadio->rst, HIGH);
  delay(10);

#define REG_OP_MODE 0x01
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_MODEM_CONFIG1 0x1D
#define REG_MODEM_CONFIG2 0x1E
#define REG_MODEM_CONFIG3 0x26
  uint8_t radioSetMode;

  // set radio to sleep mode
  write8(REG_OP_MODE, SLEEP_MODE);
  radioMode = 0;
  if (!activeRadio->enable)
  {
    return false;
  }
  delay(10);
  debugVal = read8(REG_OP_MODE);
  if (debugVal != SLEEP_MODE)
  {
    Serial.print("Set Sleep Mode Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;
  }

  // set radio power
  setRadioPwr(activeRadio->txPwr);

  // set radio to Long Range Mode
  radioSetMode = LORA_MODE;
  write8(REG_OP_MODE, radioSetMode);
  radioMode = 1;
  delay(10);
  debugVal = read8(REG_OP_MODE);
  if (debugVal != radioSetMode)
  {
    Serial.print("Set LoRa Mode Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;
  }

  // configure the modem to 125kHz bw, 4/5 cr, explicit header
  uint8_t modemConfig = 0b01110010;
  write8(REG_MODEM_CONFIG1, modemConfig);
  delay(10);
  debugVal = read8(REG_MODEM_CONFIG1);
  if (debugVal != modemConfig)
  {
    Serial.print("Config1 Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;
  }

  // configure the modem to sf7, normal TX mode, CRC on, RX MSB 0
  modemConfig = 0b01110100;
  write8(REG_MODEM_CONFIG2, modemConfig);
  delay(10);
  debugVal = read8(REG_MODEM_CONFIG2);
  if (debugVal != modemConfig)
  {
    Serial.print("Config2 Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;
  }

  // Set the Automatic Gain Control (AGC)
  modemConfig = 0x04;
  write8(REG_MODEM_CONFIG3, modemConfig);
  delay(10);
  debugVal = read8(REG_MODEM_CONFIG3);
  if (debugVal != modemConfig)
  {
    Serial.print("Config3 Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;
  }

  // set the preamble length to 8
  uint16_t preambleLength = 8;
  union
  {
    uint16_t val;
    byte bytes[2];
  } intUnion;
  intUnion.val = preambleLength;
  write8(REG_PREAMBLE_LSB, intUnion.bytes[0]);
  write8(REG_PREAMBLE_MSB, intUnion.bytes[1]);
  delay(10);
  debugVal = read8(REG_PREAMBLE_LSB);
  intUnion.bytes[0] = debugVal;
  debugVal = read8(REG_PREAMBLE_MSB);
  intUnion.bytes[1] = debugVal;
  if (intUnion.val != preambleLength)
  {
    Serial.print("Set Preamble Len Failed: ");
    Serial.println(intUnion.val, DEC);
    successFlag = false;
  }

// Set the FIFO buffer to the start
#define REG_FIFO_TX_BASE 0x0E
  write8(REG_FIFO_TX_BASE, 0x00);
  delay(10);
  debugVal = read8(REG_FIFO_TX_BASE);
  if (debugVal != 0x00)
  {
    Serial.print("Set FIFO TX Base Failed: ");
    Serial.println(debugVal, BIN);
  }

// Set the FIFO Rx Buffer base address to the start
#define REG_FIFO_RX_BASE 0x0F
  write8(REG_FIFO_RX_BASE, 0x00);
  delay(10);
  debugVal = read8(REG_FIFO_RX_BASE);
  if (debugVal != 0x00)
  {
    Serial.print("Set FIFO RX Base Failed: ");
    Serial.println(debugVal, BIN);
  }

  // Set radio to Standby Mode
  radioSetMode = STANDBY_MODE;
  write8(REG_OP_MODE, radioSetMode);
  delay(10);
  debugVal = read8(REG_OP_MODE);
  if (debugVal != (LORA_MODE | radioSetMode))
  {
    Serial.print("Set Standby Mode Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;
  }

// clear interrupts
#define REG_IRQ_FLAGS 0x12
#define REG_IRQ_FLAGS_MASK 0x11
  write8(REG_IRQ_FLAGS_MASK, 0x00);
  write8(REG_IRQ_FLAGS, 0xFF);

  // if FHSS, set the user frequency to the closest LoRa channel as the hail frequency
  if (activeRadio->fhss)
  {
    activeRadio->hailChnl = telemetry::hailChannel(activeRadio->frq);
    activeRadio->frq = getFreq(activeRadio->hailChnl);
    Serial.print("Radio ");
    Serial.print(activeRadio->num);
    Serial.print(" hail channel ");
    Serial.print(activeRadio->hailChnl);
    Serial.print(", freq: ");
    Serial.println(activeRadio->frq, 3);
  }

  return successFlag;
}

bool setRadioPwr(uint8_t pwr)
{

#define regPaDac 0x4D
#define regPaConfig 0x09

  boolean successFlag = true;
  uint8_t debugVal;

  // do this like RadioHead RFM95 does it
  // range is 2 to 20 dbm
  if (pwr > 20)
  {
    pwr = 20;
  }
  if (pwr < 2)
  {
    pwr = 2;
  }

  // enable PA_DAC if power is above 17
  if (pwr > 17)
  {
    write8(regPaDac, 0b01010111);
    pwr -= 3;
    delay(10);
    debugVal = read8(regPaDac);
    if (debugVal != 0b01010111)
    {
      successFlag = false;
      Serial.print("Set RegPaDac Failed: ");
      Serial.println(debugVal, HEX);
    }
  }

  // write to the power config register
  uint8_t radioPwr = (0x80 | (pwr - 2));
  Serial.print("Power Set: ");
  Serial.println(radioPwr, HEX);
  write8(regPaConfig, radioPwr);
  delay(10);
  debugVal = read8(regPaConfig);
  if (debugVal != radioPwr)
  {
    successFlag = false;
    Serial.print("Set RegPaConfig Failed: ");
    Serial.println(debugVal, HEX);
  }

  return successFlag;
}

bool radioRecvPkt(uint8_t *data)
{

// Check Error Flags
#define REG_IRQ_FLAGS 0x12
  uint8_t flags = read8(REG_IRQ_FLAGS);
  bool flagError = false;

  // good packet recieved
  if (flags == 0x40 || flags == 0x50)
  {
    Serial.print("Good Packet Recieved: ");
    Serial.println(flags, HEX);
  }

  // bad packet recieved
  else
  {
    Serial.print("IRQ Flag error: ");
    Serial.println(flags, BIN);
    flagError = true;
  }

  // clear the flags
  write8(REG_IRQ_FLAGS, 0xFF);

  // if there was an error, then do not process the packet
  if (flagError)
  {
    return false;
  }

// set the REG_FIFO_ADDR_PTR to REG_FIFO_RX_CURRENT_ADDR
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_RX_CURRENT_ADDR 0x10
  uint8_t val = read8(REG_FIFO_RX_CURRENT_ADDR);
  write8(REG_FIFO_ADDR_PTR, val);

// read the number of bytes recieved
#define REG_RX_NB_BYTES 0x13
  uint8_t pktLen = read8(REG_RX_NB_BYTES);
  len = pktLen;

  // read the packet
  burstRead(0x00, data, pktLen);

// get SNR
#define REG_PKT_SNR 0x19
  int8_t pktSnr = (int8_t)read8(REG_PKT_SNR);

// get Rssi
#define REG_PKT_RSSI_VALUE 0x1A
  pktRssi = read8(REG_PKT_RSSI_VALUE);

  // Calculate Rssi from datasheet
  if (pktSnr < 0)
  {
    pktRssi += pktSnr / 4;
  }
  else
  {
    pktRssi = (int16_t)(pktRssi * 16 / 15);
  }

  if (radio1Freq > 719.000)
  {
    pktRssi -= 157;
  }
  else
  {
    pktRssi -= 164;
  }

  return true;
}

float readRadioFreq()
{

#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08

  union
  {
    uint32_t val;
    uint8_t bytes[4];
  } ulongUnion;

  // read back the frequency
  ulongUnion.bytes[2] = read8(REG_FRF_MSB);
  ulongUnion.bytes[1] = read8(REG_FRF_MID);
  ulongUnion.bytes[0] = read8(REG_FRF_LSB);
  uint32_t frf = ulongUnion.val;

  float frq = 32 * (float)frf / 524288.0F;

  return frq;
}

bool setRadioFreq(float freq)
{

  boolean successFlag = true;

  // read the current mode
  uint8_t currentMode = read8(REG_OP_MODE);

  // set mode to standby
  write8(REG_OP_MODE, STANDBY_MODE);

  uint32_t frf = ((freq * 1000000.0) / 32000000) * 524288;
  union
  {
    uint32_t val;
    uint8_t bytes[4];
  } ulongUnion;

  ulongUnion.val = frf;

#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08

  write8(REG_FRF_MSB, ulongUnion.bytes[2]);
  write8(REG_FRF_MID, ulongUnion.bytes[1]);
  write8(REG_FRF_LSB, ulongUnion.bytes[0]);

  // delay(10);

  // read back the frequency
  ulongUnion.bytes[2] = read8(REG_FRF_MSB);
  ulongUnion.bytes[1] = read8(REG_FRF_MID);
  ulongUnion.bytes[0] = read8(REG_FRF_LSB);

  if (ulongUnion.val != frf)
  {
    successFlag = false;
    Serial.print("Set Freq Fail: ");
    Serial.println(ulongUnion.val);
  }

  // Set back to the previous mode
  radioSetMode(RX_MODE);

  return successFlag;
}

bool radioSetMode(uint8_t mode)
{

#define REG_OP_MODE 0x01
#define REG_DIO_MAPPING1 0x40

  // set radio mode
  write8(REG_OP_MODE, mode);
  radioMode = mode;

  delay(1);

  // check mode
  uint8_t val = read8(REG_OP_MODE);
  if (val != (LORA_MODE | mode))
  {
    Serial.print("Set Mode Failed: ");
    Serial.println(val, HEX);
    return false;
  }

  // set the IRQ to fire on TX sent
  if (mode == TX_MODE)
  {
    write8(REG_DIO_MAPPING1, 0x40);
    radioFunction = TX_MODE;
  }

  // set the IRQ to fire on RX complete
  if (mode == RX_MODE)
  {
    write8(REG_DIO_MAPPING1, 0x00);
    radioFunction = RX_MODE;
  }

  return true;
}

void hopFreq()
{

  uint8_t hopChnl;
  float freq;

  activeRadio->chnlUsed++;
  Serial.print("chnlUsed = ");
  Serial.println(activeRadio->chnlUsed);
  // up to two hops without hearing the flight computer, following the hop sequence as it does
  if (activeRadio->chnlUsed < 3)
  {
    hopChnl = activeRadio->nextChnl;
    activeRadio->nextHop = telemetry::nextHopIndex(activeRadio->nextHop, activeRadio->hailChnl);
    activeRadio->nextChnl = telemetry::HOP_SEQUENCE[activeRadio->nextHop];
    if (debugSerial)
    {
      Serial.print("Hopping to Channel: ");
      Serial.println(hopChnl);
    }
  }
  // then wait on the hailing channel for a sync packet
  else
  {
    hopChnl = activeRadio->hailChnl;
    if (debugSerial)
    {
      Serial.print(F("Moving to sync chnl: "));
      Serial.println(hopChnl);
      activeRadio->syncFreq = true;
    }
  }
  activeRadio->chnl1 = hopChnl;
  freq = getFreq(activeRadio->chnl1);
  activeRadio->frq = freq;
  setRadioFreq(freq);
  activeRadio->lastHopTime = micros();
  Serial.print("Timestamp: ");
  Serial.println(activeRadio->lastHopTime);
}

void syncPkt(byte rxPacket[])
{

  activeRadio->signalEst = true;
  activeRadio->syncFreq = false;
  lostSignalTime = 1400000UL;
  // parse packet: the flight computer's current channel, and where its next hop is in the
  // hop sequence
  telemetry::SyncPacket pkt;
  if (!telemetry::decodeSync(rxPacket, len, pkt) || pkt.nextHop < 0 || pkt.nextHop >= telemetry::HOP_SEQUENCE_LENGTH)
  {
    return;
  }
  activeRadio->chnl1 = pkt.currentChannel;
  activeRadio->nextHop = pkt.nextHop;
  activeRadio->nextChnl = telemetry::HOP_SEQUENCE[pkt.nextHop];
  activeRadio->chnlUsed = 0;
  hopFreq();
  if (debugSerial)
  {
    Serial.print("Moving to Channel ");
    Serial.println(activeRadio->nextChnl);
    dispPktInfo();
  }
}

void dispPktInfo()
{
  Serial.print(F("Current Chnl: "));
  Serial.print(activeRadio->chnl1);
  Serial.print(F(", "));
  Serial.print(getFreq(activeRadio->chnl1));
  Serial.println(F("MHz"));
  Serial.print(F("Next Chnl: "));
  Serial.print(activeRadio->nextChnl);
  Serial.print(F(", Next Freq: "));
  Serial.print(getFreq(activeRadio->nextChnl), 3);
  Serial.println("MHz");
  Serial.print(F("Next Hop: "));
  Serial.println(activeRadio->nextHop);
}

float getFreq(byte chnl)
{

  float frq;

  if (activeRadio->frq < 800.000F)
  {
    if (chnl >= sizeof(radioFreq) / sizeof(radioFreq[0]))
    {
      chnl = sizeof(radioFreq) / sizeof(radioFreq[0]) - 1;
    }
    frq = radioFreq[chnl];
  }

  else
  {
    if (chnl >= telemetry::FHSS_CHANNELS)
    {
      chnl = telemetry::FHSS_CHANNELS - 1;
    }
    frq = telemetry::FHSS_CHANNEL_MHZ[chnl];
  }

  return frq;
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
// SPI transfer functions
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
uint8_t read8(byte reg)
{

  byte val;

  // begin SPI transaction
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(activeRadio->cs, LOW);

  // Read data
  SPI.transfer(reg);
  val = SPI.transfer(0);

  // end SPI transaction
  digitalWrite(activeRadio->cs, HIGH);
  SPI.endTransaction();

  return val;
}

bool write8(byte reg, byte data)
{

  // begin SPI transaction
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(activeRadio->cs, LOW);

  // Send data
  SPI.transfer(WRITE_MASK | reg);
  SPI.transfer(data);

  // end SPI transaction
  digitalWrite(activeRadio->cs, HIGH);
  SPI.endTransaction();

  return true;
}

bool burstWrite(byte reg, uint8_t *data, byte len)
{

  // begin SPI transaction
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(activeRadio->cs, LOW);

  // Send data
  SPI.transfer(WRITE_MASK | reg);
  for (byte i = 0; i < len; i++)
  {
    SPI.transfer(*(data + i));
  }
  Serial.println(" ");

  // end SPI transaction
  digitalWrite(activeRadio->cs, HIGH);
  SPI.endTransaction();

  return true;
}

bool burstRead(byte reg, uint8_t *data, byte len)
{

  // begin SPI transaction
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(activeRadio->cs, LOW);

  // Read data
  SPI.transfer(reg);
  for (byte i = 0; i < len; i++)
  {
    *(data + i) = SPI.transfer(0);
  }

  // end SPI transaction
  digitalWrite(activeRadio->cs, HIGH);
  SPI.endTransaction();

  return true;
}
