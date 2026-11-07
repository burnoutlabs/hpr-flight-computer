/**
 * @file SX127X_Driver.cpp
 * @brief Optimized drivers for the Semtech SX1276/7/8/9 LoRa packet radios over SPI.
 *
 * @details
 * Starts and configures the radio, sets output power and frequency (with a separate
 * configuration for the 70cm band), sends packets, switches operating modes, and clears the
 * interrupt flags after a transmission.
 */

#include <Arduino.h>

#include "SX127X_Driver.h"
#include "declarations.h"
#include "flightcomp.h"
#include "Bus_Mgmt.h"

// These are optimized drivers for the Semtech SX1276, SX1277, SX1278, and SX1279 packet radios
//-----------------------------------------------
// Written by SparkyVT, TRA #12111, NAR #85720, L3
//-----------Change Log--------------------------
// 19 Nov 23: Version 1 broken out from the previous Telemetry.ino file to enable other radios with external libraries
// 29 Sep 24: Updated to reduce some of the function calls
//----------------------------
// LIST OF FUNCTIONS & ROUTINES
//----------------------------
// beginSx127x(): starts the radio
// setPwrSx127x(): sets the power in dBm
// sendPktSx127x(): sends the packet
// setFreqSx127x(): sets the radio frequency
// setModeSx127x(): sets the radio mode
// sleepSx127x(): sets radio to sleep as an independent command
// clearFlagsSx127x(): clears the interrupt flags
// config70cmSx127x(): configures for 70cm frequency band

bool beginSx127x()
{

  radioBus.spiSet = SPISettings(10000000, MSBFIRST, SPI_MODE0);
  radioBus.cs = pins.radioCs;
  radioBus.writeMask = 0x80;
  radioBus.readMask = 0x00;
  radioBus.incMask = 0x00;
  startSPI(&radioBus, sensors.radioBusNum);

  uint8_t debugVal;
  bool successFlag = true;

  // Set interrupts
  pinMode(pins.radioIrq, INPUT);
  attachInterrupt(digitalPinToInterrupt(pins.radioIrq), clearIrq, RISING);

  // reset radio
  digitalWrite(pins.radioCs, HIGH);
  digitalWrite(pins.radioRst, HIGH);
  delay(10);
  digitalWrite(pins.radioRst, LOW);
  delayMicroseconds(100);
  digitalWrite(pins.radioRst, HIGH);
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
  delay(10);
  debugVal = read8(REG_OP_MODE);
  if (debugVal != SLEEP_MODE)
  {
    Serial.print("Set Sleep Mode Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;
  }

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

// Set DIO0 to fire when transmit is complete
#define REG_DIO_MAPPING1 0x40
  write8(REG_DIO_MAPPING1, 0x40);
  delay(10);
  debugVal = read8(REG_DIO_MAPPING1);
  if (debugVal != 0x40)
  {
    Serial.print("Set DIO0 Failed: ");
    Serial.println(debugVal, HEX);
  }

// clear interrupts
#define REG_IRQ_FLAGS 0x12
#define REG_IRQ_FLAGS_MASK 0x11
  write8(REG_IRQ_FLAGS_MASK, 0x00);
  write8(REG_IRQ_FLAGS, 0xFF);

  // if the frequency band is 70cm, then we need to reconfigure the radio
  if (settings.txFreq > 400.000F && settings.txFreq < 500.000F)
  {
    if (!config70cmSx127x())
    {
      successFlag = false;
    }
  }

  if (successFlag)
  {
    Serial.println("SX127X Radio OK!");
  }

  return successFlag;
}

void sleepSx127x() { setModeSx127x(SLEEP_MODE); }

bool setPwrSx127x(int8_t pwr)
{

#define regPaDac 0x4D
#define regPaConfig 0x09

  boolean successFlag = true;
  uint8_t debugVal;

  // set bus
  activeBus = &radioBus;

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
    // write8(regPaDac, 0b01010111);
    write8(regPaDac, 0x07);
    pwr -= 3;
    delay(10);
    debugVal = read8(regPaDac);
    if (debugVal != 0x07)
    {
      successFlag = false;
      Serial.print("Set RegPaDac Failed: ");
      Serial.println(debugVal, HEX);
    }
  }

  // disable PA_DAC if power is below 17
  else
  {
    write8(regPaDac, 0x04);
    delay(10);
    debugVal = read8(regPaDac);
    if (debugVal != 0x04)
    {
      successFlag = false;
      Serial.print("Set RegPaDac Failed: ");
      Serial.println(debugVal, HEX);
    }
  }

  // write to the power config register, 0x80 ensures that the PA_Boost pin is selected
  uint8_t radioPwr = (0x80 | (pwr - 2));
  // uint8_t radioPwr = 0b10001101;
  if (settings.testMode)
  {
    Serial.print("Power Set: ");
    Serial.println(radioPwr, HEX);
  }
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

bool sendPktSx127x(uint8_t *data, uint8_t len)
{

#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_PAYLOAD_LENGTH 0x22

  // set bus
  activeBus = &radioBus;

  // put radio in standby
  write8(REG_OP_MODE, STANDBY_MODE);
  radioMode = 1;

  // clear flags
  uint8_t myFlags = read8(REG_IRQ_FLAGS);
  if (myFlags != 0x00)
  {
    Serial.print("Prior TX Error: ");
    Serial.println(myFlags);
  }
  write8(REG_IRQ_FLAGS, 0x00);

  // set the FIFO pointer to start of the buffer
  write8(REG_FIFO_ADDR_PTR, 0x00);

  // write to FIFO
  burstWrite(REG_FIFO, data, len);

  // write the payload length
  write8(REG_PAYLOAD_LENGTH, len);

  // send the packet
  write8(REG_OP_MODE, TX_MODE);

  // set flag
  radioFunction = TX_MODE;

  // indicate the radio is transmitting
  txInProgress = true;

  return true;
}

bool config70cmSx127x()
{

  // NOTE: the maximum allowable bandwidth on 70cm Ham band is 100kHz.  To keep the same data rate
  // we need to halve the bandwidth but increase the bitrate.  SF6 is the only way to do this,
  // but it requires a special configuration

#define REG_DETECT_OPTIMIZE 0x31
#define REG_DETECTION_THRESHOLD 0x37

  // set bus
  activeBus = &radioBus;
  bool successFlag = true;

  // configure the modem to 62.5kHz bw, 4/6 cr, implicit header
  uint8_t modemConfig = 0b01100101;
  write8(REG_MODEM_CONFIG1, modemConfig);
  delay(10);
  uint8_t debugVal = read8(REG_MODEM_CONFIG1);
  if (debugVal != modemConfig)
  {
    Serial.print("Config1 Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;
  }

  // configure the modem to sf6, normal TX mode, CRC on, RX MSB 0
  modemConfig = 0b01100100;
  write8(REG_MODEM_CONFIG2, modemConfig);
  delay(10);
  debugVal = read8(REG_MODEM_CONFIG2);
  if (debugVal != modemConfig)
  {
    Serial.print("Config2 Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;
  }

  // Set the bit field DetectionOptimize of register RegLoRaDetectOptimize to value "0b101"
  modemConfig = 0b11100101;
  write8(REG_DETECT_OPTIMIZE, modemConfig);
  delay(10);
  debugVal = read8(REG_DETECT_OPTIMIZE);
  if (debugVal != modemConfig)
  {
    Serial.print("RegDectectOptimize Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;
  }

  // Write 0x0C in the register REG_DETECTION_THRESHOLD
  modemConfig = 0x0C;
  write8(REG_DETECTION_THRESHOLD, modemConfig);
  delay(10);
  debugVal = read8(REG_DETECTION_THRESHOLD);
  if (debugVal != modemConfig)
  {
    Serial.print("RegDetectionThreshold Failed: ");
    Serial.println(debugVal, BIN);
    successFlag = false;
  }

  // set flag for implicit header
  implicitHdr = true;

  return successFlag;
}

bool setFreqSx127x(float freq)
{

  boolean successFlag = true;

  // set bus
  activeBus = &radioBus;

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

  return successFlag;
}

bool setModeSx127x(uint8_t mode)
{

#define REG_OP_MODE 0x01
#define REG_DIO_MAPPING1 0x40

  // set bus
  activeBus = &radioBus;

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

void clearFlagsSx127x()
{

  uint32_t txTime;
  txTime = micros() - txStartTime;
  txInProgress = false;

  // set bus
  activeBus = &radioBus;

  // read the IRQ flags
  uint8_t debugVal = read8(REG_IRQ_FLAGS);

  if (radioDebug && settings.testMode)
  {
    if (debugVal == 0x08)
    {
      Serial.print(", TX Done: ");
      Serial.print(debugVal, HEX);
      Serial.print(", TX Time: ");
      Serial.println(txTime);
    }
    else
    {
      Serial.print(", TX error: ");
      Serial.println(debugVal, BIN);
    }
  }

  // clear flag
  write8(REG_IRQ_FLAGS, 0xFF);

  // set the radio code
  radioMode = 1;

  noInterrupts();
  clearTx = false;
  interrupts();
}