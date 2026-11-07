/**
 * @file RFDx_Driver.cpp
 * @brief Driver for the RFD900 serial telemetry radio.
 *
 * @details
 * The radio is driven over the Serial2 UART at 57600 baud: beginRfd900() opens the port and
 * sendPktRfd900() writes each packet followed by a line ending. Frequency and power are set on
 * the radio itself, so setPolyMorphs() points those function pointers at dummy functions.
 */

#include <Arduino.h>

#include "RFDx_Driver.h"
#include "declarations.h"

#define radioPort Serial2
#define RADIO_UART_SPEED 57600

bool beginRfd900()
{

  boolean successFlag = true;

  radioPort.begin(RADIO_UART_SPEED);

  if (successFlag)
  {
    Serial.println("RFD900 Radio OK!");
  }

  return successFlag;
}

bool sendPktRfd900(uint8_t *data, uint8_t len)
{
  for (int i = 0; i < len; i++)
  {
    radioPort.write(data[i]);
    radioPort.flush();
  }
  radioPort.println();
  radioPort.flush();
  // set flag
  radioFunction = TX_MODE;
  txInProgress = true;
  return true;
}