/*
  SFPddm.cpp - SFPddm library

Copyright 2013 Luka Mustafa - Musti, musti@wlan-si.net

This file is part of the SFPddm library for Arduino

The SFPddm library is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your
option) any later version.

The SFPddm library is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the SFPddm library. If not, see http://www.gnu.org/licenses/.

*/

// Includes

#include "SFPddm.h"

// Definitions

#define INFOADDR 0x50 // addr A0/1
#define DDMADDR 0x51  // addr A2/3

// Uncomment if HW SFP connections are present for Enhanced options
//#define HWSFP

#ifdef HWSFP
// Hardware SFP connections
#define TX_DISABLE A2
#define TX_FAULT A3
#define RX_LOS 3 //int 1
#define RATE_SELECT A7
#define MOD_DEF_0 A6

#endif



// Constructor /////////////////////////////////////////////////////////////////

SFPddm::SFPddm(void)
{
  //reset error code
  _error = 0x00;
}

// Public Methods //////////////////////////////////////////////////////////////

// The function initializes the communication, checks if the module is present, retrieves necessary information
uint8_t SFPddm::begin(uint8_t sdaPin, uint8_t sclPin)
{
  _wire.init(sdaPin, sclPin);
  _wire.begin();

  //reset error
  _error = 0x00;

#ifdef HWSFP
  // configure pins
  pinMode(TX_DISABLE, OUTPUT);
  pinMode(TX_FAULT, INPUT);
  pinMode(RX_LOS, INPUT);
  pinMode(RATE_SELECT, OUTPUT);
  pinMode(MOD_DEF_0, INPUT);

  // test if the device is inserted
  if (digitalRead(MOD_DEF_0) != HIGH)
  {
    _error = 0xFF;
    return _error;
  }
#endif

  // test device communication and read modes
  _error |= I2Cread(INFOADDR, 92, 1, &supported);
  _error |= I2Cread(INFOADDR, 93, 1, &ddmmodes);
  // stop if not present
  if (_error)
  {
    return _error;
  }

  // if DDM mode is supported and externally callibrated
  if (supported & 0x10)
  {
    getCalibrationData();
  }

  return _error;
}

// The function returns the 128 bytes of information at address 0xA0
void SFPddm::getRawInfo(uint8_t addr, uint8_t *data)
{
  //implement when required
}

// The function returns an OR or all error codes detected, should be 0x00 if data is to be valid
// Can be used to check if the module is present
uint8_t SFPddm::getStatus()
{
  // Do a test write to register pointer.
  _error |= I2Cwrite(INFOADDR, 0x00);
  return _error;
}

// This function can be used to get the supported information
uint8_t SFPddm::getSupported()
{
  return supported;
}

// This function can be used to get the DDM supported control flags
uint8_t SFPddm::getDDMmodes()
{
  return ddmmodes;
}

// The function acquires the measurements and returns an error code if sth went wrong, 0x00 is OK
uint8_t SFPddm::readMeasurements()
{
  int i;
  //read diagnostic measurements registers 96-105 of 0xA2, store them in buffer
  _error |= I2Cread(DDMADDR, 96, 22, (byte *)&raw_buffer);

  //copy raw measurements to results union
  uint8_t *p_meas = (uint8_t *)&measdata;
  for (i = 0; i < 22; i += 2)
  {
    *p_meas++ = raw_buffer[i + 1];
    *p_meas++ = raw_buffer[i];
  }

//get raw values from hardware pins if enabled
// txfaultstate (bit 2)
// rxlosstate (bit 1)
#ifdef HWSFP
  //mask
  measdata.status &= 0xF9;
  //read
  if (digitalRead(TX_FAULT) == HIGH)
  {
    measdata.status |= 0x04;
  }
  //read
  if (digitalRead(RX_LOS) == HIGH)
  {
    measdata.status |= 0x02;
  }
#endif

  //calibration if external data
  if (supported & 0x10)
  {
    measdata.temperature = calibrateTemperature(measdata.temperature, cal_general.t_slope, cal_general.t_off);
    measdata.voltage = calibrateMeasurement(measdata.voltage, cal_general.v_slope, cal_general.v_off);
    measdata.TXcurrent = calibrateMeasurement(measdata.TXcurrent, cal_general.txc_slope, cal_general.txc_off);
    measdata.TXpower = calibrateMeasurement(measdata.TXpower, cal_general.txp_slope, cal_general.txp_off);
    measdata.RXpower = calibrateRXpower(measdata.RXpower, &cal_rxpower[0]);
  }

  return _error;
}

// The function gets the value of the control register (0xA2 memory, register 110)
uint8_t SFPddm::getControl()
{
  return measdata.status;
}
// The function sets the value of the control register (0xA2 memory, register 110)
void SFPddm::setControl(uint8_t data)
{
// Software control
#ifndef HWSFP
  //write the byte (not all bits are writable!)
  _error |= I2Cwrite(DDMADDR, 110, data);
#endif

// Hardware control if enabled
#ifdef HWSFP
  if ((data & 0x40) != 0x00)
  {
    digitalWrite(TX_DISABLE, HIGH);
  }
  if ((data & 0x10) != 0x00)
  {
    digitalWrite(TX_DISABLE, HIGH);
  }
#endif
}

// The function returns the temperature , signed.
int16_t SFPddm::getTemperature()
{
  return measdata.temperature;
}

// The function returns the supply voltage , unsigned
uint16_t SFPddm::getVoltage()
{
  return measdata.voltage;
}

// The function returns the supply current , unsigned
uint16_t SFPddm::getTXcurrent()
{
  return measdata.TXcurrent;
}

// The function returns the TX power , unsigned
uint16_t SFPddm::getTXpower()
{
  return measdata.TXpower;
}

// The function returns the RX power , unsigned
uint16_t SFPddm::getRXpower()
{
  return measdata.RXpower;
}

uint16_t SFPddm::getAlarms()
{
  return measdata.alarms;
}

uint16_t SFPddm::getWarnings()
{
  return measdata.warnings;
}

// Private Methods /////////////////////////////////////////////////////////////

// The function retrieves callibration values
void SFPddm::getCalibrationData()
{
  // buffer for data
  uint8_t calData[36];

  //read data
  _error |= I2Cread(DDMADDR, 56, 36, &calData[0]);
  //loop variable
  int i;

  //writing binary to the float register using pointers
  uint8_t *pRXcal = (uint8_t *)&cal_rxpower;

  for (i = 0; i < 20; i++)
  {
    //this goes from 0xA2 SFP bytes 56-75
    //write data to pointer location and decrement pointer
    //beware, low byte is LSB in Arduino
    *pRXcal = calData[19 - i];
    pRXcal++;
  }

  //writing to uint16_t register using pointers
  //creating a pointer
  uint8_t *pCal = (uint8_t *)&cal_general;

  for (i = 20; i < 36; i += 2)
  {
    //this goes from 0xA2 SFP bytes 76-91
    //write data to pointer location and increment pointer
    //beware od the endiness
    *pCal++ = calData[i + 1];
    *pCal++ = calData[i];
  }
}

// This function calibrates all values except RX power and temperature
uint16_t SFPddm::calibrateMeasurement(uint16_t rawdata, uint16_t slope, int16_t offset)
{
  int32_t temporary = slope;
  temporary *= rawdata;
  //safe to use signed for all function, values do not overflow
  int16_t result = ((temporary >> 8) + offset);

  return result;
}

// This function calibrates temperature
int16_t SFPddm::calibrateTemperature(int16_t rawdata, uint16_t slope, int16_t offset)
{
  int32_t temporary = slope;
  temporary *= rawdata;
  //safe to use signed for all function, values do not overflow
  int16_t result = ((temporary >> 8) + offset);

  return result;
}

uint16_t SFPddm::calibrateRXpower(uint16_t rawdata, float *calibrationRX)
{
  float temporary;

  temporary = calibrationRX[4] * rawdata;
  temporary += calibrationRX[3] * rawdata;
  temporary += calibrationRX[2] * rawdata;
  temporary += calibrationRX[1] * rawdata;
  temporary += calibrationRX[0]; //offset

  //Serial.print("RXcalibrated: ");
  //Serial.println(temporary);

  return (int16_t)temporary;
}

// I2C wrapper functions to support standard wiring library
uint8_t SFPddm::I2Cread(uint8_t address, uint8_t registerAddress, uint8_t numberBytes, uint8_t *dataBuffer)
{
  _wire.beginTransmission(address);
  _wire.write(registerAddress);
  _wire.endTransmission();
  uint8_t receivedBytes = _wire.requestFrom(address, (uint8_t)numberBytes); // need to cast int to avoid compiler warnings

  uint8_t *ptr = (uint8_t *)dataBuffer;
  while (_wire.available())
  {
    *ptr++ = _wire.read();
  }

  //checking for status of the transmission
  if (receivedBytes == numberBytes)
  {
    return 0x00;
  }
  else
  {
    return 0x00;
  }
}

uint8_t SFPddm::I2Cwrite(uint8_t address, uint8_t registerAddress, uint8_t data)
{
  _wire.beginTransmission(address);
  _wire.write(registerAddress);
  _wire.write(data);
  return _wire.endTransmission(); // returns 0 of communication ok
}

uint8_t SFPddm::I2Cwrite(uint8_t address, uint8_t registerAddress)
{
  _wire.beginTransmission(address);
  _wire.write(registerAddress);
  return _wire.endTransmission(); // returns 0 of communication ok
}
