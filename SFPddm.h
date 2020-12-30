/*
  SFPddm.h - SFPddm library 
  
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

// ensure this library description is only included once
#ifndef SFPddm_h
#define SFPddm_h

#if defined(ENERGIA) // LaunchPad, FraunchPad and StellarPad specific
#include "Energia.h"
#elif defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(ARDUINO) && (ARDUINO >= 100) // Arduino 1.0x and 1.5x specific
#include "Arduino.h"
#elif defined(ARDUINO) && (ARDUINO < 100) // Arduino 23 specific
#include "WProgram.h"
#endif // end IDE

#include "SoftwareI2C.h"
//#include "Wire.h" //We replace this with the SoftwareI2C interface.

// library interface description
class SFPddm
{
  // user-accessible "public" interface
public:
  SFPddm(void);
  uint8_t begin(uint8_t sdaPin, uint8_t sclPin);
  void getRawInfo(uint8_t addr, uint8_t *data);
  uint8_t getStatus();
  uint8_t getSupported();
  uint8_t getDDMmodes();
  uint8_t readMeasurements();
  uint8_t getControl();
  void setControl(uint8_t data);
  int16_t getTemperature();
  uint16_t getVoltage();
  uint16_t getTXcurrent();
  uint16_t getTXpower();
  uint16_t getRXpower();
  uint16_t getAlarms();
  uint16_t getWarnings();

  // library-accessible "private" interface
private:
  void getCalibrationData();
  uint8_t I2Cread(uint8_t address, uint8_t registerAddress, uint8_t numberBytes, uint8_t *dataBuffer);
  uint8_t I2Cwrite(uint8_t address, uint8_t registerAddress, uint8_t data);
  uint8_t I2Cwrite(uint8_t address, uint8_t registerAddress);
  uint16_t calibrateMeasurement(uint16_t rawdata, uint16_t slope, int16_t offset);
  int16_t calibrateTemperature(int16_t rawdata, uint16_t slope, int16_t offset);
  uint16_t calibrateRXpower(uint16_t rawdata, float *calibrationRX);

  // Private variables
  SoftwareI2C _wire;

  // error variable
  uint8_t _error;
  // calibration data variables

  struct _cal
  {
    uint16_t txc_slope;
    int16_t txc_off;
    uint16_t txp_slope;
    int16_t txp_off;
    uint16_t t_slope;
    int16_t t_off;
    uint16_t v_slope;
    int16_t v_off;
  };
  _cal cal_general = {1, 0, 1, 0, 1, 0, 1, 0};

  float cal_rxpower[5];
  //raw measurement buffer
  uint8_t raw_buffer[22];
  //raw alarm and warnign buffer
  uint8_t raw_alarmwarning[6];
  //measurement values

  struct _meas
  {
    int16_t temperature; //reg A2/96-97
    uint16_t voltage;    //reg A2/98-99
    uint16_t TXcurrent;  //reg A2/100-101
    uint16_t TXpower;    //reg A2/102-103
    uint16_t RXpower;    //reg A2/104-105
    uint32_t RESERVED;   //reg A2/106-109
    uint8_t RESERVED2;   //reg A2/111 Intentional swapping due to eandiness adjustment in writing!
    uint8_t status;      //reg A2/110 Intentional swapping due to eandiness adjustment in writing!
    uint16_t alarms;     //reg A2/112-113
    uint16_t RESERVED3;  //reg A2/114-115
    uint16_t warnings;   //reg A2/116-117
  };
  _meas measdata = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  //supported modes flags
  uint8_t supported;
  //contains register A0/92

  //supported ddm modes flags
  uint8_t ddmmodes;
  //contains register A0/93
};

#endif
