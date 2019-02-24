/*********************************************************************************
Arduino Library for the Microchip EMC2301 RPM-based PWM fan controller.
I2C communication is performed using a custom I2C library.

Pg 12 of datasheet: The SMBus/I2C address is set at 0101_111(r/w)b (aka 47 or 0x2F)

Copyright (C) 2019 Soon Kiat Lau

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*********************************************************************************/

#ifndef _EMC2301_h
#define _EMC2301_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <I2C.h>


// Statuses/Errors returned by the functions in this class
typedef enum
{
  EMC2301_STATUS_OK         = 0,  // No problemo.
  EMC2301_STATUS_FAIL       = 1,  // Something went wrong.
  EMC2301_STATUS_INVALIDARG = 2   // The argument given to a function is invalid.
} EMC2301_STATUS;



class EMC2301
{
public:
  EMC2301(I2C * i2cWire);
  ~EMC2301();

  EMC2301_STATUS setPWMFrequencyBase(double frequencyKHz);
  EMC2301_STATUS setPWMFrequencyDivider(uint8_t divisor);
  EMC2301_STATUS toggleControlAlgorithm(bool enable);
  EMC2301_STATUS setTachMinRPM(uint16_t minRPM);
  EMC2301_STATUS setFanPoles(uint8_t poleCount);
  EMC2301_STATUS setDriveUpdatePeriod(uint16_t periodMs);
  EMC2301_STATUS toggleRampControl(bool enable);
  EMC2301_STATUS toggleGlitchFilter(bool enable);
  EMC2301_STATUS setDerivativeMode(uint8_t modeType);
  EMC2301_STATUS setControlErrRange(uint8_t errorRangeRPM);
  EMC2301_STATUS toggleSpinUpMax(bool enable);
  EMC2301_STATUS setSpinUpDrive(uint8_t drivePercent);
  EMC2301_STATUS setSpinUpTime(uint16_t timeMs);
  EMC2301_STATUS setControlMaxStep(uint8_t stepSize);
  EMC2301_STATUS setFanMinDrive(double minDrivePercent);
  EMC2301_STATUS setMinValidRPM(uint16_t minRPM);
  EMC2301_STATUS setRPMTarget(uint16_t targetRPM);
  EMC2301_STATUS toggleFan(bool enable);
  EMC2301_STATUS fetchFanSpeed();
  uint16_t getFanSpeed();

private:
  // Pg 12 of datasheet : The SMBus / I2C address is set at 0101_111(r / w)b(aka 47 or 0x2F)
  const uint8_t I2C_ADDRESS = 0x2F;

  // Assume that we use internal clock for tachometer
  const uint16_t TACHO_FREQUENCY = 32768;

  // 2-byte to be written to tacho target register to turn off fan
  const uint16_t TACHO_OFF = 0x1FFF << 3; // 1111 1111 1111 1000

  /******************************
   *     List of registers      *
   ******************************/
  const uint8_t EMC2301_REG_PWMBASEFREQ          = 0x2D;
  const uint8_t EMC2301_REG_FANSETTING           = 0x30;
  const uint8_t EMC2301_REG_PWMDIVIDE            = 0x31;
  const uint8_t EMC2301_REG_FANCONFIG1           = 0x32;
  const uint8_t EMC2301_REG_FANCONFIG2           = 0x33;
  const uint8_t EMC2301_REG_FANSPINUP            = 0x36;
  const uint8_t EMC2301_REG_FANMAXSTEP           = 0x37;
  const uint8_t EMC2301_REG_FANMINDRIVE          = 0x38;
  const uint8_t EMC2301_REG_FANVALTACHCOUNT      = 0x39;
  const uint8_t EMC2301_REG_TACHTARGETLSB        = 0x3C;
  const uint8_t EMC2301_REG_TACHTARGETMSB        = 0x3D;
  const uint8_t EMC2301_REG_TACHREADMSB          = 0x3E;
  const uint8_t EMC2301_REG_TACHREADLSB          = 0x3F;

  /* Registers that can have values written directly into them (i.e. the entire register is meant for a single number):
      EMC2301_REG_FANSETTING
      EMC2301_REG_PWMDIVIDE
      EMC2301_REG_FANMAXSTEP (Max 0b00111111 or 0x3F or 63)
      EMC2301_REG_FANMINDRIVE
      EMC2301_REG_FANVALTACHCOUNT (The final value from this register is 32 x (value in register))
      EMC2301_REG_TACHTARGETLSB and EMC2301_REG_TACHTARGETMSB (MUST write both LSB and MSB, with LSB written before MSB)
  */

  // EMC2301_REG_PWMBASEFREQ
  const uint8_t EMC2301_REG_PWMBASEFREQ_26KHZ  = 0x00;
  const uint8_t EMC2301_REG_PWMBASEFREQ_19KHZ  = 0x01;
  const uint8_t EMC2301_REG_PWMBASEFREQ_4KHZ   = 0x02;
  const uint8_t EMC2301_REG_PWMBASEFREQ_2KHZ   = 0x03;

  // EMC2301_REG_FANCONFIG1
  const uint8_t EMC2301_REG_FANCONFIG1_RPMCONTROL    = 0x80;
  const uint8_t EMC2301_REG_FANCONFIG1_MINRPM_CLEAR    = ~0x60;
  const uint8_t EMC2301_REG_FANCONFIG1_MINRPM_500    = 0x00;
  const uint8_t EMC2301_REG_FANCONFIG1_MINRPM_1000   = 0x20;
  const uint8_t EMC2301_REG_FANCONFIG1_MINRPM_2000   = 0x40;
  const uint8_t EMC2301_REG_FANCONFIG1_MINRPM_4000   = 0x60;
  const uint8_t EMC2301_REG_FANCONFIG1_FANPOLES_CLEAR  = ~0x18;
  const uint8_t EMC2301_REG_FANCONFIG1_FANPOLES_1    = 0x00;
  const uint8_t EMC2301_REG_FANCONFIG1_FANPOLES_2    = 0x08;
  const uint8_t EMC2301_REG_FANCONFIG1_FANPOLES_3    = 0x10;
  const uint8_t EMC2301_REG_FANCONFIG1_FANPOLES_4    = 0x18;
  const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_CLEAR    = ~0x07;
  const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_100    = 0x00;
  const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_200    = 0x01;
  const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_300    = 0x02;
  const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_400    = 0x03;
  const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_500    = 0x04;
  const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_800    = 0x05;
  const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_1200   = 0x06;
  const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_1600   = 0x07;

  // EMC2301_REG_FANCONFIG2
  const uint8_t EMC2301_REG_FANCONFIG2_RAMPCONTROL   = 0x40;
  const uint8_t EMC2301_REG_FANCONFIG2_GLITCHFILTER  = 0x20;
  const uint8_t EMC2301_REG_FANCONFIG2_DEROPT_CLEAR    = ~0x18;
  const uint8_t EMC2301_REG_FANCONFIG2_DEROPT_NONE   = 0x00;
  const uint8_t EMC2301_REG_FANCONFIG2_DEROPT_BASIC  = 0x08;
  const uint8_t EMC2301_REG_FANCONFIG2_DEROPT_STEP   = 0x10;
  const uint8_t EMC2301_REG_FANCONFIG2_DEROPT_BOTH   = 0x18;
  const uint8_t EMC2301_REG_FANCONFIG2_ERRRANGE_CLEAR  = ~0x06;
  const uint8_t EMC2301_REG_FANCONFIG2_ERRRANGE_0    = 0x00;
  const uint8_t EMC2301_REG_FANCONFIG2_ERRRANGE_50   = 0x02;
  const uint8_t EMC2301_REG_FANCONFIG2_ERRRANGE_100  = 0x04;
  const uint8_t EMC2301_REG_FANCONFIG2_ERRRANGE_200  = 0x06;

  // EMC2301_REG_FANSPINUP
  const uint8_t EMC2301_REG_FANSPINUP_NOKICK           = 0x20;
  const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_CLEAR      = ~0x1C;
  const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_30       = 0x00;
  const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_35       = 0x04;
  const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_40       = 0x08;
  const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_45       = 0x0C;
  const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_50       = 0x10;
  const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_55       = 0x14;
  const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_60       = 0x18;
  const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_65       = 0x1C;
  const uint8_t EMC2301_REG_FANSPINUP_SPINUPTIME_CLEAR   = ~0x03;
  const uint8_t EMC2301_REG_FANSPINUP_SPINUPTIME_250   = 0x00;
  const uint8_t EMC2301_REG_FANSPINUP_SPINUPTIME_500   = 0x01;
  const uint8_t EMC2301_REG_FANSPINUP_SPINUPTIME_1000  = 0x02;
  const uint8_t EMC2301_REG_FANSPINUP_SPINUPTIME_2000  = 0x03;

  // EMC2301_REG_FANMAXSTEP
  const uint8_t EMC2301_REG_FANMAXSTEP_MAX = 0b00111111;

  I2C *i2cWire_;
  uint8_t tachMinRPMMultiplier_;
  uint8_t tachFanPolesMultiplier_; // To avoid doubles, this is actually multiplied by 2 to make it an integer.
  uint8_t fanPoleCount_;
  uint16_t targetTachCount_;
  uint16_t fanSpeed_;

  EMC2301_STATUS writeRegisterBits(uint8_t registerAddress, uint8_t clearingMask, uint8_t byteToWrite);
  EMC2301_STATUS writeTachoTarget(uint16_t tachoTarget);
};


#endif

