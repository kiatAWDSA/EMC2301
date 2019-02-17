/*********************************************************************************
Arduino Library for the Microchip EMC2301 RPM-based PWM fan controller.
I2C communication is performed using a custom I2C library.

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
  EMC2301_STATUS_OK = 0,  // No problemo.
  EMC2301_STATUS_FAIL = 1,  // Something went wrong.
  EMC2301_STATUS_NOTREADY = 2,  // The sensor is not ready to send out data.
  EMC2301_STATUS_CORRUPT = 3,   // Either the relative humidity or temperature bytes failed the CRC check.
  EMC2301_STATUS_DATA_LESS = 4,  // Received less data bytes than expected.
} EMC2301_STATUS;



class EMC2301
{
public:
  // Repeatability of measurements
  typedef enum
  {
    REP_LOW = 0,
    REP_MED = 1,
    REP_HIG = 2
  } Repeatability;

  EMC2301(I2C& i2cWire);
  EMC2301(I2C& i2cWire, bool ADDRPinHigh);
  void changeAddress(bool ADDRPinHigh);
  EMC2301_STATUS triggerOneMeasurement(bool stretchClock, Repeatability repeatability);
  EMC2301_STATUS fetchMeasurement();
  double getRH();
  double getTemperature();

private:
  // The sensor has a "base" address that can be modified depending on the state of the ADDR pin (pin 2)
  static const uint8_t BASE_ADDRESS = 0x44;

  /******************************
   *     List of registers      *
   ******************************/
  static const uint8_t EMC2301_REG_PWMBASEFREQ          = 0x2D;
  static const uint8_t EMC2301_REG_FANSETTING           = 0x30;
  static const uint8_t EMC2301_REG_PWMDIVIDE            = 0x31;
  static const uint8_t EMC2301_REG_FANCONFIG1           = 0x32;
  static const uint8_t EMC2301_REG_FANCONFIG2           = 0x33;
  static const uint8_t EMC2301_REG_FANSPINUP            = 0x36;
  static const uint8_t EMC2301_REG_FANMAXSTEP           = 0x37;
  static const uint8_t EMC2301_REG_FANMINDRIVE          = 0x38;
  static const uint8_t EMC2301_REG_FANVALTACHCOUNT      = 0x39;
  static const uint8_t EMC2301_REG_TACHTARGETLSB        = 0x3C;
  static const uint8_t EMC2301_REG_TACHTARGETMSB        = 0x3D;
  static const uint8_t EMC2301_REG_TACHREADMSB          = 0x3E;
  static const uint8_t EMC2301_REG_TACHREADLSB          = 0x3F;

  /* Registers that can have values written directly into them (i.e. the entire register is meant for a single number):
      EMC2301_REG_FANSETTING
      EMC2301_REG_PWMDIVIDE
      EMC2301_REG_FANMAXSTEP (Max 0b00111111 or 0x3F or 63)
      EMC2301_REG_FANMINDRIVE
      EMC2301_REG_FANVALTACHCOUNT (The final value from this register is 32 x (value in register))
      EMC2301_REG_TACHTARGETLSB and EMC2301_REG_TACHTARGETMSB (MUST write both LSB and MSB, with LSB written before MSB)
  */

  // EMC2301_REG_PWMBASEFREQ
  static const uint8_t EMC2301_REG_PWMBASEFREQ_26KHZ_NOT  = ~0x03;
  static const uint8_t EMC2301_REG_PWMBASEFREQ_19KHZ  = 0x01;
  static const uint8_t EMC2301_REG_PWMBASEFREQ_4KHZ   = 0x02;
  static const uint8_t EMC2301_REG_PWMBASEFREQ_2KHZ   = 0x03;

  // EMC2301_REG_PWMDIVIDE
  static const uint8_t EMC2301_REG_PWMDIVIDE_1    = 0x01;
  static const uint8_t EMC2301_REG_PWMDIVIDE_2    = 0x02;
  static const uint8_t EMC2301_REG_PWMDIVIDE_4    = 0x04;
  static const uint8_t EMC2301_REG_PWMDIVIDE_8    = 0x08;
  static const uint8_t EMC2301_REG_PWMDIVIDE_16   = 0x10;
  static const uint8_t EMC2301_REG_PWMDIVIDE_32   = 0x20;
  static const uint8_t EMC2301_REG_PWMDIVIDE_64   = 0x40;
  static const uint8_t EMC2301_REG_PWMDIVIDE_128  = 0x80;

  // EMC2301_REG_FANCONFIG1
  static const uint8_t EMC2301_REG_FANCONFIG1_RPMCONTROL   = 0x80;
  static const uint8_t EMC2301_REG_FANCONFIG1_MINRPM_500_XOR   = ~0x00;
  static const uint8_t EMC2301_REG_FANCONFIG1_MINRPM_1000  = 0x20;
  static const uint8_t EMC2301_REG_FANCONFIG1_MINRPM_2000  = 0x40;
  static const uint8_t EMC2301_REG_FANCONFIG1_MINRPM_4000  = 0x60;
  static const uint8_t EMC2301_REG_FANCONFIG1_FANPOLES_1_NOT   = ~0x18;
  static const uint8_t EMC2301_REG_FANCONFIG1_FANPOLES_2   = 0x08;
  static const uint8_t EMC2301_REG_FANCONFIG1_FANPOLES_3   = 0x10;
  static const uint8_t EMC2301_REG_FANCONFIG1_FANPOLES_4   = 0x18;
  static const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_100_NOT   = ~0x07;
  static const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_200   = 0x01;
  static const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_300   = 0x02;
  static const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_400   = 0x03;
  static const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_500   = 0x04;
  static const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_800   = 0x05;
  static const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_1200  = 0x06;
  static const uint8_t EMC2301_REG_FANCONFIG1_UPDATE_1600  = 0x07;

  // EMC2301_REG_FANCONFIG2
  static const uint8_t EMC2301_REG_FANCONFIG2_RAMPCONTROL   = 0x40;
  static const uint8_t EMC2301_REG_FANCONFIG2_GLITCHFILTER  = 0x20;
  static const uint8_t EMC2301_REG_FANCONFIG2_DEROPT_NONE_NOT   = ~0x18;
  static const uint8_t EMC2301_REG_FANCONFIG2_DEROPT_BASIC  = 0x08;
  static const uint8_t EMC2301_REG_FANCONFIG2_DEROPT_STEP   = 0x10;
  static const uint8_t EMC2301_REG_FANCONFIG2_DEROPT_BOTH   = 0x18;
  static const uint8_t EMC2301_REG_FANCONFIG2_ERRRANGE_0_NOT    = ~0x06;
  static const uint8_t EMC2301_REG_FANCONFIG2_ERRRANGE_50   = 0x02;
  static const uint8_t EMC2301_REG_FANCONFIG2_ERRRANGE_100  = 0x04;
  static const uint8_t EMC2301_REG_FANCONFIG2_ERRRANGE_200  = 0x06;

  // EMC2301_REG_FANSPINUP
  static const uint8_t EMC2301_REG_FANSPINUP_NOKICK           = 0x20;
  static const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_30_NOT     = ~0x1C;
  static const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_35       = 0x04;
  static const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_40       = 0x08;
  static const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_45       = 0x0C;
  static const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_50       = 0x10;
  static const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_55       = 0x14;
  static const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_60       = 0x18;
  static const uint8_t EMC2301_REG_FANSPINUP_SPINLVL_65       = 0x1C;
  static const uint8_t EMC2301_REG_FANSPINUP_SPINUPTIME_250_NOT = ~0x03;
  static const uint8_t EMC2301_REG_FANSPINUP_SPINUPTIME_500   = 0x01;
  static const uint8_t EMC2301_REG_FANSPINUP_SPINUPTIME_1000  = 0x02;
  static const uint8_t EMC2301_REG_FANSPINUP_SPINUPTIME_2000  = 0x03;

  // EMC2301_REG_FANVALTACHCOUNT


  // Maximum duration (ms) needed to complete a measurement during the one-shot mode.
  // See datasheet Table 4.
  static const uint8_t DURATION_HIGREP = 15;
  static const uint8_t DURATION_MEDREP = 6;
  static const uint8_t DURATION_LOWREP = 4;

  // Number of bytes for I2C transmission
  static const uint8_t BYTECOUNT_DAQ_TOTAL = 6;
  static const uint8_t BYTECOUNT_DAQ_TEMP = 2;
  static const uint8_t BYTECOUNT_DAQ_RH = 2;
  static const uint8_t BYTECOUNT_DAQ_CRC = 1;

  /********************
  * DATA ACQUISITION *
  ********************/
  // One shot mode, clock stretching enabled
  static const uint8_t COM_DAQ_ONE_STRETCH_MSB = 0x2C;
  static const uint8_t COM_DAQ_ONE_STRETCH_LSB_HIGREP = 0x06;
  static const uint8_t COM_DAQ_ONE_STRETCH_LSB_MEDREP = 0x0D;
  static const uint8_t COM_DAQ_ONE_STRETCH_LSB_LOWREP = 0x10;

  // One shot mode, clock stretching disabled
  static const uint8_t COM_DAQ_ONE_NOSTRETCH_MSB = 0x24;
  static const uint8_t COM_DAQ_ONE_NOSTRETCH_LSB_HIGREP = 0x00;
  static const uint8_t COM_DAQ_ONE_NOSTRETCH_LSB_MEDREP = 0x0B;
  static const uint8_t COM_DAQ_ONE_NOSTRETCH_LSB_LOWREP = 0x16;

  I2C *i2cWire_;
  uint8_t i2cAddress_;
  uint8_t tachMinRPMMultiplier_;
  uint8_t tachFanPolesMultiplier_;
  double targetFanSpeed_;
  double fanSpeed_;

  bool setBaseFrequency(double frequencyKHz);
  bool toggleControlAlgorithm(bool enable);
  bool setTachMinRPM(double minRPM);
  bool setFanPoles(uint8_t poleCount);
  bool setDriveUpdatePeriod(double periodMs);
  bool toggleRampControl(bool enable);
  bool toggleGlitchFilter(bool enable);
  bool setDerivativeMode(uint8_t modeType);
  bool setControlErrRange(double errorRangeRPM);
  bool toggleSpinUpMax(bool enable);
  bool setSpinUpDrive(double drivePercent);
  bool setSpinUpTime(double timeMs);
  bool setControlMaxStep(uint8_t stepSize);
  bool setFanMinDrive(double minDrivePercent);
  bool setTachMaxValidCount(unsigned long validCount);
  bool setTachTarget(unsigned long targetCount);
  bool toggleFan(bool enable);
  bool getFanSpeed();
};


#endif

