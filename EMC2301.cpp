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

#include "EMC2301.h"

EMC2301::EMC2301(I2C * i2cWire) : i2cWire_(i2cWire)
{
  // Base config for a fan with 2 poles and 500 min RPM.
  // Fan starts in off state.
  tachMinRPMMultiplier_   = 1;
  tachFanPolesMultiplier_ = 1;
  fanPoleCount_           = 2;
  fanSpeed_               = 0;
  targetTachCount_        = TACHO_OFF;
}

EMC2301::~EMC2301()
{
}

// Set the base frequency of the PWM output, which could be divided further by setPWMFrequencyDivider().
// The function is written such that any frequency below the next higher base frequency
// will automatically be converted to the previous frequency.
EMC2301_STATUS EMC2301::setPWMFrequencyBase(double frequencyKHz)
{
  uint8_t writeByte;
  if (frequencyKHz < 4.882)
  {
    writeByte = EMC2301_REG_PWMBASEFREQ_2KHZ;
  }
  else if (frequencyKHz < 19.531)
  {
    writeByte = EMC2301_REG_PWMBASEFREQ_4KHZ;
  }
  else if (frequencyKHz < 26)
  {
    writeByte = EMC2301_REG_PWMBASEFREQ_19KHZ;
  }
  else
  {
    writeByte = EMC2301_REG_PWMBASEFREQ_26KHZ;
  }

  if (i2cWire_->write(I2C_ADDRESS, EMC2301_REG_PWMBASEFREQ, writeByte) == I2C_STATUS_OK)
  {
    return EMC2301_STATUS_OK;
  }
  else
  {
    return EMC2301_STATUS_FAIL;
  }
}

// 
EMC2301_STATUS EMC2301::setPWMFrequencyDivider(uint8_t divisor)
{
  if (i2cWire_->write(I2C_ADDRESS, EMC2301_REG_PWMDIVIDE, divisor) == I2C_STATUS_OK)
  {
    return EMC2301_STATUS_OK;
  }
  else
  {
    return EMC2301_STATUS_FAIL;
  }
}

// Toggles the RPM-based Fan Speed Control Algorithm, whereby the fan speed will be controlled
// by the EMC2301 using a PID algorithm based on the target tachometer reading given by user.
EMC2301_STATUS EMC2301::toggleControlAlgorithm(bool enable)
{
  if (enable)
  {
    return writeRegisterBits(EMC2301_REG_FANCONFIG1, ~EMC2301_REG_FANCONFIG1_RPMCONTROL, EMC2301_REG_FANCONFIG1_RPMCONTROL);
  }
  else
  {
    return writeRegisterBits(EMC2301_REG_FANCONFIG1, ~EMC2301_REG_FANCONFIG1_RPMCONTROL, 0);
  }
}

// Since the tachometer register on the EMC2301 has an upper limit, it is necessary to apply multipliers
// for fans with high RPMs. This is done by adjusting the minimum RPM expected for the fan.
// The function is written such that the min RPM will be forced to the closest lower RPM.
EMC2301_STATUS EMC2301::setTachMinRPM(uint16_t minRPM)
{
  uint8_t writeByte;
  if (minRPM < 1000)
  {
    writeByte = EMC2301_REG_FANCONFIG1_MINRPM_500;
    tachMinRPMMultiplier_ = 1;
  }
  else if (minRPM < 2000)
  {
    writeByte = EMC2301_REG_FANCONFIG1_MINRPM_1000;
    tachMinRPMMultiplier_ = 2;
  }
  else if (minRPM < 4000)
  {
    writeByte = EMC2301_REG_FANCONFIG1_MINRPM_2000;
    tachMinRPMMultiplier_ = 4;
  }
  else
  {
    writeByte = EMC2301_REG_FANCONFIG1_MINRPM_4000;
    tachMinRPMMultiplier_ = 8;
  }

  return writeRegisterBits(EMC2301_REG_FANCONFIG1, EMC2301_REG_FANCONFIG1_MINRPM_CLEAR, writeByte);
}

// The number of fan poles would also affect the tachometer counting because
// different fans generate different amount of pulses per rotation. Most fans
// are 2 poles, but this info should be obtained directly from the fan datasheet.
EMC2301_STATUS EMC2301::setFanPoles(uint8_t poleCount)
{
  uint8_t writeByte;

  // To avoid doubles, we first multiply the fan pole multiplier by 2 to make it an integer.
  // It will be divided by 2 in the equations relating RPM to tachometer readings.
  switch (poleCount)
  {
  case 1:
    writeByte = EMC2301_REG_FANCONFIG1_FANPOLES_1;
    tachFanPolesMultiplier_ = 1;
    break;
  case 2:
    writeByte = EMC2301_REG_FANCONFIG1_FANPOLES_2;
    tachFanPolesMultiplier_ = 2;
    break;
  case 3:
    writeByte = EMC2301_REG_FANCONFIG1_FANPOLES_3;
    tachFanPolesMultiplier_ = 3;
    break;
  case 4:
    writeByte = EMC2301_REG_FANCONFIG1_FANPOLES_4;
    tachFanPolesMultiplier_ = 4;
  default:
    return EMC2301_STATUS_INVALIDARG;
    break;
  }

  return writeRegisterBits(EMC2301_REG_FANCONFIG1, EMC2301_REG_FANCONFIG1_FANPOLES_CLEAR, writeByte);
}

// Adjusts the period between subsequent PWM drive updates.
// This is only used if toggleRampControl() was enabled.
// The function is written such that the period will be forced to the closest lower period.
EMC2301_STATUS EMC2301::setDriveUpdatePeriod(uint16_t periodMs)
{
  uint8_t writeByte;
  if (periodMs < 200)
  {
    writeByte = EMC2301_REG_FANCONFIG1_UPDATE_100;
  }
  else if (periodMs < 300)
  {
    writeByte = EMC2301_REG_FANCONFIG1_UPDATE_200;
  }
  else if (periodMs < 400)
  {
    writeByte = EMC2301_REG_FANCONFIG1_UPDATE_300;
  }
  else if (periodMs < 500)
  {
    writeByte = EMC2301_REG_FANCONFIG1_UPDATE_400;
  }
  else if (periodMs < 800)
  {
    writeByte = EMC2301_REG_FANCONFIG1_UPDATE_500;
  }
  else if (periodMs < 1200)
  {
    writeByte = EMC2301_REG_FANCONFIG1_UPDATE_800;
  }
  else if (periodMs < 1600)
  {
    writeByte = EMC2301_REG_FANCONFIG1_UPDATE_1200;
  }
  else
  {
    writeByte = EMC2301_REG_FANCONFIG1_UPDATE_1600;
  }

  return writeRegisterBits(EMC2301_REG_FANCONFIG1, EMC2301_REG_FANCONFIG1_UPDATE_CLEAR, writeByte);
}

// Toggle ramp control for DIRECT CONTROL MODE, whereby the fan speed will be increased gradually.
// Disabling this will allow fan speed to be changed instantly.
// Note that enabling the RPM-based Fan Speed Control will automatically use the
// ramp control, regardless of the status of this function.
EMC2301_STATUS EMC2301::toggleRampControl(bool enable)
{
  if (enable)
  {
    return writeRegisterBits(EMC2301_REG_FANCONFIG2, ~EMC2301_REG_FANCONFIG2_RAMPCONTROL, EMC2301_REG_FANCONFIG2_RAMPCONTROL);
  }
  else
  {
    return writeRegisterBits(EMC2301_REG_FANCONFIG2, ~EMC2301_REG_FANCONFIG2_RAMPCONTROL, 0);
  }
}

// Toggle the glitch filter, which removes high frequency noise
// from the TACH pin.
EMC2301_STATUS EMC2301::toggleGlitchFilter(bool enable)
{
  if (enable)
  {
    return writeRegisterBits(EMC2301_REG_FANCONFIG2, ~EMC2301_REG_FANCONFIG2_GLITCHFILTER, EMC2301_REG_FANCONFIG2_GLITCHFILTER);
  }
  else
  {
    return writeRegisterBits(EMC2301_REG_FANCONFIG2, ~EMC2301_REG_FANCONFIG2_GLITCHFILTER, 0);
  }
}

// Change the type of derivative used in the PID algorithm for RPM-based speed control.
// Refer to Table 5.15 at pg 31 of the datasheet.
EMC2301_STATUS EMC2301::setDerivativeMode(uint8_t modeType)
{
  uint8_t writeByte;

  switch (modeType)
  {
  case 0:
    writeByte = EMC2301_REG_FANCONFIG2_DEROPT_NONE;
    break;
  case 1:
    writeByte = EMC2301_REG_FANCONFIG2_DEROPT_BASIC;
    break;
  case 2:
    writeByte = EMC2301_REG_FANCONFIG2_DEROPT_STEP;
    break;
  case 3:
    writeByte = EMC2301_REG_FANCONFIG2_DEROPT_BOTH;
  default:
    return EMC2301_STATUS_INVALIDARG;
    break;
  }

  return writeRegisterBits(EMC2301_REG_FANCONFIG2, EMC2301_REG_FANCONFIG2_DEROPT_CLEAR, writeByte);
}

// Since the tachometer has an accuracy rating, it is not expected that the
// RPM readings will be constant even if the PWM drive is constant. Therefore,
// it may be desirable to tell the EMC2301 to stop changing PWM drive as long as
// the RPM reading is within a tolerance of the target. This function does that.
// The function is written such that the error range will be forced to the closest higher range.
// The argument should be a positive number.
EMC2301_STATUS EMC2301::setControlErrRange(uint8_t errorRangeRPM)
{
  uint8_t writeByte;
  if (errorRangeRPM < 0.01) // Account for doubles sometimes not being exactly 0
  {
    writeByte = EMC2301_REG_FANCONFIG2_ERRRANGE_0;
  }
  else if (errorRangeRPM <= 50)
  {
    writeByte = EMC2301_REG_FANCONFIG2_ERRRANGE_50;
  }
  else if (errorRangeRPM <= 100)
  {
    writeByte = EMC2301_REG_FANCONFIG2_ERRRANGE_100;
  }
  else
  {
    writeByte = EMC2301_REG_FANCONFIG2_ERRRANGE_200;
  }

  return writeRegisterBits(EMC2301_REG_FANCONFIG2, EMC2301_REG_FANCONFIG2_ERRRANGE_CLEAR, writeByte);
}

// Toggle max spin up, whereby the fan is set to 100% duty cycle for 1/4th of the
// time during the spin up routine.
EMC2301_STATUS EMC2301::toggleSpinUpMax(bool enable)
{
  if (enable)
  {
    return writeRegisterBits(EMC2301_REG_FANSPINUP, ~EMC2301_REG_FANSPINUP_NOKICK, EMC2301_REG_FANSPINUP_NOKICK);
  }
  else
  {
    return writeRegisterBits(EMC2301_REG_FANSPINUP, ~EMC2301_REG_FANSPINUP_NOKICK, 0);
  }
}

// Set the drive level that should be used during the spin up routine.
// The function is written such that the drive will be forced to the closest lower drive.
EMC2301_STATUS EMC2301::setSpinUpDrive(uint8_t drivePercent)
{
  uint8_t writeByte;
  if (drivePercent < 35)
  {
    writeByte = EMC2301_REG_FANSPINUP_SPINLVL_30;
  }
  else if (drivePercent < 40)
  {
    writeByte = EMC2301_REG_FANSPINUP_SPINLVL_35;
  }
  else if (drivePercent < 45)
  {
    writeByte = EMC2301_REG_FANSPINUP_SPINLVL_40;
  }
  else if (drivePercent < 50)
  {
    writeByte = EMC2301_REG_FANSPINUP_SPINLVL_45;
  }
  else if (drivePercent < 55)
  {
    writeByte = EMC2301_REG_FANSPINUP_SPINLVL_50;
  }
  else if (drivePercent < 60)
  {
    writeByte = EMC2301_REG_FANSPINUP_SPINLVL_55;
  }
  else if (drivePercent < 65)
  {
    writeByte = EMC2301_REG_FANSPINUP_SPINLVL_60;
  }
  else
  {
    writeByte = EMC2301_REG_FANSPINUP_SPINLVL_65;
  }

  return writeRegisterBits(EMC2301_REG_FANSPINUP, EMC2301_REG_FANSPINUP_SPINLVL_CLEAR, writeByte);
}

// Determine the duration of the spin up routine.
// The function is written such that the time will be forced to the closest shorter time.
EMC2301_STATUS EMC2301::setSpinUpTime(uint16_t timeMs)
{
  uint8_t writeByte;
  if (timeMs < 500)
  {
    writeByte = EMC2301_REG_FANSPINUP_SPINUPTIME_250;
  }
  else if (timeMs < 1000)
  {
    writeByte = EMC2301_REG_FANSPINUP_SPINUPTIME_500;
  }
  else if (timeMs < 2000)
  {
    writeByte = EMC2301_REG_FANSPINUP_SPINUPTIME_1000;
  }
  else
  {
    writeByte = EMC2301_REG_FANSPINUP_SPINUPTIME_2000;
  }

  return writeRegisterBits(EMC2301_REG_FANSPINUP, EMC2301_REG_FANSPINUP_SPINUPTIME_CLEAR, writeByte);
}

// Set the maximum change in fan drive that could be performed over a single
// update period. Maximum is 0b00111111 (aka 63 or 0x3F)
EMC2301_STATUS EMC2301::setControlMaxStep(uint8_t stepSize)
{
  if (stepSize > EMC2301_REG_FANMAXSTEP_MAX)
  {
    stepSize = EMC2301_REG_FANMAXSTEP_MAX;
  }

  if (i2cWire_->write(I2C_ADDRESS, EMC2301_REG_FANMAXSTEP, stepSize) == I2C_STATUS_OK)
  {
    return EMC2301_STATUS_OK;
  }
  else
  {
    return EMC2301_STATUS_FAIL;
  }
}

// Sets the minimum allowable drive for the RPM-based Fan Speed Control algorithm.
// The algorithm will not drive the fan at a level lower than this unless the
// tachometer target is specifically set to 0xFF.
// This is extremely useful for fans that would stop spinning if the PWM signal
// is low but not zero because once the PWM signal is low enough, the fan stops spinning
// and the tachometer readings become zero causing the algorithm to drive high and restart
// the fan, but now the tachometer is above the target. The fan is then driven to a halt again
// and this process is repeated indefinitely, causing the fan to on-off-on-off......
// Having a minimum drive prevents this from happening.
EMC2301_STATUS EMC2301::setFanMinDrive(double minDrivePercent)
{
  // Convert the percent to byte format
  uint8_t writeByte = (uint8_t) (constrain(minDrivePercent, 0, 100) / 100 * 255);

  if (i2cWire_->write(I2C_ADDRESS, EMC2301_REG_FANMINDRIVE, writeByte) == I2C_STATUS_OK)
  {
    return EMC2301_STATUS_OK;
  }
  else
  {
    return EMC2301_STATUS_FAIL;
  }
}

// Sets the minimum RPM which is checked at the end of the spin up routine to decide if the fan is actually
// moving or if it is stalled.
// Internally, the function converts the min RPM to tachometer count that will be written
// to the appropriate register.
// NOTE: this value shouldn't be the absolute minimum RPM because it only serves as a check
// for the spin up routine. Absolute minimum speed should be set at setFanMinDrive(), although
// that function accepts percentage, not RPM.
// Also NOTE: the min value is dependent on what was set in setTachMinRPM(). This function will automatically
// increase the RPM to the lower limit if the given RPM is lower than the one set in setTachMinRPM().
EMC2301_STATUS EMC2301::setMinValidRPM(uint16_t minRPM)
{
  // Ensure the given min RPM is not below the limits of the tachometer.
  uint16_t tachMinRPM;
  switch (tachMinRPMMultiplier_)
  {
  case 1:
    tachMinRPM = 500;
    break;
  case 2:
    tachMinRPM = 1000;
    break;
  case 3:
    tachMinRPM = 2000;
    break;
  default:
  case 4:
    tachMinRPM = 4000;
    break;
  }

  if (minRPM < tachMinRPM)
  {
    minRPM = tachMinRPM;
  }

  // To avoid doubles, the fan pole multiplier was multiplied by 2 to make it an integer.
  // Here, we divide it (and the -1 in the bracket) by 2 to bring it back to its proper value.
  uint8_t maxTachCount_ = 60 * tachMinRPMMultiplier_ * TACHO_FREQUENCY * (tachFanPolesMultiplier_ - 2) / 2 / fanPoleCount_ / minRPM;

  if (i2cWire_->write(I2C_ADDRESS, EMC2301_REG_FANVALTACHCOUNT, maxTachCount_) == I2C_STATUS_OK)
  {
    return EMC2301_STATUS_OK;
  }
  else
  {
    return EMC2301_STATUS_FAIL;
  }
}

// Based on the given target RPM, calculate the appropriate target tachometer reading and
// write it to the appropriate register.
// TODO: need sanity check for targetRPM to ensure it doesn't cause the calculation to overflow
// the max value of uint16_t (65535).
EMC2301_STATUS EMC2301::setRPMTarget(uint16_t targetRPM)
{
  // To avoid doubles, the fan pole multiplier was multiplied by 2 to make it an integer.
  // Here, we divide it (and the -1 in the bracket) by 2 to bring it back to its proper value.
  targetTachCount_ = 60 * tachMinRPMMultiplier_ * TACHO_FREQUENCY * (tachFanPolesMultiplier_ - 2) / 2 / fanPoleCount_ / targetRPM;
  return writeTachoTarget(targetTachCount_);
}

// Turn the fan on (to the most recently known target RPM) or turn it off
EMC2301_STATUS EMC2301::toggleFan(bool enable)
{
  if (enable)
  {
    return writeTachoTarget(targetTachCount_);
  }
  else
  {
    return writeTachoTarget(TACHO_OFF);
  }
}

// Obtain the tachometer reading, convert to RPM, and store in a private variable.
// Get the fan speed (RPM) by calling getFanSpeed().
EMC2301_STATUS EMC2301::fetchFanSpeed()
{
  if (i2cWire_->read(I2C_ADDRESS, EMC2301_REG_TACHREADMSB, (uint8_t) 1) == I2C_STATUS_OK)
  {
    uint16_t tachoCount = (i2cWire_->getByte()) << 8;

    if (i2cWire_->read(I2C_ADDRESS, EMC2301_REG_TACHREADLSB, (uint8_t)1) == I2C_STATUS_OK)
    {
      tachoCount |= i2cWire_->getByte();
      tachoCount = tachoCount >> 3;

      // To avoid doubles, the fan pole multiplier was multiplied by 2 to make it an integer.
      // Here, we divide it (and the -1 in the bracket) by 2 to bring it back to its proper value.
      fanSpeed_ = 60 * tachMinRPMMultiplier_ * TACHO_FREQUENCY * (tachFanPolesMultiplier_ - 2) / 2 / fanPoleCount_ / tachoCount;
    }
    else
    {
      return EMC2301_STATUS_FAIL;
    }
  }
  else
  {
    return EMC2301_STATUS_FAIL;
  }
}

uint16_t EMC2301::getFanSpeed()
{
  return fanSpeed_;
}

// Writes specific bits in the given register, such that the other bits in the register
// are unaffected. This is done by reading the register first, masking the appropriate
// bits, and then writing this modified byte into the register.
EMC2301_STATUS EMC2301::writeRegisterBits(uint8_t registerAddress, uint8_t clearingMask, uint8_t byteToWrite)
{
  if (i2cWire_->read(I2C_ADDRESS, registerAddress, (uint8_t) 1) == I2C_STATUS_OK)
  {
    uint8_t registerContents = i2cWire_->getByte();
    registerContents &= clearingMask; // Reset the bits at the location of interest
    registerContents |= byteToWrite;  // Write bits to the location of interest

    if (i2cWire_->write(I2C_ADDRESS, registerAddress, registerContents) == I2C_STATUS_OK)
    {
      return EMC2301_STATUS_OK;
    }
    else
    {
      return EMC2301_STATUS_FAIL;
    }
  }
  else
  {
    return EMC2301_STATUS_FAIL;
  }
}

EMC2301_STATUS EMC2301::writeTachoTarget(uint16_t tachoTarget)
{
  uint8_t tachCountLSB = (tachoTarget << 3) && 0xF8;
  uint8_t tachCountMSB = (tachoTarget >> 5) && 0xFF;

  // The low byte must be written before the high byte, because
  // the target is officially changed once the high byte is written (pg 36 of datasheet).
  if (i2cWire_->write(I2C_ADDRESS, EMC2301_REG_TACHTARGETLSB, tachCountLSB) == I2C_STATUS_OK)
  {
    if (i2cWire_->write(I2C_ADDRESS, EMC2301_REG_TACHTARGETMSB, tachCountMSB) == I2C_STATUS_OK)
    {
      return EMC2301_STATUS_OK;
    }
    else
    {
      return EMC2301_STATUS_FAIL;
    }
  }
  else
  {
    return EMC2301_STATUS_FAIL;
  }
}
