/*
 * motor.cpp
 *
 *  Created on: Apr 22, 2018
 *      Author: secar
 */

#include "motor.h"

// this finishes the initialisation of the PWM channel
// and also initialises the driver
// on error throws
MotorDriver::MotorDriver(ledc_timer_config_t *TimerConfig_pst,
                         ledc_channel_config_t *ChannelConfig_pst) noexcept(false)
{
  MotorState_e = MOTOR_STATE_UNINITIALISED;
  IsActive_b = false;
  MaxPWMValue_u16 = 0;
  MinPWMValue_u16 = 0;
  this->ChannelConfig_pst = NULL;
  PercentageOfThrottle = STARTING_PERCENTAGE_OF_THROTTLE;
  ControlMode_e = MOTOR_NORMAL_CONTROL_MODE;

  try
  {
    GetSavedConfiguration();
    TimerConfig_pst->duty_resolution = LEDC_TIMER_13_BIT;
    TimerConfig_pst->freq_hz = UPDATE_FREQUENCY;
    ledc_timer_config(TimerConfig_pst);
    ChannelConfig_pst->duty = MinPWMValue_u16;
    ChannelConfig_pst->intr_type = LEDC_INTR_DISABLE;
    this->ChannelConfig_pst = ChannelConfig_pst;
    IsActive_b = true;
    MotorState_e = MOTOR_STATE_INITIALISED;
  }
  catch (GeneralErrorCodes_te &ErrorCode_e)
  {
    throw ErrorCode_e;
  }
}

uint16_t MotorDriver::PercentageToPWMMicroseconds(uint8_t PercentageOfThrottle)
{
  uint16_t MaxPWM = MaxPWMValue_u16 - MinPWMValue_u16;
  return (PercentageOfThrottle * MaxPWM) / 100;
}

void MotorDriver::StartMotor_u16() noexcept(false)

{
  if (MotorState_e == MOTOR_STATE_INITIALISED)
  {
    if (ledc_channel_config(ChannelConfig_pst) != ESP_OK)
    {
      throw HAL_ERROR;
    }
  }
  else
    throw MODULE_NOT_INITIALISED;
}

void MotorDriver::SetControlMode_v(ControlMode_te ControlMode_e)
{
  this->ControlMode_e = ControlMode_e;
}

void MotorDriver::EmergencyStopMotor() noexcept(false)

{
  if (ControlMode_e == MOTOR_EMERGENCY_CONTROL_MODE)
  {
    try
    {
      SetThrottlePercentage(0);
    }
    catch (GeneralErrorCodes_te &ErrorCode_e)
    {
      throw ErrorCode_e;
    }
  }
  else
    throw EMERGENCY_MODE_NOT_SET;
}

void MotorDriver::SetThrottlePercentage(uint8_t PercentageOfThrottle) noexcept(false)

{
  if ((PercentageOfThrottle > 100) || (PercentageOfThrottle < 1))
  {
    throw INVALID_PARAMETERS;
  }

  this->PercentageOfThrottle = PercentageOfThrottle;

  UpdatePWM_v();
}

void MotorDriver::UpdatePWM_v()
{
  ledc_set_duty(ChannelConfig_pst->speed_mode, ChannelConfig_pst->channel,
                static_cast<uint32_t>(PercentageToPWMMicroseconds(PercentageOfThrottle)));
  ledc_update_duty(ChannelConfig_pst->speed_mode, ChannelConfig_pst->channel);
}

void MotorDriver::GetSavedConfiguration() noexcept(false)
{
  try
  {
    NVSModule *NVSModule_po = new NVSModule();
    MinPWMValue_u16 = NVSModule_po->GetValueOfField_u16("min_pwm");
    MaxPWMValue_u16 = NVSModule_po->GetValueOfField_u16("max_pwm");
    delete NVSModule_po;
  }
  catch (GeneralErrorCodes_te &ErrorCode_e)
  {
    throw ErrorCode_e;
  }
}

void MotorDriver::arm(void)
{
  ledc_set_duty(ChannelConfig_pst->speed_mode, ChannelConfig_pst->channel,
                MinPWMValue_u16);
  ledc_update_duty(ChannelConfig_pst->speed_mode, ChannelConfig_pst->channel);
}

void MotorDriver::SaveConfiguration(void) noexcept(false)
{
}
