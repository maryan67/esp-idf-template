/*
 * motor.cpp
 *
 *  Created on: Apr 22, 2018
 *      Author: secar
 */

#include "motor.h"

MotorDriver::MotorDriver (ledc_timer_config_t* TimerConfig_pst,
			  ledc_channel_config_t * ChannelConfig_pst) const
			      throw (GeneralErrorCodes_te)
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
      GetSavedConfiguration ();
      TimerConfig_pst->duty_resolution = DUTY_RESOLUTION;
      TimerConfig_pst->freq_hz = UPDATE_FREQUENCY;
      ledc_timer_config (TimerConfig_pst);
      ChannelConfig_pst->duty = MinPWMValue_u16;
      ChannelConfig_pst->intr_type = GPIO_INTR_DISABLE;
      this->ChannelConfig_pst = ChannelConfig_pst;
      IsActive_b = true;
      MotorState_e = MOTOR_STATE_INITIALISED;

    }
  catch (GeneralErrorCodes_te & ErrorCode_e)
    {
      throw ErrorCode_e;

    }

}

void MotorDriver::GetSavedConfiguration () const throw (GeneralErrorCodes_te)
{
  NVSModule
  NVSModule_o ();

  try
    {
      // get the value from saved memory

    }
  catch (GeneralErrorCodes_te &ErrorCode_e)
    {
      throw ErrorCode_e;
    }

}

void MotorDriver::SetThrottlePercentage (uint8_t PercentageOfThrottle) const
    throw (GeneralErrorCodes_te)
{
  if ((PercentageOfThrottle > 100) || (PercentageOfThrottle < 1))
    {
      throw INVALID_PARAMETERS;
    }

  this->PercentageOfThrottle = PercentageOfThrottle;


  WritePWM_v();



}

