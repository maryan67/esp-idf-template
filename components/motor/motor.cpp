/*
 * motor.cpp
 *
 *  Created on: Apr 22, 2018
 *      Author: secar
 */

#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint8_t MotorDriver::ChannelID = 0u;
uint8_t MotorDriver::TimerID = 0u;

// this finishes the initialisation of the PWM channel
// and also initialises the driver
// on error throws
MotorDriver::MotorDriver(ledc_timer_config_t *TimerConfig_pst,
                         ledc_channel_config_t ChannelConfig_pst) noexcept(false)
{
  MotorState_e = MOTOR_STATE_UNINITIALISED;
  IsActive_b = false;
  MaxPWMValue_u16 = 0;
  MinPWMValue_u16 = 0;

  PercentageOfThrottle = STARTING_PERCENTAGE_OF_THROTTLE;
  ControlMode_e = MOTOR_NORMAL_CONTROL_MODE;

  try
  {
    //GetSavedConfiguration();
    MinPWMValue_u16 = 1060;
    MaxPWMValue_u16 = 2000;

    ChannelConfig_pst.duty = 0;
    ChannelConfig_pst.intr_type = LEDC_INTR_DISABLE;
    ChannelConfig_pst.speed_mode = LEDC_HIGH_SPEED_MODE;
    ChannelConfig_pst.hpoint = 0;
    ChannelConfig_pst.channel = static_cast<ledc_channel_t>(ChannelID);
    ChannelConfig_pst.timer_sel = static_cast<ledc_timer_t>(TimerID);

    TimerConfig_pst->duty_resolution = LEDC_TIMER_16_BIT;
    TimerConfig_pst->freq_hz = UPDATE_FREQUENCY;
    TimerConfig_pst->speed_mode = LEDC_HIGH_SPEED_MODE;
    TimerConfig_pst->timer_num = static_cast<ledc_timer_t>(TimerID);
    TimerConfig_pst->clk_cfg = LEDC_AUTO_CLK;

    TimerID++;
    ledc_timer_config(TimerConfig_pst);

    ChannelID++;
    this->ChannelConfig_pst = ChannelConfig_pst;
    IsActive_b = true;
    MotorState_e = MOTOR_STATE_INITIALISED;
  }
  catch (GeneralErrorCodes_te &ErrorCode_e)
  {

    // if(ErrorCode_e == NO_CONFIGURATION_FOUND)
    //   SaveConfiguration();
    throw ErrorCode_e;
  }
}

uint64_t MotorDriver::PWMMicroSecondstoDuty(uint16_t InputMicroSeconds_u16)
{

  return static_cast<uint64_t>(((static_cast<float>(InputMicroSeconds_u16 * 100) / static_cast<float>(20000)) / 100) * static_cast<float>((1 << 16) - 1));
}

void MotorDriver::StartMotor_u16() noexcept(false)

{
  if (MotorState_e == MOTOR_STATE_INITIALISED)
  {
    if (ledc_channel_config(&ChannelConfig_pst) != ESP_OK)
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

void MotorDriver::stop_motor() noexcept(false)
{

  try
  {
    armLow();
  }
  catch (GeneralErrorCodes_te &ErrorCode_e)
  {
    throw ErrorCode_e;
  }
}

// void MotorDriver::SetThrottlePercentage(uint8_t PercentageOfThrottle) noexcept(false)

// {
//   if ((PercentageOfThrottle > 100) || (PercentageOfThrottle < 1))
//   {
//     throw INVALID_PARAMETERS;
//   }

//   this->PercentageOfThrottle = PercentageOfThrottle;

//   UpdatePWM_v();
// }

void MotorDriver::UpdatePWM_v(uint16_t newPWMValue)
{

  uint64_t newPWMDuty = PWMMicroSecondstoDuty(static_cast<uint64_t>(newPWMValue));
  // Transform form arduino microseconds to register duty
  ledc_set_duty(ChannelConfig_pst.speed_mode, ChannelConfig_pst.channel,
                newPWMDuty);

  ledc_update_duty(ChannelConfig_pst.speed_mode, ChannelConfig_pst.channel);
}

// void MotorDriver::GetSavedConfiguration() noexcept(false)
// {
//   try
//   {
//     NVSModule *NVSModule_po = new NVSModule();
//     MinPWMValue_u16 = NVSModule_po->GetValueOfField_u16("min_pwm");
//     MaxPWMValue_u16 = NVSModule_po->GetValueOfField_u16("max_pwm");
//     delete NVSModule_po;
//   }
//   catch (GeneralErrorCodes_te &ErrorCode_e)
//   {
//     throw ErrorCode_e;
//   }
// }

void MotorDriver::armLow(void)
{
  UpdatePWM_v(1000);
  ActualPwmDuty_u16 = 1000;
}

void MotorDriver::armHigh(void)
{
  UpdatePWM_v(1860);
}

// void MotorDriver::SaveConfiguration(void) noexcept(false)
// {
//   try
//   {
//     NVSModule *NVSModule_po = new NVSModule();
//     NVSModule_po->Writeu16ToField_v("min_pwm", 748);
//     NVSModule_po->Writeu16ToField_v("max_pwm", 2400);
//     delete NVSModule_po;
//   }
//   catch (GeneralErrorCodes_te &ErrorCode_e)
//   {
//     throw ErrorCode_e;
//   }
// }

// ultilitary
void MotorDriver::Calibrate(void)
{
  armHigh();
  printf("ARMED HIGH\n ");
  vTaskDelay(10000 / portTICK_PERIOD_MS);

  printf("ARMED LOW\n ");
  armLow();
  vTaskDelay(5000 / portTICK_PERIOD_MS);
}
