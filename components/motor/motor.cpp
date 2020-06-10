/*
 * motor.cpp
 *
 *  Created on: Apr 22, 2018
 *      Author: secar
 */

#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"






void MotorDriver::init_motor() noexcept(false)

{
 
  mcpwm_gpio_init(m_pwm_unit, m_pwm_timer == MCPWM_TIMER_0 ? MCPWM0A: MCPWM1A, m_pin_number);
  mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(m_pwm_unit, m_pwm_timer, &pwm_config);
    mcpwm_start(m_pwm_unit,m_pwm_timer);

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

   mcpwm_set_duty_in_us(m_pwm_unit, m_pwm_timer, MCPWM_OPR_A, newPWMValue);
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
  
  printf("ARMED LOW\n ");
  UpdatePWM_v(1000);
  ActualPwmDuty_u16 = 1000;
}

void MotorDriver::armHigh(void)
{
  printf("ARMED HIGH\n ");
  UpdatePWM_v(2000);
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
  
  vTaskDelay(10000 / portTICK_PERIOD_MS);

  
  armLow();
  vTaskDelay(5000 / portTICK_PERIOD_MS);
}
