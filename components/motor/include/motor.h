/*
 * motor.h
 *
 *  Created on: Apr 22, 2018
 *      Author: secar
 */

#ifndef _MOTOR_MOTOR_H_
#define _MOTOR_MOTOR_H_
extern "C"
{

#include "driver/ledc.h"
#include "driver/mcpwm.h"
}

#include "GeneralErrorCodes.h"
#include "motor_config.h"

typedef enum MotorState
{

    MOTOR_STATE_UNINITIALISED = 0,
    MOTOR_STATE_INITIALISED,
    MOTOR_STATE_UNKNOWN

} MotorState_te;

typedef enum ControlMode
{
    MOTOR_NORMAL_CONTROL_MODE = 0,
    MOTOR_EMERGENCY_CONTROL_MODE,
    MOTOR_UNKNOWN_CONTROL_MODE
} ControlMode_te;

class MotorDriver
{

public:
    // This constructor looks for empty PWD channels and assigns one
    MotorDriver(mcpwm_unit_t pwm_unit, mcpwm_timer_t pwm_timer,uint8_t pin_number): 
    m_pwm_unit(pwm_unit),
    m_pwm_timer(pwm_timer),
    m_pin_number(pin_number)
    {
        if(pwm_timer == MCPWM_TIMER_2)
            throw int (HAL_ERROR);
    }
    // Starts the electric motor
    void init_motor() noexcept(false);

    // arms the esc of the motor( should hear 2 beeps )
    void armLow();

    // THIS SHOULD ONLY BE USED ON A BIG EMERGENCY
    void stop_motor() noexcept(false);
    // THIS SHOULD ONLY BE USED ON A BIG EMERGENCY

    // Set the percentage of the throttle
    // void SetThrottlePercentage(uint8_t PercentageOfThrottle) noexcept(false);

    uint8_t GetThrottlePercentage(void)
    {
        return this->PercentageOfThrottle;
    }

    // Used for extra-saftey when stopping motor
    void SetControlMode_v(ControlMode_te ControlMode_e);

    void SetPWMThrottleValue_v(uint16_t newValue_u16) noexcept(false)
    {
        if (newValue_u16 > 1000 && newValue_u16 < 2000)
        {
            this->ActualPwmDuty_u16 = newValue_u16;

            UpdatePWM_v(this->ActualPwmDuty_u16);
        }

        else
            throw GeneralErrorCodes_te(INVALID_PARAMETERS);
    }

    void SetPWMDutyGain(int16_t gain, bool add) noexcept(false)
    {
        uint16_t newPWMValue = 0;

        {
            if (add)
            {
                if (ActualPwmDuty_u16 + gain > 2000)
                    newPWMValue = 2000;
                else
                    newPWMValue = this->ActualPwmDuty_u16 + gain;
            }
            else
            {
                if (ActualPwmDuty_u16 - gain < 1000)
                    newPWMValue = 1000;
                else
                    newPWMValue = this->ActualPwmDuty_u16 - gain;
            }

            if (newPWMValue < 1000)
                newPWMValue = 1000;
            if (newPWMValue > 2000)
                newPWMValue = 2000;
            UpdatePWM_v(newPWMValue);
        }
    }

    //Calibrate the ESC with the desired values
    void Calibrate();
    // Sends highest possible value to ESC
    void armHigh(void);

private:
    // The actual percentage of motor throttle
    uint8_t PercentageOfThrottle;
    mcpwm_unit_t m_pwm_unit;
    mcpwm_timer_t m_pwm_timer;
    uint8_t m_pin_number;

    // Added for extra saftey when fully stopping electric motors
    ControlMode_te ControlMode_e;

    // To check if the motor was succesfully initialised and can function
    MotorState_te MotorState_e;

    // Variables to hold the threshold of the PWM for electric motors
    uint16_t MinPWMValue_u16;
    uint16_t MaxPWMValue_u16;

    // replacement for the percentage
    double ActualPwmDuty_u16;

    // Generates PWM for ESC control
    void UpdatePWM_v(uint16_t newPWMValue);

    // to transform from percentage to PWM according to calibration

    // If the motor is active/inactive
    bool IsActive_b;

    /*
    
    // Get the saved configuration from the non-volatile memory
    void GetSavedConfiguration() noexcept(false);

    // Save configuration to the non-volatile memory
    void SaveConfiguration() noexcept(false);


    */
    // configuration of the PWM channel the motor is attached to
    ledc_channel_config_t ChannelConfig_pst;
};

#endif /* _MOTOR_MOTOR_H_ */
