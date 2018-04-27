/*
 * motor.h
 *
 *  Created on: Apr 22, 2018
 *      Author: secar
 */

#ifndef _MOTOR_MOTOR_H_
#define _MOTOR_MOTOR_H_

#include <stdint.h>
#include "driver/ledc.h"
#include "GeneralErrorCodes.h"
#include "NVMOperator.h"
#include "motor_config.h"


typedef enum MotorState
{

    MOTOR_STATE_UNINITIALISED = 0,
    MOTOR_STATE_INITIALISED,
    MOTOR_STATE_UNKNOWN

}MotorState_te;

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
    MotorDriver(ledc_timer_config_t* TimerConfig_pst,
            ledc_channel_config_t * ChannelConfig_pst)const throw();

    // Starts the electric motor
    void StartMotor_u16() const throw (GeneralErrorCodes_te);

    // arms the esc of the motor( should hear 2 beeps )
    void arm();

    // THIS SHOULD ONLY BE USED ON A BIG EMERGENCY
    void EmergencyStopMotor() const throw (GeneralErrorCodes_te);
    // THIS SHOULD ONLY BE USED ON A BIG EMERGENCY

    // Set the percentage of the throttle
    void SetThrottlePercentage(uint8_t PercentageOfThrottle) const throw (GeneralErrorCodes_te);

    // Used for extra-saftey when stopping motor
    void SetControlMode_v(ControlMode_te ControlMode_e);

    // Calibrate the ESC- probabily requires serial connection to USB
    // Future versions may include this function into the mobile application
    static void Calibrate();

private:

    // The actual percentage of motor throttle
    uint8_t PercentageOfThrottle;

    // Added for extra saftey when fully stopping electric motors
    ControlMode_te ControlMode_e;

    // To check if the motor was succesfully initialised and can function
    MotorState_te MotorState_e;

    // Variables to hold the threshold of the PWM for electric motors
    uint16_t MinPWMValue_u16;
    uint16_t MaxPWMValue_u16;

    // Generates PWM for ESC control
    void UpdatePWM_v();

    // to transform from percentage to PWM according to calibration
    uint16_t PercentageToPWMMicroseconds(uint8_t PercentageToMove);

    // Deataches the current electric motor from the master
    //void DeatachMotor_v();

    // for reading the value on the esc
    //void ReadPwm_v();

    // If the motor is active/inactive
    bool IsActive_b;

    // Get the saved configuration from the non-volatile memory
    void GetSavedConfiguration() const throw();

    // Save configuration to the non-volatile memory
    void SaveConfiguration();
    // configuration of the PWM channel the motor is attached to
    ledc_channel_config_t * ChannelConfig_pst;



};

#endif /* _MOTOR_MOTOR_H_ */
