/*
 * motor.h
 *
 *  Created on: Apr 22, 2018
 *      Author: secar
 */

#ifndef _MOTOR_MOTOR_H_
#define _MOTOR_MOTOR_H_

#include <stdint.h>


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
    MotorDriver(uint8_t pinNumber);

    // Starts the electric motor
    uint8_t StartMotor_u16();

    // arms the esc of the motor( should hear 2 beeps )
    void arm();

    // THIS SHOULD ONLY BE USED ON A BIG EMERGENCY
    uint8_t EmergencyStopMotor();
    // THIS SHOULD ONLY BE USED ON A BIG EMERGENCY

    // Set the percentage of the throttle
    uint8_t SetThrottlePercentage(uint8_t PercentageOfThrottle);

    // Used for extra-saftey when stopping motor
    void SetControlMode_v(ControlMode_te ControlMode_e);

    // Calibrate the ESC- probabily requires serial connection to USB
    // Future versions may include this function into the mobile application
    static void Calibrate();

private:
    // The pin number the motor will be attached to
    uint8_t PinNumber_u8;

    // The actual percentage of motor throttle
    uint8_t PercentageOfThrottle;

    // Added for extra saftey when fully stopping electric motors
    ControlMode_te ControlMode_e;

    // To check if the motor was succesfully initialised and can function
    MotorState_te MotorState_e;

    // Generates PWM for ESC control
    void WritePWM_v();

    // Deataches the current electric motor from the master
    void DeatachMotor_v();

    // for reading the value on the esc
    void ReadPwm_v();

    // If the motor is active/inactive
    bool IsActive_b;

    // Get the saved configuration from the non-volatile memory
    void GetSavedConfiguration();

    // Save configuration to the non-volatile memory
    void SaveConfiguration();



};

#endif /* _MOTOR_MOTOR_H_ */
