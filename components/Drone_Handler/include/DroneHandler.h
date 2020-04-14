/*
 * DroneHandler.h
 *
 *  Created on: Apr 22, 2018
 *      Author: secar
 */

#ifndef _DRONE_HANDLER_DRONEHANDLER_H_
#define _DRONE_HANDLER_DRONEHANDLER_H_

#include "DroneHandler_config.h"
#include "motor.h"
#include "MPU6050.h"
#include "pid.h"

typedef enum DroneHandlerState
{

    HANDLER_NOT_INITIALISED = 0,
    HANDLER_INITIALISED,
    HANDLER_FLYING

} DroneHandlerState_te;

class DroneHandler
{

public:
    // Auto calibration of the esc's if needed
    void Calibrate();

    // Singleton because it's only one commander/ master controller
    static DroneHandler *getSingletonInstance();

    void setDefaultThrottle(uint8_t ThrottleLevel_u8)
    {
        this->defaultThrottleLevel = ThrottleLevel_u8;
    }

    void init();

    void set_throttle(char throttle_percentage)
    {
        if ((throttle_percentage <= 100))
        {

            int new_throttle = 1000 +((throttle_percentage * 1000) / 100);
            for (MotorDriver *motor : motors)
            {
                
                motor->SetPWMThrottleValue_v(new_throttle);
            }
        }
    }

    // StartPoint for the feedback loop
    static void xTaskStartMotors(void *pvParam); // freeRTOS supports only C callbacks

    void start_motors();

    SemaphoreHandle_t get_loop_sema()
    {
        return loop_sema;
    }

private:
    // Private because singleton usage
    DroneHandler();

    // pointer to the MPU unit object
    MPU6050 *orientationSensor_po;

    // PID object
    PID *pid_poX;
    PID *pid_poY;

    // The function for the actual feedback loop
    void ComputeAndUpdateThrottle();

    // Pointer to the singleton instance of the class
    static DroneHandler *DroneHandlerSingleton_po;

    // Array containing the motors of the drone
    MotorDriver *motors[NUMBER_OF_MOTORS];

    // State of the drone control object
    DroneHandlerState_te DroneHandlerState_e;

    //pointer to the task handler after creating and starting the
    //feedback loop
    void *FeedBackLoop_pv;
    uint8_t defaultThrottleLevel = 100;

    SemaphoreHandle_t loop_sema;
};

#endif /* _DRONE_HANDLER_DRONEHANDLER_H_ */
