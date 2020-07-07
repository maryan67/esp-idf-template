/*
 * DroneHandler.h
 *
 *  Created on: Apr 22, 2018
 *      Author: secar
 */

#ifndef _DRONE_HANDLER_DRONEHANDLER_H_
#define _DRONE_HANDLER_DRONEHANDLER_H_

// #define ANGLE_MONITOR
#include "DroneHandler_config.h"
#include "motor.h"
#include "MPU6050.h"
#include "pid.h"

// #define ANGLE_MONITOR
// #define GAIN_MONITOR
// #define TIME_MON
// #define GAIN_MONITOR
// #define ANGLE_MONITOR

// #define OSCILOSCOPE_MOTORS
typedef enum DroneHandlerState

{

    HANDLER_NOT_INITIALISED = 0,
    HANDLER_INITIALISED = 99,
    IDLE = 1,
    CALIBRATION = 2,
    FLIGHT = 3,

} DroneHandlerState_te;

typedef enum pid_axes
{
    PID_DEFAULT = 0,
    PID_X,
    PID_Y,
    PID_Z,
} quad_pid_axes;

class DroneHandler
{

public:
    // Singleton because it's only one commander/ master controller
    static DroneHandler *getSingletonInstance();

    void init();

    void set_throttle(char throttle_percentage)
    {
        // cannot modify throttle if flight mode not set
        if (DroneHandlerState_e != FLIGHT)
            return;
        if ((throttle_percentage <= 100))
        {

            int new_throttle = 1000 + ((throttle_percentage * 1000) / 100);
            for (MotorDriver *motor : motors)
            {
                if (new_throttle <= 2000 && new_throttle >= 1000)
                    motor->SetPWMThrottleValue_v(new_throttle);
            }
        }
    }

    // StartPoint for the feedback loop
    static void quad_task(void *pvParam); // freeRTOS supports only C callbacks

    PID *get_pid(quad_pid_axes axe);
    void set_pid(quad_pid_axes axle, double kp, double kd, double ki);

    DroneHandlerState_te get_state()
    {
        return this->DroneHandlerState_e;
    }

    void start_motors();

    SemaphoreHandle_t get_loop_sema()
    {
        return loop_sema;
    }

    void switch_state(DroneHandlerState_te new_state)
    {
        DroneHandlerState_e = new_state;
    }

    void test_motors_diff();

private:
    // Private because singleton usage
    DroneHandler();

    // The function for the actual feedback loop
    void ComputeAndUpdateThrottle();

    void calibrate_esc();

    // pointer to the MPU unit object
    MPU6050 *orientationSensor_po;

    //pointer to the task handler after creating and starting the
    //feedback loop
    void *FeedBackLoop_pv;

    SemaphoreHandle_t loop_sema;
    // PID object
    PID *pid_poX;
    PID *pid_poY;
    PID *pid_poZ;

    // Pointer to the singleton instance of the class
    static DroneHandler *DroneHandlerSingleton_po;

    // Array containing the motors of the drone
    MotorDriver *motors[NUMBER_OF_MOTORS];

    // State of the drone control object
    DroneHandlerState_te DroneHandlerState_e;

    bool install_i2c_driver(char sda_pin, char clk_pin);

    void init_mpu_to_default();
};

#endif /* _DRONE_HANDLER_DRONEHANDLER_H_ */
