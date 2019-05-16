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

}DroneHandlerState_te;



class DroneHandler
{

public :


    // Auto calibration of the esc's if needed
    void Calibrate ();

    
    // Singleton because it's only one commander/ master controller
    static DroneHandler * getSingletonInstance();

    void setDefaultThrottle(uint8_t ThrottleLevel_u8 )
    {
        this->defaultThrottleLevel = ThrottleLevel_u8;
    }

    void init();




private:

    // StartPoint for the feedback loop
    void xTaskStartMotors(); 
    
    // Private because singleton usage
    DroneHandler();

    // pointer to the MPU unit object
    MPU6050* orientationSensor_po;

    // PID object
    PID* pid_poX ;
    PID* pid_poY ;
    
    // The function for the actual feedback loop
    void ComputeAndUpdateThrottle();

    // Pointer to the singleton instance of the class
    static DroneHandler * DroneHandlerSingleton_po;

    // Array containing the motors of the drone
    MotorDriver* motors[NUMBER_OF_MOTORS];

    // State of the drone control object
    DroneHandlerState_te DroneHandlerState_e;

    //pointer to the task handler after creating and starting the
    //feedback loop
    void * FeedBackLoop_pv;
    uint8_t defaultThrottleLevel =100;

};


#endif /* _DRONE_HANDLER_DRONEHANDLER_H_ */
