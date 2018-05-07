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


typedef enum DroneHandlerState
{

    HANDLER_NOT_INITIALISED = 0,
    HANDLER_INITIALISED,
    UNKNOWN_STATE

}DroneHandlerState_te;

class DroneHandler
{

public :


    // First of all it will calibrate to retain the knowledge of "front" and "Back" concepts
    // probably the configs will be writtten to EEPROM
    bool Calibrate ();


    // Singleton because it's only one commander/ master controller
    DroneHandler * getSingletonInstance();




private:

    // Private because singleton usage
    DroneHandler();

    // Pointer to the singleton instance of the class
    DroneHandler * DroneHandlerSingleton_po;

    // Array containing the motors of the drone
    MotorDriver motors[NUMBER_OF_MOTORS];

    // State of the drone control object
    DroneHandlerState_te DroneHandlerState_e;

};


#endif /* _DRONE_HANDLER_DRONEHANDLER_H_ */
