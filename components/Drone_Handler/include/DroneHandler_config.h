/*
 * DroneHandler_config.h
 *
 *  Created on: Apr 22, 2018
 *      Author: secar
 */

// config file for the DroneHandler

#ifndef MAIN_DRONE_HANDLER_DRONEHANDLER_CONFIG_H_
#define MAIN_DRONE_HANDLER_DRONEHANDLER_CONFIG_H_




#define FRONT_LEFT_MOTOR 0
#define FRONT_RIGHT_MOTOR 1
#define BACK_LEFT_MOTOR 2
#define BACK_RIGHT_MOTOR 3
 



#define NUMBER_OF_MOTORS 4



// PID constants configuration
#define KP 2.6 //2.1
#define KD 0.90
#define KI 0

// PID minimum and maximum steps (in throttle percentage)

#define PID_MAX_STEP 400
#define PID_MIN_STEP -400

// PID update interval in seconds
#define PID_UPDATE_INTERVAL 0.025


#endif /* MAIN_DRONE_HANDLER_DRONEHANDLER_CONFIG_H_ */
//https://www.youtube.com/watch?v=JBvnB0279-Q