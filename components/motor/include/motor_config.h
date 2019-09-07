/*
 * motor_config.h
 *
 *  Created on: Apr 22, 2018
 *      Author: secar
 */

#ifndef MAIN_MOTOR_MOTOR_CONFIG_H_
#define MAIN_MOTOR_MOTOR_CONFIG_H_


#define DUTY_RESOLUTION 12
#define UPDATE_FREQUENCY 500 //divide a second by this value and you obtain the maximum value of miliseconds to write (arduino-like, eg 500hz means appx 2000(microseconds)
#define PWM_CHANNEL_REGISTER_SIZE  10//define the step (register full means 100%)
#define STARTING_PERCENTAGE_OF_THROTTLE 0



#endif /* MAIN_MOTOR_MOTOR_CONFIG_H_ */
