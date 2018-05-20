/*
 * DroneHandler.cpp
 *
 *  Created on: Apr 27, 2018
 *      Author: secar
 */

#include "DroneHandler.h"

DroneHandler* DroneHandler::DroneHandlerSingleton_po = NULL;

DroneHandler::DroneHandler()
{
    this->DroneHandlerState_e = HANDLER_NOT_INITIALISED;
}

DroneHandler *DroneHandler::getSingletonInstance()
{
    if (DroneHandlerSingleton_po == NULL)
    {
        static DroneHandler *droneHandler_o = new DroneHandler();
        droneHandler_o->init();
        DroneHandlerSingleton_po = droneHandler_o;
    }
    return DroneHandlerSingleton_po;
}

void DroneHandler::init()
{
    ledc_timer_config_t timer_st1;
    timer_st1.timer_num = LEDC_TIMER_0;
    ledc_channel_config_t channel1;
    channel1.channel = LEDC_CHANNEL_0;
    channel1.gpio_num = 15;
    channel1.timer_sel = LEDC_TIMER_0;

    ledc_timer_config_t timer_st2;
    timer_st2.timer_num = LEDC_TIMER_1;
    ledc_channel_config_t channel2;
    channel2.channel = LEDC_CHANNEL_1;
    channel2.gpio_num = 16;
    channel2.timer_sel = LEDC_TIMER_1;

    ledc_timer_config_t timer_st3;
    timer_st3.timer_num = LEDC_TIMER_2;
    ledc_channel_config_t channel3;
    channel3.channel = LEDC_CHANNEL_2;
    channel3.gpio_num = 17;
    channel3.timer_sel = LEDC_TIMER_2;

    ledc_timer_config_t timer_st4;
    timer_st4.timer_num = LEDC_TIMER_3;
    ledc_channel_config_t channel4;
    channel4.channel = LEDC_CHANNEL_3;
    channel4.gpio_num = 18;
    channel4.timer_sel = LEDC_TIMER_3;

    motors[FRONT_LEFT_MOTOR] = new MotorDriver(&timer_st1, &channel1);
    motors[FRONT_RIGHT_MOTOR] = new MotorDriver(&timer_st2, &channel2);
    motors[BACK_LEFT_MOTOR] = new MotorDriver(&timer_st3, &channel3);
    motors[BACK_RIGHT_MOTOR] = new MotorDriver(&timer_st4, &channel4);

    this->orientationSensor_po = new MPU6050();
    this->orientationSensor_po->init(GPIO_NUM_21,GPIO_NUM_22);
    this->DroneHandlerState_e =HANDLER_INITIALISED;
    xTaskStartMotors();


}

void DroneHandler::Calibrate()
{
    if(this->DroneHandlerState_e == HANDLER_INITIALISED)
    {
        for(uint8_t index =0 ; index < 4; index++)
        {
            motors[index]->Calibrate();
        }
    }
    else throw 1; // ned update
}

void DroneHandler::xTaskStartMotors()
{
    if(this->DroneHandlerState_e == HANDLER_INITIALISED)
    {
        for(uint8_t index =0 ; index < 4; index++)
        {
            this->motors[index]->StartMotor_u16();
            this->motors[index]->armLow();
            vTaskDelay(1000/portTICK_PERIOD_MS);
            this->motors[index]->SetThrottlePercentage(defaultThrottleLevel);
        }
    // call the feedback loop start point and should work
    }

}
