/*
 * DroneHandler.cpp
 *
 *  Created on: Apr 27, 2018
 *      Author: secar
 */

#include "DroneHandler.h"
#include <math.h>
#include "sys/time.h"

DroneHandler *DroneHandler::DroneHandlerSingleton_po = NULL;

DroneHandler::DroneHandler()
{
    this->DroneHandlerState_e = HANDLER_NOT_INITIALISED;
    this->FeedBackLoop_pv = NULL;
}

// DroneHandler::~DroneHandler()
// {
//     vTaskDelete(this->FeedBackLoop_pv);
// }

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
    // Configuring the free timers for the motor driver
    ledc_timer_config_t timer_st1;

    ledc_channel_config_t channel1;
  
    channel1.gpio_num = 15;


    ledc_timer_config_t timer_st2;

    ledc_channel_config_t channel2;

    channel2.gpio_num = 16;


    ledc_timer_config_t timer_st3;

    ledc_channel_config_t channel3;

    channel3.gpio_num = 17;
 

    ledc_timer_config_t timer_st4;

    ledc_channel_config_t channel4;

    channel4.gpio_num = 18;


    // Initialise the motor drivers assigning it to the positions and
    // pins and configuring them based on resulted configs
    motors[FRONT_LEFT_MOTOR] = new MotorDriver(&timer_st1, channel1);
    motors[FRONT_RIGHT_MOTOR] = new MotorDriver(&timer_st2, channel2);
    motors[BACK_LEFT_MOTOR] = new MotorDriver(&timer_st3, channel3);
    motors[BACK_RIGHT_MOTOR] = new MotorDriver(&timer_st4, channel4);

    this->orientationSensor_po = new MPU6050();
    this->orientationSensor_po->init(GPIO_NUM_21, GPIO_NUM_22);
    //this->orientationSensor_po->CalibrateGyroscope();

    //PID(double dt, double max, double min,
    //double Kp, double Kd, double Ki );
    // Initialize the pid Object
    this->pid_poX = new PID(PID_UPDATE_INTERVAL, PID_MAX_STEP,
                           PID_MIN_STEP, KP, KD, KI);
                           this->pid_poY = new PID(PID_UPDATE_INTERVAL, PID_MAX_STEP,
                           PID_MIN_STEP, KP, KD, KI);
    this->DroneHandlerState_e = HANDLER_INITIALISED;

    // Function that calls the startpoint of the feedback
    // looping
   // Calibrate();
    xTaskStartMotors();
}

void DroneHandler::Calibrate()
{
    if (this->DroneHandlerState_e == HANDLER_INITIALISED)
    {
        for (uint8_t index = 0; index < 4; index++)
        {

            motors[index]->Calibrate();
        }
    }
    else
        throw 1; // ned update
}

void DroneHandler::xTaskStartMotors()
{
    if (this->DroneHandlerState_e == HANDLER_INITIALISED)
    {
        for (uint8_t index = 0; index < 4; index++)
        {
            this->motors[index]->StartMotor_u16();
              this->motors[index]->armLow();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        for (uint8_t index = 0; index < 4; index++)
        {
            this->motors[index]->SetPWMThrottleValue_v(1300);
        }
        // xTaskCreate(&ComputeAndUpdateThrottle,"FeedbackLoop",2048,NULL,
        // this->FeedBackLoop_pv);

        this->DroneHandlerState_e = HANDLER_FLYING;
        ComputeAndUpdateThrottle();

      //  Calibrate();
    }
}

void DroneHandler::ComputeAndUpdateThrottle()
{
    double ax, ay, az = 0;
    double gainX, gainY = 0;
    float yRotationAcc = 0;
    float xRotationAcc = 0;
    double yRotationGyro = 0, xRotationGyro = 0;

    bool start = true;
    float radToDeg = 180 / 3.141592;
    long int TimePassed = 0;
    while (this->DroneHandlerState_e == HANDLER_FLYING)
    {

        //execute readings of the sensor through I2C
        //  orientationSensor_po->readAccel();
        orientationSensor_po->readData();
        ax = orientationSensor_po->getAccelX();
        ay = orientationSensor_po->getAccelY();
        az = orientationSensor_po->getAccelZ();
        // x axis
        // y axis
        yRotationAcc = (radToDeg)*atan(ax / sqrt(ay * ay + az * az));
        xRotationAcc = (radToDeg)*atan(ay / sqrt(ax * ax + az * az));

        if (start)
        {
            yRotationGyro = yRotationAcc;
            xRotationGyro = xRotationAcc;
            start = false;
        }
        else
        {
            yRotationGyro = ((yRotationGyro + (orientationSensor_po->getGyroY() / 131.0) * (PID_UPDATE_INTERVAL)) * 0.98) +
                            (0.02 * yRotationAcc);
            xRotationGyro = ((xRotationGyro + (orientationSensor_po->getGyroX() / 131.0) * (PID_UPDATE_INTERVAL)) * 0.98) +
                            (0.02 * xRotationAcc);
        }

        gainX = pid_poX->calculate(0, yRotationGyro);
        gainY = pid_poY->calculate(0, xRotationGyro);


//        this->motors[FRONT_RIGHT_MOTOR]->SetPWMThrottleValue_v(0);
      
            this->motors[FRONT_RIGHT_MOTOR]->SetPWMDutyGain(gainX,true);
            this->motors[BACK_LEFT_MOTOR]->SetPWMDutyGain(gainX,false);
            this->motors[FRONT_LEFT_MOTOR]->SetPWMDutyGain(gainY,true);
            this->motors[BACK_RIGHT_MOTOR]->SetPWMDutyGain(gainY,false);

        

          

        
        vTaskDelay((PID_UPDATE_INTERVAL*1000) / portTICK_PERIOD_MS);
    }
}
