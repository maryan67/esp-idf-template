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
    this->pid_po = new PID(PID_UPDATE_INTERVAL, PID_MAX_STEP,
                           PID_MIN_STEP, KP, KD, KI);
    this->DroneHandlerState_e = HANDLER_INITIALISED;

    // Function that calls the startpoint of the feedback
    // looping
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
            this->motors[index]->SetPWMDutyValue_v(1300);
        }
        // xTaskCreate(&ComputeAndUpdateThrottle,"FeedbackLoop",2048,NULL,
        // this->FeedBackLoop_pv);

        this->DroneHandlerState_e = HANDLER_FLYING;
        ComputeAndUpdateThrottle();
        Calibrate();
    }
}

void DroneHandler::ComputeAndUpdateThrottle()
{
    double ax, ay, az = 0;
    double gain = 0;
    double kp = KP;
    struct timeval now = {0};

    double yRotationGyro = 0, xRotationGyro = 0;
    double NowInMilliseconds;
    gettimeofday(&now, NULL);

    double LastTimeStamp = (now.tv_sec * 1000LL) +
                           (now.tv_usec / 1000LL);

    bool start = true;

    long int TimePassed = 0;
    while (this->DroneHandlerState_e == HANDLER_FLYING)
    {

        if (TimePassed % 4000 == 0)
        {
           // kp += 0.005;
            delete pid_po;
            pid_po = new PID(PID_UPDATE_INTERVAL, PID_MAX_STEP,
                             PID_MIN_STEP, KP, KD, KI);
        }
        //execute readings of the sensor through I2C
        //  orientationSensor_po->readAccel();
        orientationSensor_po->readData();
        ax = orientationSensor_po->getAccelX();
        ay = orientationSensor_po->getAccelY();
        az = orientationSensor_po->getAccelZ();
        // x axis
        // y axis
        double yRotation = (180 / 3.141592) * atan(ax / sqrt(pow(ay, 2) + pow(az, 2)));
        double xRotation = (180 / 3.141592) * atan(ay / sqrt(pow(ax, 2) + pow(az, 2)));
        gettimeofday(&now, NULL);

        NowInMilliseconds = (now.tv_sec * 1000LL) +
                            (now.tv_usec / 1000LL);

        if (start)
        {
            yRotationGyro = yRotation;
            xRotationGyro = xRotation;
            start = false;
        }
        else
        {
            yRotationGyro = ((yRotationGyro + (orientationSensor_po->getGyroY() / 131.0) * ((NowInMilliseconds - LastTimeStamp) / 1000)) * 0.98) +
                            (0.02 * ((-1) * yRotation));
            xRotationGyro = ((xRotationGyro + (orientationSensor_po->getGyroX() / 131.0) * ((NowInMilliseconds - LastTimeStamp) / 1000)) * 0.98) +
                            (0.02 * ((-1) * xRotation));
        }

        LastTimeStamp = NowInMilliseconds;

        printf("xRotationGyro: %lf, yRotationGyro:%lf\n",
               xRotationGyro, yRotationGyro);

        if (abs(0 - yRotationGyro) < 30)
            gain = pid_po->calculate(0, (-1) * yRotationGyro);
        if (gain < 0)
        {
            this->motors[FRONT_RIGHT_MOTOR]->SetPWMDutyGain(gain * (-1), false);
            this->motors[BACK_LEFT_MOTOR]->SetPWMDutyGain(gain * (-1), true);
        }
        if (gain > 0)
        {
            this->motors[FRONT_RIGHT_MOTOR]->SetPWMDutyGain(gain, true);
            this->motors[BACK_LEFT_MOTOR]->SetPWMDutyGain(gain, false);
        }
        printf("gain is:%lf\n", gain);
        printf("KP GAIN IS: %lf\n", KP);
        vTaskDelay(2.5 / portTICK_PERIOD_MS);
        TimePassed += 2.5;
    }

    // motor back left + motor front right and vice versa to get the axes
}
