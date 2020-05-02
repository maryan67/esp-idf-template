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
        DroneHandlerSingleton_po = droneHandler_o;
    }
    return DroneHandlerSingleton_po;
}

void DroneHandler::init()
{
    // Configuring the free timers for the motor driver
    ledc_timer_config_t timer_st1;

    ledc_channel_config_t channel1;

    channel1.gpio_num = 4;

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
    this->pid_poX = new PID(PID_MAX_STEP,
                            PID_MIN_STEP, KP, KD, KI);
    this->pid_poY = new PID(PID_MAX_STEP,
                            PID_MIN_STEP, KP, KD, KI);
    this->pid_poZ = new PID(PID_MAX_STEP, PID_MIN_STEP, 0, 0, 0); // for yaw
    this->DroneHandlerState_e = HANDLER_INITIALISED;
    loop_sema = xSemaphoreCreateBinary();
    xSemaphoreGive(loop_sema);
}

void DroneHandler::calibrate_esc()
{
    for (MotorDriver *motor : motors)
    {
        motor->Calibrate();
    }
}

void DroneHandler::start_motors()
{
    if (this->DroneHandlerState_e == HANDLER_INITIALISED)
    {
        for (uint8_t index = 0; index < 4; index++)
        {
            this->motors[index]->StartMotor_u16();
            this->motors[index]->armLow();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        this->DroneHandlerState_e = FLIGHT;
    }
}
void DroneHandler::quad_task(void *pvParam)
{
    DroneHandler *handler_obj = DroneHandler::getSingletonInstance();

    while (1)
    {

        switch (handler_obj->get_state())
        {
        // fly the quad
        case FLIGHT:
        {
            handler_obj->start_motors();
            handler_obj->ComputeAndUpdateThrottle();
        }
        break;

        // calibrate ESC
        case CALIBRATION:
        {
            xSemaphoreTake(handler_obj->get_loop_sema(), portMAX_DELAY);
            // calibrate and set everything back to idle;
            handler_obj->calibrate_esc();
            handler_obj->switch_state(IDLE);
            xSemaphoreGive(handler_obj->get_loop_sema());
        }
        break;

        default:
            break;
        }

        // give time to update the state
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void DroneHandler::ComputeAndUpdateThrottle()
{
    double ax, ay, az = 0;
    double gainX, gainY = 0;
    float yRotationAcc = 0;
    float xRotationAcc = 0;
    float zRotationAcc = 0;
    double yRotationGyro = 0, xRotationGyro, zRotationGyro = 0;

    bool start = true;
    float radToDeg = 180 / 3.141592;
    double last_timestamp = esp_timer_get_time() / 1000;
    double cycle_time = 0;

    while (this->DroneHandlerState_e == FLIGHT)
    {

        cycle_time = (esp_timer_get_time() / 1000) - last_timestamp;
        last_timestamp = esp_timer_get_time() / 1000;
        //execute readings of the sensor through I2C
        //  orientationSensor_po->readAccel();
        orientationSensor_po->readData();
        ax = orientationSensor_po->getAccelX();
        ay = orientationSensor_po->getAccelY();
        az = orientationSensor_po->getAccelZ();

        // x axis -pitch

        // y axis - roll
        // z axis -yaw
        yRotationAcc = (radToDeg)*atan(ax / sqrt(ay * ay + az * az));
        xRotationAcc = (radToDeg)*atan(ay / sqrt(ax * ax + az * az));

        // first time init from accel
        if (start)
        {
            yRotationGyro = yRotationAcc;
            xRotationGyro = xRotationAcc;
            zRotationGyro = 0U; // yaw means the starting position
            start = false;
            xSemaphoreGive(loop_sema);
            continue;
        }
        else
        {
            yRotationGyro = ((yRotationGyro + (orientationSensor_po->getGyroY() / 131.0) * (cycle_time / 1000)) * 0.98) +
                            (0.02 * yRotationAcc);
            xRotationGyro = ((xRotationGyro + (orientationSensor_po->getGyroX() / 131.0) * (cycle_time / 1000)) * 0.98) +
                            (0.02 * xRotationAcc);
            zRotationGyro = ((zRotationGyro + (orientationSensor_po->getGyroZ() / 131.0) * (cycle_time / 1000)) * 0.98) +
                            (0.02 * xRotationAcc);
        }

#ifdef ANGLE_MONITOR
        printf("Pitch angle: %f\n", xRotationGyro);
        printf("Roll angle: %f\n", yRotationGyro);
#endif

        // lock to sync threads
        xSemaphoreTake(loop_sema, portMAX_DELAY);
        gainX = pid_poX->calculate(0, xRotationGyro, cycle_time);
        gainY = pid_poY->calculate(0, yRotationGyro, cycle_time);

#ifdef GAIN_MONITOR
        printf("Pitch gain: %f\n", gainX);
        printf("Roll gain: %f\n", gainY);
#endif

        this->motors[FRONT_RIGHT_MOTOR]->SetPWMDutyGain(static_cast<int16_t>((xRotationGyro < 0 ? gainX : -1 * gainX) + (yRotationGyro > 0 ? -1 * gainY : gainY)), true);
        this->motors[FRONT_LEFT_MOTOR]->SetPWMDutyGain(static_cast<int16_t>((xRotationGyro < 0 ? gainX : -1 * gainX) + (yRotationGyro > 0 ? gainY : -1 * gainY)), true);
        this->motors[BACK_RIGHT_MOTOR]->SetPWMDutyGain(static_cast<int16_t>((xRotationGyro < 0 ? -1 * gainX : gainX) + (yRotationGyro > 0 ? -1 * gainY : gainY)), true);
        this->motors[BACK_LEFT_MOTOR]->SetPWMDutyGain(static_cast<int16_t>((xRotationGyro < 0 ? -1 * gainX : gainX) + (yRotationGyro > 0 ? gainY : -1 * gainY)), true);

        xSemaphoreGive(loop_sema);
    }

    // stop motos if loop exits, we don't want a crash
    for (MotorDriver *motor : motors)
    {
        motor->stop_motor();
    }
}

PID *DroneHandler::get_pid(quad_pid_axes axle)
{
    switch (axle)
    {
    case PID_X:
        return pid_poX;
    case PID_Y:
        return pid_poY;
    case PID_Z:
        return pid_poZ;
    default:
        return nullptr;
        break;
    }
}

void DroneHandler::set_pid(quad_pid_axes axle, PID *new_pid)
{
    switch (axle)
    {
    case PID_X:
        delete pid_poX;
        pid_poX = new_pid;
        break;
    case PID_Y:
        delete pid_poY;
        pid_poY = new_pid;
        break;
    case PID_Z:
        delete pid_poZ;
        pid_poZ = new_pid;
        break;
    default:
        return;
    }
}
