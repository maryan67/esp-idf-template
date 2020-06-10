/*
 * DroneHandler.cpp
 *
 *  Created on: Apr 27, 2018
 *      Author: secar
 */

#include "DroneHandler.h"
#include <math.h>
#include "sys/time.h"
#include "MPU6050_6Axis_MotionApps20.h"

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
    if (!install_i2c_driver(21, 22))
    {
        vTaskDelete(nullptr);
        // do nothing
    }
    init_mpu_to_default();

    // Initialise the motor drivers assigning it to the positions and
    // pins and configuring them based on resulted configs

    try
    {

        motors[FRONT_LEFT_MOTOR] = new MotorDriver(MCPWM_UNIT_0, MCPWM_TIMER_0, 0);
        motors[FRONT_RIGHT_MOTOR] = new MotorDriver(MCPWM_UNIT_0, MCPWM_TIMER_1, 16);
        motors[BACK_LEFT_MOTOR] = new MotorDriver(MCPWM_UNIT_1, MCPWM_TIMER_0, 17);
        motors[BACK_RIGHT_MOTOR] = new MotorDriver(MCPWM_UNIT_1, MCPWM_TIMER_1, 18);
    }
    catch (int e)
    {
        if (e == HAL_ERROR)
            vTaskDelete(NULL);
    }

    this->pid_poX = new PID(PID_MAX_STEP,
                            PID_MIN_STEP, KP, KD, KI);
    this->pid_poY = new PID(PID_MAX_STEP,
                            PID_MIN_STEP, KP, KD, KI);
    this->pid_poZ = new PID(PID_MAX_STEP, PID_MIN_STEP, 0, 0, 0); // for yaw

    start_motors();
    this->DroneHandlerState_e = HANDLER_INITIALISED;
    loop_sema = xSemaphoreCreateBinary();
    xSemaphoreGive(loop_sema);
}

void DroneHandler::calibrate_esc()
{
    for (MotorDriver *motor : motors)
    {
        motor->armHigh();
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    for (MotorDriver *motor : motors)
    {
        motor->armLow();
    }
}

void DroneHandler::start_motors()
{
    
        for (uint8_t index = 0; index < 4; index++)
        {
            this->motors[index]->init_motor();
            this->motors[index]->armLow();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        this->DroneHandlerState_e = FLIGHT;
    
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
           

// Special test fw
#ifdef OSCILOSCOPE_MOTORS
            handler_obj->test_motors_diff();
#else
            handler_obj->ComputeAndUpdateThrottle();
#endif
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
    double gainX, gainY, gainZ = 0;
    float yRotationAcc = 0;
    float xRotationAcc = 0;
    float zRotationAcc = 0;
    double yRotationGyro = 0, xRotationGyro = 0, zRotationGyro = 0;

    bool start = true;
    float radToDeg = 180 / 3.141592;
    double last_timestamp = esp_timer_get_time() / 1000;
    double cycle_time = 0;

    Quaternion q;             // [w, x, y, z]         quaternion container
    VectorFloat gravity;      // [x, y, z]            gravity vector
    float ypr[3];             // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    uint16_t packetSize = 42; // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;       // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64];   // FIFO storage buffer
    uint8_t mpuIntStatus;
    while (this->DroneHandlerState_e == FLIGHT)
    {

        //execute readings of the sensor through I2C
        //  orientationSensor_po->readAccel();

        mpuIntStatus = orientationSensor_po->getIntStatus();
        // get current FIFO count
        fifoCount = orientationSensor_po->getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount == 1024)
        {
            // reset so we can continue cleanly
            orientationSensor_po->resetFIFO();

            // otherwise, check for DMP data ready interrupt frequently)
        }
        else if (mpuIntStatus & 0x02)
        {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize)
                fifoCount = orientationSensor_po->getFIFOCount();

            // read a packet from FIFO

            orientationSensor_po->getFIFOBytes(fifoBuffer, packetSize);
            orientationSensor_po->dmpGetQuaternion(&q, fifoBuffer);
            orientationSensor_po->dmpGetGravity(&gravity, &q);
            orientationSensor_po->dmpGetYawPitchRoll(ypr, &q, &gravity);
            zRotationGyro = ypr[0] * 180 / M_PI;
            xRotationGyro = ypr[2] * 180 / M_PI;
            yRotationGyro = ypr[1] * 180 / M_PI;
        }

        // first time init from accel

#ifdef ANGLE_MONITOR
        printf("Pitch angle: %f\n", xRotationGyro);
        printf("Roll angle: %f\n", yRotationGyro);
        printf("Yaw angle: %f\n", zRotationGyro);
#endif

        // lock to sync threads
        xSemaphoreTake(loop_sema, portMAX_DELAY);
        cycle_time = (esp_timer_get_time() / 1000) - last_timestamp;
        last_timestamp = esp_timer_get_time() / 1000;
        gainX = pid_poX->calculate(0, xRotationGyro, cycle_time);
        gainY = pid_poY->calculate(0, yRotationGyro, cycle_time);
        gainZ = pid_poZ->calculate(0, zRotationGyro, cycle_time);

#ifdef GAIN_MONITOR
        printf("Pitch gain: %f\n", gainX);
        printf("Roll gain: %f\n", gainY);
#endif

        this->motors[FRONT_RIGHT_MOTOR]->SetPWMDutyGain(static_cast<int16_t>((xRotationGyro < 0 ? gainX : -1 * gainX) + (yRotationGyro > 0 ? -1 * gainY : gainY) + (zRotationGyro > 0 ? gainZ : -1 * gainZ)), true);
        this->motors[FRONT_LEFT_MOTOR]->SetPWMDutyGain(static_cast<int16_t>((xRotationGyro < 0 ? gainX : -1 * gainX) + (yRotationGyro > 0 ? gainY : -1 * gainY) + (zRotationGyro > 0 ? -1 * gainZ : gainZ)), true);
        this->motors[BACK_RIGHT_MOTOR]->SetPWMDutyGain(static_cast<int16_t>((xRotationGyro < 0 ? -1 * gainX : gainX) + (yRotationGyro > 0 ? -1 * gainY : gainY) + (zRotationGyro > 0 ? -1 * gainZ : gainZ)), true);
        this->motors[BACK_LEFT_MOTOR]->SetPWMDutyGain(static_cast<int16_t>((xRotationGyro < 0 ? -1 * gainX : gainX) + (yRotationGyro > 0 ? gainY : -1 * gainY) + (zRotationGyro > 0 ? gainZ : -1 * gainZ)), true);

        xSemaphoreGive(loop_sema);
        vTaskDelay(10/portTICK_PERIOD_MS);
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

#define I2C_FAIL_CHECK(x) \
    if ((x) != ESP_OK)    \
        return false;
bool DroneHandler::install_i2c_driver(char sda_pin, char clk_pin)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)sda_pin;
    conf.scl_io_num = (gpio_num_t)clk_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    I2C_FAIL_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    I2C_FAIL_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    return true;
}

void DroneHandler::init_mpu_to_default()
{
    orientationSensor_po = new MPU6050();
    orientationSensor_po->initialize();
    orientationSensor_po->dmpInitialize();

    // This need to be setup individually

    orientationSensor_po->setXGyroOffset(220);
    orientationSensor_po->setYGyroOffset(76);
    // always start at 0
    orientationSensor_po->setZGyroOffset(20);
    orientationSensor_po->setZAccelOffset(1788);
    orientationSensor_po->setDMPEnabled(true);
}

void DroneHandler::test_motors_diff()
{
    motors[FRONT_LEFT_MOTOR]->SetPWMThrottleValue_v(1200);
    motors[FRONT_RIGHT_MOTOR]->SetPWMThrottleValue_v(1400);
    motors[BACK_LEFT_MOTOR]->SetPWMThrottleValue_v(1600);
    motors[BACK_RIGHT_MOTOR]->SetPWMThrottleValue_v(1800);
}
