#include <string>
#include "sdkconfig.h"
#include "DroneHandler.h"

static char tag[] = "mpu6050";
#define I2C_ADDRESS 0x68
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1 0x6B

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
  void app_main(void);
}

void app_main(void)
{
  
  DroneHandler* handler = DroneHandler::getSingletonInstance();
}
