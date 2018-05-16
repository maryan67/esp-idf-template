#include <string>
#include "sdkconfig.h"
#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "MPU6050.h"
#include "HC_SR04.h"

static char tag[] = "mpu6050";
#define I2C_ADDRESS 0x68
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1 0x6B

extern "C"
{
  void app_main(void);
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "DroneHandler.h"
}

void app_main(void)
{

  HC_SR04 *sensor = new HC_SR04(GPIO_NUM_27, GPIO_NUM_34);
  
  while (1)
  {
    sensor->startReadingDistance();
    printf("%lf\n", sensor->getDistance());
    vTaskDelay(2000/portTICK_PERIOD_MS);
  } // while (1)
  // {

  //   printf("%lf", sensor->getDistance());
  // }
  // MPU6050 *sensor = new MPU6050();

  // try
  // {
  //   sensor->init(GPIO_NUM_21, GPIO_NUM_22);
  //   while (1 == 1)
  //   {

  //     sensor->readAccel();
  //     sensor->readGyro();

  //     printf("Accel: %i %i %i", sensor->getAccelX(), sensor->getAccelY(), sensor->getAccelZ());
  //     printf("\n");
  //     printf("Gyro: %i %i %i", sensor->getGyroX(), sensor->getGyroY(), sensor->getGyroZ());
  //     printf("\n");
  //     vTaskDelay(1000 / portTICK_PERIOD_MS);
  //   }
  // }
  // catch (GeneralErrorCodes_te &error)
  // {
  //   printf("cannot read i2c");
  // }
}
