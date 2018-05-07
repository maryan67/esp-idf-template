#include <string>
#include "sdkconfig.h"
#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "MPU6050.h"

extern "C"
{
  void app_main (void);
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "nvs.h"
#include "nvs_flash.h"
}

void app_main (void)
{
  MPU6050 *sensor = new MPU6050();
  sensor->init(GPIO_NUM_32,GPIO_NUM_33);
  while(1 == 1)
  {
    sensor->readAccel();
    sensor->readGyro();

    printf("Accel: %i %i %i" , sensor->getAccelX(),sensor->getAccelY(),sensor->getAccelZ());
    printf("\n");
    printf("Gyro: %i %i %i" , sensor->getGyroX(),sensor->getGyroY(),sensor->getGyroZ());
    printf("\n");
    vTaskDelay(1000/ portTICK_PERIOD_MS);
  }
  
}

