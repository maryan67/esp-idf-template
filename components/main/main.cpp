#include <string>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "MPU6050.h"
#include "HC_SR04.h"
#include "motor.h"
#include "DroneHandler.h"


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
#include <stdio.h>
}

void app_main(void)
{

  ledc_timer_config_t timer_st;
  
  timer_st.timer_num = LEDC_TIMER_0; 
  ledc_channel_config_t channel;
  channel.channel = LEDC_CHANNEL_0;
  channel.gpio_num = 18;
  channel.timer_sel =  LEDC_TIMER_0;

 
 MotorDriver *motor = new MotorDriver(&timer_st,&channel);
 motor->StartMotor_u16();
 motor->armLow();
 vTaskDelay(2000/portTICK_PERIOD_MS);
 motor->SetThrottlePercentage(50);
 



  


 
}
