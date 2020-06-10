#include "sdkconfig.h"
#include "DroneHandler.h"
#include "wifi.h"
#include <cstring>
#include "quad_rest.h"

static char tag[] = "mpu6050";
#define I2C_ADDRESS 0x68
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1 0x6B

extern "C"
{
  void app_main(void);
}



void app_main(void)
{
  TaskHandle_t main_handle = nullptr;
  DroneHandler * handler_ob = DroneHandler::getSingletonInstance();
  handler_ob->init(); // init here to avoid deadlocks
  if (pdPASS == xTaskCreate(DroneHandler::quad_task, "quad_main", 4096, nullptr, configMAX_PRIORITIES -1, &main_handle))
  {
    ap_wifi_driver wifi_driver(ap_wifi_driver::default_quad_ap(), ap_wifi_driver::wifi_handler);
    wifi_driver.start_acces_point();
  }
}
