#include <string>
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


quad_rest rest;
void wifi_handler(void *event_handler_arg,
                  esp_event_base_t event_base,
                  int32_t event_id,
                  void *event_data)
{

  switch (event_id)
  {
  case WIFI_EVENT_AP_STACONNECTED:
  {
    rest.start_http_server();
  }
  break;
  case WIFI_EVENT_AP_STADISCONNECTED:
  {
    rest.stop_http_server();
  }
  }
}

void app_main(void)
{

  TaskHandle_t main_handle = nullptr;
  if (pdPASS == xTaskCreate(DroneHandler::xTaskStartMotors, "quad_main", 2048, nullptr, 2, &main_handle))
  {

    wifi_config_t wifi_config;

    unsigned char SSID[] = "auto_quad";
    unsigned char pass[] = "12345678";

    memcpy(wifi_config.ap.ssid, SSID, sizeof(SSID));
    memcpy(wifi_config.ap.password, pass, sizeof(pass));

    wifi_config.ap.ssid_len = strlen(reinterpret_cast<const char *>(SSID));
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_config.ap.max_connection = 2;
    ap_wifi_driver wifi_driver(wifi_config, wifi_handler);
    wifi_driver.start_acces_point();

  }
}
