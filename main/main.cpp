#include <string>
#include "sdkconfig.h"
#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C"
{
  void app_main (void);
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "nvs.h"
#include "nvs_flash.h"
}

class Greet
{

  public:

    Greet (gpio_num_t PinNumber_e)
    {
      gpio_set_direction (PinNumber_e, GPIO_MODE_OUTPUT);
    }

    esp_err_t BlinkOnPin (gpio_num_t PinNumber_e)
    {
      return gpio_set_level (PinNumber_e, 1);
    }

  private:

};

void app_main (void)
{

  ledc_timer_config_t Config_st;
  Config_st.freq_hz = 500;
  Config_st.duty_resolution = LEDC_TIMER_10_BIT;
  Config_st.speed_mode = LEDC_HIGH_SPEED_MODE;
  Config_st.timer_num = LEDC_TIMER_0;
  ledc_timer_config (&Config_st);
  ledc_channel_config_t ChannelConfig_st;
  ChannelConfig_st.channel = LEDC_CHANNEL_0;
  ChannelConfig_st.duty = 50;
  ChannelConfig_st.gpio_num = 16;
  ChannelConfig_st.intr_type = LEDC_INTR_DISABLE;
  ChannelConfig_st.speed_mode = LEDC_HIGH_SPEED_MODE;
  ChannelConfig_st.timer_sel = LEDC_TIMER_0;
  ledc_channel_config (&ChannelConfig_st);


  nvs_close (NvsHandler_pu8);

//    while (1 == 1) {
//        for (int i = 0; i < 1023; i++) {
//            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, i);
//
//            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
//            vTaskDelay(portTICK_PERIOD_MS);
//        }
//        for (int i = 1023; i > 0; i--) {
//            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, i);
//
//            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
//            vTaskDelay(portTICK_PERIOD_MS);
//        }
//    }
}

