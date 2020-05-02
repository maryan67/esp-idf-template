#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "wifi.h"
#include "quad_rest.h"

void ap_wifi_driver::init_nvs()
{
    esp_err_t return_error = nvs_flash_init();

    if (return_error != ESP_OK)
    {
        if (return_error == ESP_ERR_NVS_NO_FREE_PAGES ||
            return_error == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            return_error = nvs_flash_erase(); // maybe
            if (return_error == ESP_OK)
                module_exception::idf_error_check(nvs_flash_init(), module);
        }
        else
            throw module_exception(module, return_error);
    }
}

void ap_wifi_driver::init_wifi_stack()
{
    tcpip_adapter_init();
    module_exception::idf_error_check(esp_event_loop_create_default(), module);
    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT(); // default config

    module_exception::idf_error_check(esp_wifi_init(&wifi_config), module);
    module_exception::idf_error_check(
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, m_wifi_ap_event_handler, nullptr), module);
    module_exception::idf_error_check(esp_wifi_set_mode(WIFI_MODE_AP), module);
    module_exception::idf_error_check(esp_wifi_set_config(ESP_IF_WIFI_AP, &m_wifi_config), module);
    module_exception::idf_error_check(esp_wifi_start(), module);
}

void ap_wifi_driver::start_acces_point()
{
    init_nvs();
    init_wifi_stack();
}

void ap_wifi_driver::wifi_handler(void *event_handler_arg,
                                  esp_event_base_t event_base,
                                  int32_t event_id,
                                  void *event_data)
{
    quad_rest * rest_server = quad_rest::get_singleton();
    switch (event_id)
    {
    case WIFI_EVENT_AP_STACONNECTED:
    {
        rest_server->start_http_server();
    }
    break;
    case WIFI_EVENT_AP_STADISCONNECTED:
    {
        rest_server->stop_http_server();
    }
    }
}