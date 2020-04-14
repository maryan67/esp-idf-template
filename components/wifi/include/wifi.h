#ifndef _DRONE_WIFI_H_
#define _DRONE_WIFI_H_

extern "C"
{
#include "esp_wifi.h"
}
#include "GeneralErrorCodes.h"

class ap_wifi_driver
{

public:
    ap_wifi_driver(wifi_config_t wifi_config, esp_event_handler_t esp_event_handler) : m_wifi_config(wifi_config), m_wifi_ap_event_handler(esp_event_handler){};
    ~ap_wifi_driver(){};

    void scan();
    void start_acces_point() noexcept(false);

private:
    wifi_config_t m_wifi_config;
    esp_event_handler_t m_wifi_ap_event_handler;

    esp_err_t last_error_code;
    void init_nvs();
    void init_wifi_stack();
    const modules_exception_codes module = wifi_module_exception;
};

#endif // _DRONE_WIFI_H