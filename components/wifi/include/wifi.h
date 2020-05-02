#ifndef _DRONE_WIFI_H_
#define _DRONE_WIFI_H_

extern "C"
{
#include <cstring>
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

    static wifi_config_t default_quad_ap()
    {
        wifi_config_t default_config = {0};

        unsigned char SSID[] = "auto_quad";
        unsigned char pass[] = "12345678";

        memcpy(default_config.ap.ssid, SSID, sizeof(SSID));
        memcpy(default_config.ap.password, pass, sizeof(pass));

        default_config.ap.ssid_len = strlen(reinterpret_cast<const char *>(SSID));
        default_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
        default_config.ap.max_connection = 2;
        return default_config;
    }
    static void wifi_handler(void *event_handler_arg,
                             esp_event_base_t event_base,
                             int32_t event_id,
                             void *event_data);

private:
    wifi_config_t m_wifi_config;
    esp_event_handler_t m_wifi_ap_event_handler;

    esp_err_t last_error_code;
    void init_nvs();
    void init_wifi_stack();
    const modules_exception_codes module = wifi_module_exception;

    
};

#endif // _DRONE_WIFI_H