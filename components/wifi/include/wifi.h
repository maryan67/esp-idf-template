#ifndef _DRONE_WIFI_H_
#define _DRONE_WIFI_H_


extern "C"
{
#include "esp_wifi.h"
}

class WiFi_Module
{

    public: 

    WiFi_Module();
    ~WiFi_Module();
    
    // initialize the module
    init();

    //scan for avaible ap's in the range
    void scan();
    void connect(wifi_config_t * ApToConnect) noexcept(false);
    
    // get signal strength of a wifi ap
    uint8_t GetSignalStrength(wifi_config_t * ApToCheck) noexcept(false);



    private:

    //tbd


};

#endif // _DRONE_WIFI_H