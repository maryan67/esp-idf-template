
#ifndef QUAD_REST
#define QUAD_REST

#include "esp_http_server.h"
#include "GeneralErrorCodes.h"
#include "DroneHandler.h"
// QUAD HAS 3 STATES THAT CAN BE CHANGED BY REST 
/*
IDLE = 1,
CALIBRATION = 2,
FLIGHT = 3,
*/
#define URI_MAX_LENGTH 100

typedef esp_err_t (*uri_handler)(httpd_req_t *r);

class quad_rest
{
public:
    ~quad_rest(){};
    static quad_rest *get_singleton()
    {
        if (rest_obj == nullptr)
        {
            rest_obj = new quad_rest();
        }
        return rest_obj;
    }

    void start_http_server() noexcept(false);
    void stop_http_server() noexcept(false);

private:
    quad_rest()
    {
        m_esp_server_handle = nullptr;
        if (request_buffer == nullptr)
            request_buffer = reinterpret_cast<char *>(malloc(2000));
    };
    httpd_handle_t m_esp_server_handle;
    const modules_exception_codes m_module = http_module_exception;
    void load_endpoints();
    static quad_rest *rest_obj;
    static esp_err_t dummy_handler_rest(httpd_req_t *r)
    {
        return ESP_OK;
    }

    // Server api
    // for all http requests, don't crash stack, and save time
    static char *request_buffer;
    static esp_err_t handler_throttle_put(httpd_req_t *request);
     static esp_err_t handler_state_put(httpd_req_t *request);
     static esp_err_t handler_pid_put(httpd_req_t * request);

     
    const httpd_uri_t m_endpoints[8] = {{"/", HTTP_GET, dummy_handler_rest, nullptr},
                                        {"/control", HTTP_GET, dummy_handler_rest, nullptr},
                                        {"/control/state", HTTP_PUT, handler_state_put, nullptr},
                                        {"/control/acceleration", HTTP_PUT, handler_throttle_put, nullptr},
                                        {"/settings", HTTP_GET, dummy_handler_rest, nullptr},
                                        {"/settings/PID", HTTP_PUT, handler_pid_put, nullptr},
                                        {"/settings/targetangle", HTTP_GET, dummy_handler_rest, nullptr},
                                        {"/settings/calibrate", HTTP_GET, dummy_handler_rest, nullptr}};
};

#endif