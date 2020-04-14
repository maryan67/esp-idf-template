
#ifndef QUAD_REST
#define QUAD_REST

#include "esp_http_server.h"
#include "GeneralErrorCodes.h"
#include "DroneHandler.h"

#define URI_MAX_LENGTH 100

typedef esp_err_t (*uri_handler)(httpd_req_t *r);

class quad_rest
{
public:
    quad_rest()
    {
        m_esp_server_handle = nullptr;
        // quad_main = DroneHandler::getSingletonInstance();
    };
    ~quad_rest(){};

    void start_http_server() noexcept(false);
    void stop_http_server() noexcept(false);

    static esp_err_t handler_throttle_put(httpd_req_t *request)
    {
        char content[100];
        size_t request_size = request->content_len;
        
        if (httpd_req_recv(request, content, request_size) >= 0)
        {
            DroneHandler *quad_handle = DroneHandler::getSingletonInstance();
            int new_throttle = atoi(content);
            printf("%d\n", new_throttle);
            xSemaphoreTake(quad_handle->get_loop_sema(), portMAX_DELAY);

            quad_handle->set_throttle(new_throttle);

            xSemaphoreGive(quad_handle->get_loop_sema());
        }

        return ESP_OK;
    }

private:
    httpd_handle_t m_esp_server_handle;
    const modules_exception_codes m_module = http_module_exception;
    void load_endpoints();

    static esp_err_t dummy_handler_rest(httpd_req_t *r)
    {
        return ESP_OK;
    }

    // Server api

    httpd_uri_t m_endpoints[8] = {{"/", HTTP_GET, dummy_handler_rest, nullptr},
                                  {"/control", HTTP_GET, dummy_handler_rest, nullptr},
                                  {"/control/state", HTTP_GET, dummy_handler_rest, nullptr},
                                  {"/control/acceleration", HTTP_PUT, quad_rest::handler_throttle_put, nullptr},
                                  {"/settings", HTTP_GET, dummy_handler_rest, nullptr},
                                  {"/settings/PID", HTTP_GET, dummy_handler_rest, nullptr},
                                  {"/settings/targetangle", HTTP_GET, dummy_handler_rest, nullptr},
                                  {"/settings/calibrate", HTTP_GET, dummy_handler_rest, nullptr}};
};

#endif