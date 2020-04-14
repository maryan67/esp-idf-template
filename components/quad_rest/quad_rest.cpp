#include "quad_rest.h"

void quad_rest::start_http_server()
{
    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    module_exception::idf_error_check(httpd_start(&m_esp_server_handle, &server_config), m_module);
    load_endpoints();
}

void quad_rest::stop_http_server()
{
    httpd_stop(m_esp_server_handle);
}

void quad_rest::load_endpoints()
{
    for (httpd_uri_t endpoint : m_endpoints)
    {
        httpd_register_uri_handler(m_esp_server_handle,&endpoint);
    }
}


// handlers