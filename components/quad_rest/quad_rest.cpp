#include "quad_rest.h"
#include "cJSON.h"
#include "PID.h"
#include <ctype.h>

char *quad_rest::request_buffer = nullptr;
quad_rest *quad_rest::rest_obj = nullptr;

void quad_rest::start_http_server()
{
    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    module_exception::idf_error_check(httpd_start(&m_esp_server_handle, &server_config), m_module);
    load_endpoints();
}

void quad_rest::stop_http_server()
{
    httpd_stop(m_esp_server_handle);
    m_esp_server_handle = 0;
}

void quad_rest::load_endpoints()
{
    for (httpd_uri_t endpoint : m_endpoints)
    {
        httpd_register_uri_handler(m_esp_server_handle, &endpoint);
    }
}

esp_err_t quad_rest::handler_throttle_put(httpd_req_t *request)
{

    size_t request_size = request->content_len;

    if (httpd_req_recv(request, request_buffer, request_size) >= 0)
    {
        DroneHandler *quad_handle = DroneHandler::getSingletonInstance();
        cJSON *request_json = nullptr;
        request_json = cJSON_Parse(request_buffer);

        cJSON *throttle_percentage_ob = cJSON_GetObjectItem(request_json, "throttle");
        int new_throttle = 0;
        if (cJSON_IsNumber(throttle_percentage_ob))
        {
            new_throttle = throttle_percentage_ob->valueint;
            free(request_json);
            free(throttle_percentage_ob);
        }
        else
        {
            free(request_json);
            free(throttle_percentage_ob);
            httpd_resp_set_status(request, "400");
            httpd_resp_send(request, nullptr, 0);
            return ESP_OK;
        }

        xSemaphoreTake(quad_handle->get_loop_sema(), portMAX_DELAY);

        try
        {
            quad_handle->set_throttle(new_throttle);
            xSemaphoreGive(quad_handle->get_loop_sema());
        }
        catch (GeneralErrorCodes_te except)
        {
            xSemaphoreGive(quad_handle->get_loop_sema());
            if (except == INVALID_PARAMETERS)
            {
                httpd_resp_send_500(request);
                return ESP_OK;
            }
        }
    }

    httpd_resp_send(request, nullptr, 0);
    return ESP_OK;
}

esp_err_t quad_rest::handler_state_put(httpd_req *request)
{
    size_t request_size = request->content_len;

    if (httpd_req_recv(request, request_buffer, request_size) >= 0)
    {
        DroneHandler *quad_handle = DroneHandler::getSingletonInstance();
        cJSON *request_json = nullptr;
        request_json = cJSON_Parse(request_buffer);

        cJSON *state_ob = cJSON_GetObjectItem(request_json, "state");
        int new_state = 0;
        if (cJSON_IsNumber(state_ob))
        {
            new_state = state_ob->valueint;
            free(request_json);
            free(state_ob);
        }
        else
        {
            free(request_json);
            free(state_ob);
            httpd_resp_set_status(request, "400");
            httpd_resp_send(request, nullptr, 0);
            return ESP_OK;
        }

        xSemaphoreTake(quad_handle->get_loop_sema(), portMAX_DELAY);

        try
        {
            quad_handle->switch_state(static_cast<DroneHandlerState_te>(new_state));
            xSemaphoreGive(quad_handle->get_loop_sema());
        }
        catch (GeneralErrorCodes_te except)
        {
            xSemaphoreGive(quad_handle->get_loop_sema());
            if (except == INVALID_PARAMETERS)
            {
                httpd_resp_send_500(request);
                return ESP_OK;
            }
        }
    }

    httpd_resp_send(request, nullptr, 0);
    return ESP_OK;
}

esp_err_t quad_rest::handler_pid_put(httpd_req *request)
{
    size_t request_size = request->content_len;

    if (httpd_req_recv(request, request_buffer, request_size) >= 0)
    {
        DroneHandler *quad_handle = DroneHandler::getSingletonInstance();
        cJSON *request_json = nullptr;
        request_json = cJSON_Parse(request_buffer);

        cJSON *axis_ob = cJSON_GetObjectItem(request_json, "axis");
        pid_axes axis_to_change = PID_DEFAULT;
        if (cJSON_IsString(axis_ob) && strlen(axis_ob->valuestring)== 1)
        {
            // case insensitive
            switch (tolower(axis_ob->valuestring[0]))
            {
            case 'x':
                axis_to_change = PID_X;
                break;
            case 'y':
                axis_to_change = PID_Y;
                break;
            case 'z':
                axis_to_change = PID_Z;
                break;
            default:
                free(request_json);
                free(axis_ob);
                httpd_resp_set_status(request, "400");
                httpd_resp_send(request, nullptr, 0);
                return ESP_OK;
                break;
            }
        }
        cJSON *kp_ob = cJSON_GetObjectItem(request_json, "kp");
        double new_kp = 0U;
        if (cJSON_IsNumber(kp_ob))
        {
            new_kp = kp_ob->valuedouble;
        }

        cJSON *kd_ob = cJSON_GetObjectItem(request_json, "kd");
        double new_kd = 0U;
        free(kp_ob);
        if (cJSON_IsNumber(kd_ob))
        {
            new_kd = kd_ob->valuedouble;
        }
        free(kd_ob);
        cJSON *ki_ob = cJSON_GetObjectItem(request_json, "ki");
        double new_ki = 0U;
        if (cJSON_IsNumber(ki_ob))
        {
            new_ki = ki_ob->valuedouble;
        }
        free(ki_ob);
        free(request_json);
        free(axis_ob);
        httpd_resp_set_status(request, "400");
        httpd_resp_send(request, nullptr, 0);

        xSemaphoreTake(quad_handle->get_loop_sema(), portMAX_DELAY);

        try
        {
            quad_handle->set_pid(axis_to_change, new PID(PID_MAX_STEP,
                                                         PID_MIN_STEP, new_kp, new_kd, new_ki));
            // TODO check if 0
            xSemaphoreGive(quad_handle->get_loop_sema());
        }
        catch (GeneralErrorCodes_te except)
        {
            xSemaphoreGive(quad_handle->get_loop_sema());
            if (except == INVALID_PARAMETERS)
            {
                httpd_resp_send_500(request);
                return ESP_OK;
            }
        }
    }

    httpd_resp_send(request, nullptr, 0);
    return ESP_OK;
}

// handlers