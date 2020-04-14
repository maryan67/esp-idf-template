
#ifndef _GENERAL_ERROR_CODES_
#define _GENERAL_ERROR_CODES_

typedef enum GeneralErrorCodes
{
    NO_ERROR = 0,
    NO_CONFIGURATION_FOUND,
    INVALID_PARAMETERS,
    HAL_ERROR,
    MODULE_NOT_INITIALISED,
    EMERGENCY_MODE_NOT_SET

} GeneralErrorCodes_te;

typedef enum exception_codes
{
    wifi_module_exception = 0,
    http_module_exception = 1,
    
    module_max
} modules_exception_codes;



class module_exception
{
public:
    module_exception(modules_exception_codes module, esp_err_t last_error_code) : m_module(module), m_last_error_code(last_error_code)
    {
    }
    ~module_exception(){};

    modules_exception_codes get_module()
    {
        return m_module;
    }

    esp_err_t get_last_error_code()
    {
        return m_last_error_code;
    }
    static void idf_error_check(esp_err_t error_code, modules_exception_codes module) noexcept(false)
    {
        if (error_code != ESP_OK)
        {
            throw module_exception(module, error_code);
        }
    }

private:
    modules_exception_codes m_module;
    esp_err_t m_last_error_code;
};

#endif //_GENERAL_ERROR_CODES_