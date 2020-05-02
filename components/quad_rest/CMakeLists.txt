idf_component_register(
    SRCS "quad_rest.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server
    REQUIRES GeneralErrorCodes
    REQUIRES Drone_Handler
    REQUIRES json  
)
