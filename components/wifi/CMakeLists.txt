idf_component_register(
    SRCS "wifi.cpp"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash 
    REQUIRES GeneralErrorCodes
    REQUIRES quad_rest
)
