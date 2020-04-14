idf_component_register(
    SRCS "DroneHandler.cpp"
    INCLUDE_DIRS "include"
    REQUIRES motor 
    REQUIRES MPUModule
    REQUIRES PID
)
