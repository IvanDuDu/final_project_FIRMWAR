idf_component_register(
    SRCS
        "actuators.c"
    INCLUDE_DIRS
        "."
        
    REQUIRES
        globals
        driver
        freertos
)