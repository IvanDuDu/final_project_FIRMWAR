idf_component_register(
    SRCS
        "ble_manager.c"
        "ble_handler.c"
        "ble_notify.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        globals
        bt
        nvs_flash
        json
        storage
        network
        actuators
        esp_wifi
        local_network
        
        
)