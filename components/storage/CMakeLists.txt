idf_component_register(
    SRCS
        "nvs_manager.c"
        "sd_manager.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        globals
        nvs_flash
        sdmmc
        fatfs
        vfs

)