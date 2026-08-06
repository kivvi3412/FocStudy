idf_component_register(SRCS "app_main.cpp"
        INCLUDE_DIRS ""
        REQUIRES "foc_driver" "spi_mt6816" "project_conf" "debug_console" "esp_driver_mcpwm" "esp_driver_gpio" "esp_timer"
)
