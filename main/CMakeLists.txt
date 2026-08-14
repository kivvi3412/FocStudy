idf_component_register(SRCS "app_main.cpp"
        INCLUDE_DIRS ""
        REQUIRES "esp_driver_spi" "esp_driver_gpio" "esp_timer" "freertos"
)
