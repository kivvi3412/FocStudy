idf_component_register(SRCS "mt6835_driver.cpp"
        INCLUDE_DIRS "include"
        REQUIRES "esp_driver_spi" "esp_driver_gpio" "project_conf" "esp_timer"
)
