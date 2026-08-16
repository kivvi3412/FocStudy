idf_component_register(SRCS "foc_driver.cpp"
        INCLUDE_DIRS "include"
        REQUIRES "mt6835_driver" "project_conf" "esp_driver_mcpwm" "esp_driver_gpio" "esp_timer" "fast_trig"
)
