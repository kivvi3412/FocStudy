idf_component_register(SRCS "iic_as5600.cpp"
        INCLUDE_DIRS "include"
        REQUIRES "driver" "iic_master" "esp_driver_i2c"
)
