idf_component_register(SRCS "iic_master.cpp"
        INCLUDE_DIRS "include"
        REQUIRES "driver" "project_conf" "esp_driver_i2c"
)
