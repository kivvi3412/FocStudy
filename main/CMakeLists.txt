idf_component_register(SRCS "app_main.cpp"
        INCLUDE_DIRS ""
        REQUIRES "motor_foc_driver" "iic_master" "iic_as5600" "project_conf" "debug_console"
)
