file(GLOB COMPONENT_SRCS "*.cpp")

idf_component_register(SRCS ${COMPONENT_SRCS}
        INCLUDE_DIRS "include"
        REQUIRES "driver" "iic_as5600" "esp_timer"
)
