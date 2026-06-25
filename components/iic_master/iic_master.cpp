//
// Created by HAIRONG ZHU on 25-1-14.
//

#include "iic_master.h"

static const char *TAG = "IICMaster";

IICMaster::IICMaster(i2c_port_num_t i2c_port_num, gpio_num_t sda_io_num, gpio_num_t scl_io_num) {
    ESP_ERROR_CHECK(IICMaster::iic_master_init(i2c_port_num, sda_io_num, scl_io_num));
}

esp_err_t IICMaster::iic_master_init(i2c_port_num_t i2c_port_num, gpio_num_t sda_io_num, gpio_num_t scl_io_num) {
    if (iic_bus_handle != nullptr) {
        ESP_LOGW(TAG, "I2C master already initialized");
        return ESP_OK;
    }

    // IIC bus configuration
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = i2c_port_num;
    bus_config.sda_io_num = sda_io_num;
    bus_config.scl_io_num = scl_io_num;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t ret = i2c_new_master_bus(&bus_config, &iic_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create IIC master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "IIC master initialized successfully");
    return ESP_OK;
}

i2c_master_bus_handle_t IICMaster::iic_master_get_bus_handle() {
    return iic_bus_handle;
}
