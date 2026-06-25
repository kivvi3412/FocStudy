//
// Created by HAIRONG ZHU on 25-1-14.
//

#include <cmath>
#include "iic_as5600.h"
#include "project_conf.h"

static const char *TAG = "AS5600";

AS5600::AS5600(i2c_master_bus_handle_t bus_handle, uint8_t device_address) {
    if (bus_handle == nullptr) {
        ESP_LOGE(TAG, "I2C master bus not initialized");
        return;
    }
    // 配置 I2C 设备
    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = device_address;
    dev_config.scl_speed_hz = IIC_MASTER_FREQ_HZ;
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
    }
}

uint16_t AS5600::_location_read_raw() {
    uint8_t reg = IIC_AS5600_RAW_ANGLE_REG;
    uint8_t buffer[2] = {0};

    esp_err_t ret = i2c_master_transmit_receive(
            dev_handle_,
            &reg,
            1,
            buffer,
            2,
            -1
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C transmit/receive failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ((uint16_t) buffer[0] << 8) | buffer[1];
}

float AS5600::read_radian_from_sensor() {
    auto current_radian = float(_location_read_raw() * M_TWOPI / IIC_AS5600_RESOLUTION);    // 读取传感器的弧度
    _update_total_radian_and_velocity(current_radian);   // 更新累计的总弧度和转速
    return current_radian;
}

float AS5600::read_radian_from_sensor_with_no_update() {
    auto current_radian = float(_location_read_raw() * M_TWOPI / IIC_AS5600_RESOLUTION);
    return current_radian;
}

esp_err_t AS5600::_update_total_radian_and_velocity(float currentRadian) {
    float deltaRadian = currentRadian - previous_radian_;
    if (fabsf(deltaRadian) > M_PI) {
        if (deltaRadian > 0) {
            deltaRadian -= M_TWOPI;
        } else {
            deltaRadian += M_TWOPI;
        }
    }
    total_accumulated_radian_ += deltaRadian;
    previous_radian_ = currentRadian;

    // 更新速度
    float Ts = FOC_CALC_PERIOD * 1e-6f; // 单位: 秒
    velocity_ = deltaRadian / Ts;

    // 低通滤波
    float alpha = FOC_LOW_PASS_FILTER_ALPHA;
    velocity_filter_ = alpha * velocity_ + (1 - alpha) * velocity_filter_;   // 一阶低通滤波

    return ESP_OK;
}

void AS5600::set_custom_total_radian(float radian) {    // 设置相对的累计弧度，将当前总累计弧度作为新的偏移 (用户自定义
    relative_offset_radian_ = total_accumulated_radian_ - radian;
}

void AS5600::reset_custom_total_radian() {    // 重置相对的累计弧度，将当前总累计弧度作为新的偏移 (用户自定义
    relative_offset_radian_ = total_accumulated_radian_;
}

float AS5600::get_radian() const {
    return previous_radian_;
}

float AS5600::get_total_radian() const {
    return total_accumulated_radian_;
}

float AS5600::get_custom_total_radian() const {
    return total_accumulated_radian_ - relative_offset_radian_;
}

float AS5600::get_velocity() const {
    return velocity_;
}

float AS5600::get_velocity_filter() const {
    return velocity_filter_;
}




