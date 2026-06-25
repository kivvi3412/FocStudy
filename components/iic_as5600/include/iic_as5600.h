//
// Created by HAIRONG ZHU on 25-1-14.
//

#ifndef FOCKNOB_IIC_AS5600_H
#define FOCKNOB_IIC_AS5600_H

#include "iic_master.h"
#include "esp_err.h"

class AS5600 {
public:
    explicit AS5600(i2c_master_bus_handle_t bus_handle, uint8_t device_address);

    [[nodiscard]] float read_radian_from_sensor();    // 从传感器读取弧度(并更新累计的总弧度和转速)

    [[nodiscard]] float read_radian_from_sensor_with_no_update();  // 获取当前弧度(不做更新)

    [[nodiscard]] float get_radian() const;  // 获取当前弧度

    [[nodiscard]] float get_total_radian() const; // 获取累计的总角度

    [[nodiscard]] float get_velocity() const;  // 获取当前转速

    [[nodiscard]] float get_velocity_filter() const;  // 获取低通滤波后的转速

    [[nodiscard]] float get_custom_total_radian() const; // 获取相对于重置时的累计总角度(自定义角度)

    void set_custom_total_radian(float radian); // 设置相对于重置时的累计总角度(自定义角度)

    void reset_custom_total_radian(); // 重置累计总自定义角度


private:
    i2c_master_dev_handle_t dev_handle_{};  // I2C设备句柄

    float previous_radian_{};   // 上一次读取的角度
    float total_accumulated_radian_{};  // 累计的总角度(从开机开始)
    float relative_offset_radian_{};    // 重置时的累计弧度偏移

    float velocity_{};   // 转速 (弧度/秒)
    float velocity_filter_{}; // 转速低通滤波

    uint16_t _location_read_raw();

    esp_err_t _update_total_radian_and_velocity(float currentRadian);    // 更新累计的总弧度
};


#endif //FOCKNOB_IIC_AS5600_H
