//
// Created by HAIRONG ZHU on 25-1-14.
//

#ifndef FOCKNOB_IIC_AS5600_H
#define FOCKNOB_IIC_AS5600_H

#include "iic_master.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

    // ===================== 新增：连续读取 + 缓存接口 =====================

    /**
     * @brief 启动后台连续读取任务（运行在 Core 1）
     *        该任务以最快速度循环读取 AS5600 角度，将结果缓存到 cached_radian_
     *        FOC 任务可以通过 get_cached_angle_us() 零延迟获取角度
     */
    void start_continuous_read();

    /**
     * @brief 停止后台连续读取任务（用于校准期间避免 I2C 冲突）
     */
    void stop_continuous_read();

    /**
     * @brief 获取缓存的角度值和对应的时间戳（线程安全，零延迟）
     *
     * @param[out] radian_out        缓存的机械角度（弧度）
     * @param[out] timestamp_us_out  该角度被读取时的时间戳（esp_timer_get_time, µs）
     */
    void get_cached_angle_us(float *radian_out, int64_t *timestamp_us_out);


private:
    i2c_master_dev_handle_t dev_handle_{};  // I2C设备句柄

    float previous_radian_{};   // 上一次读取的角度
    float total_accumulated_radian_{};  // 累计的总角度(从开机开始)
    float relative_offset_radian_{};    // 重置时的累计弧度偏移

    float velocity_{};   // 转速 (弧度/秒)
    float velocity_filter_{}; // 转速低通滤波

    uint16_t _location_read_raw();

    esp_err_t _update_total_radian_and_velocity(float currentRadian, int64_t now_us);    // 更新累计的总弧度

    // ===================== 连续读取相关私有成员 =====================
    portMUX_TYPE spinlock_ = portMUX_INITIALIZER_UNLOCKED;   // 跨核自旋锁
    volatile float cached_radian_{};            // 缓存的最新角度（由 Core 1 写入）
    volatile int64_t cached_timestamp_us_{};    // 缓存角度的读取时间戳
    int64_t prev_read_time_us_{};               // 上一次读取的时间戳（用于速度计算）
    TaskHandle_t reader_task_handle_{};          // 连续读取任务句柄
    volatile bool reader_running_{};            // 读取任务运行标志

    static void _continuous_read_task_static(void *arg);
    void _continuous_read_task();
};


#endif //FOCKNOB_IIC_AS5600_H
