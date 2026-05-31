//
// Created by HAIRONG ZHU on 2026/5/31.
//

#ifndef FOCCOMPRESSOR_MOTOR_FOC_DRIVER_H
#define FOCCOMPRESSOR_MOTOR_FOC_DRIVER_H

#include "driver/gpio.h"
#include "esp_svpwm.h"
#include <esp_timer.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_foc.h"


class FocDriver {
public:
    FocDriver(gpio_num_t u_gpio,
          gpio_num_t v_gpio,
          gpio_num_t w_gpio,
          gpio_num_t en_gpio,
          int pole_pairs
    );
    
    void driver_enable(bool enable);
    
    void open_loop_1_cycle(float Ud, float Uq); 
    
private:
    gpio_num_t en_gpio_{};
    int pole_pairs_ = 0;

    inverter_handle_t inverter_{};
    esp_timer_handle_t foc_timer{};
    TaskHandle_t foc_task_handle_; 
    foc_dq_coord_t dq_out_{};   // 最大值为FOC_MCPWM_PERIOD / 2
    foc_ab_coord_t ab_out_{};
    foc_uvw_coord_t uvw_out_{};
    int uvw_duty_[3]{};  // 电机PWM占空比
    
    static float _normalize_angle(float angle);   // 角度归一化
    
    static void _timer_callback_static(void *args);   // 定时器回调函数
    static void _foc_task_static(void *arg);
    void _set_dq_out_loop();   // 设置DQ坐标 (力矩控制) 循环
    void _set_dq_out_exec(float Ud, float Uq, float e_theta_rad); 
};



#endif //FOCCOMPRESSOR_MOTOR_FOC_DRIVER_H
