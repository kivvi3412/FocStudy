//
// Created by HAIRONG ZHU on 2026/5/31.
//

#include "motor_foc_driver.h"

#include <cmath>


#include "esp_log.h"
#include "project_conf.h"

static const char *TAG = "FocDriver";
#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

FocDriver::FocDriver(gpio_num_t u_gpio,
                     gpio_num_t v_gpio,
                     gpio_num_t w_gpio,
                     gpio_num_t en_gpio,
                     int pole_pairs) : en_gpio_(en_gpio), pole_pairs_(pole_pairs) {
    // 初始化电机驱动，使能引脚, 创建逆变器
    inverter_config_t cfg = {};
    cfg.timer_config.group_id = 0;
    cfg.timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    cfg.timer_config.resolution_hz = FOC_MCPWM_TIMER_RESOLUTION_HZ;
    cfg.timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
    cfg.timer_config.period_ticks = FOC_MCPWM_PERIOD;
    cfg.operator_config.group_id = 0;
    cfg.compare_config.flags.update_cmp_on_tez = true;
    cfg.gen_gpios[0] = u_gpio;
    cfg.gen_gpios[1] = v_gpio;
    cfg.gen_gpios[2] = w_gpio;

    ESP_ERROR_CHECK(svpwm_new_inverter(&cfg, &inverter_)); // 新建一个逆变器
    ESP_ERROR_CHECK(svpwm_inverter_start(inverter_, MCPWM_TIMER_START_NO_STOP)); // 启动逆变器
    ESP_LOGI(TAG, "Inverter init OK");


    gpio_config_t drv_en_config = {};
    drv_en_config.pin_bit_mask = 1ULL << en_gpio;
    drv_en_config.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&drv_en_config));

    // 创建FOC计算任务
    //    xTaskCreate(_foc_task_static, "foc_calc_task", 4096, this, configMAX_PRIORITIES - 1, &foc_task_handle_);
    xTaskCreatePinnedToCore(
        _foc_task_static,
        "foc_calc_task",
        4096,
        this,
        20, //configMAX_PRIORITIES - 1
        &foc_task_handle_,
        0
    );

    // 创建 foc 主循环定时器
    // 开启 menuconfig 的 CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD, .dispatch_method = ESP_TIMER_ISR,
    // 调整优先级 组件配置 → ESP Timer → Timer task priority
    esp_timer_create_args_t timer_args ={};
    timer_args.callback = &_timer_callback_static;
    timer_args.arg = this;
    timer_args.name = "foc_loop_timer";
    
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &foc_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(foc_timer, FOC_CALC_PERIOD));
}


// public
void FocDriver::driver_enable(bool enable) {
    gpio_set_level(en_gpio_, enable ? 1 : 0);
}

void FocDriver::open_loop_1_cycle(float Ud, float Uq) {
    const int steps_per_elec = 100; // 每个电周期的步数
    const int steps = steps_per_elec * pole_pairs_; // 一个机械周期总步数
    const float step_angle = 2.0f * M_PI / steps_per_elec; // 每一步的电角度增量

    for (int i = 0; i < steps; i++) {
        float angle = step_angle * (float) i;
        _set_dq_out_exec(Ud, Uq, angle);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 停止输出，三相全部设为中位（零电压矢量）
    svpwm_inverter_set_duty(inverter_, FOC_MCPWM_PERIOD / 4, FOC_MCPWM_PERIOD / 4, FOC_MCPWM_PERIOD / 4);
}

// private
float FocDriver::_normalize_angle(float angle) {
    float a = fmodf(angle, 2.0f * M_PI); //取余运算可以用于归一化，列出特殊值例子算便知
    return a >= 0 ? a : (a + 2.0f * static_cast<float>(M_PI));
}

void FocDriver::_timer_callback_static(void *args) {
    auto *self = static_cast<FocDriver *>(args);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(self->foc_task_handle_, &xHigherPriorityTaskWoken); // 通知FOC任务
    portYIELD_FROM_ISR();
}

void FocDriver::_foc_task_static(void *arg) {
    auto *self = static_cast<FocDriver *>(arg);
    self->_set_dq_out_loop();
}

void FocDriver::_set_dq_out_loop() {
    // 定时器循环用于控制电机
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

void FocDriver::_set_dq_out_exec(float Ud, float Uq, float e_theta_rad) {
    dq_out_.d = _constrain(Ud, -FOC_MCPWM_OUTPUT_LIMIT, FOC_MCPWM_OUTPUT_LIMIT); // 限制Ud的范围
    dq_out_.q = _constrain(Uq, -FOC_MCPWM_OUTPUT_LIMIT, FOC_MCPWM_OUTPUT_LIMIT); // 限制Uq的范围

    foc_inverse_park_transform(e_theta_rad, &dq_out_, &ab_out_);
    foc_svpwm_duty_calculate(&ab_out_, &uvw_out_); // SVPWM计算
    // foc_inverse_clarke_transform(&ab_out_, &uvw_out_); // 克拉克逆变换(SPWM)

    // 设置PWM
    uvw_duty_[0] = int(uvw_out_.u / 2) + FOC_MCPWM_PERIOD / 4;
    uvw_duty_[1] = int(uvw_out_.v / 2) + FOC_MCPWM_PERIOD / 4;
    uvw_duty_[2] = int(uvw_out_.w / 2) + FOC_MCPWM_PERIOD / 4;

    // 使能PWM
    ESP_ERROR_CHECK(svpwm_inverter_set_duty(inverter_, uvw_duty_[0], uvw_duty_[1], uvw_duty_[2]));
}
