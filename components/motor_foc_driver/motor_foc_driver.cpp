//
// Created by HAIRONG ZHU on 25-1-14.
//

#include "motor_foc_driver.h"
#include "driver/gpio.h"
#include "project_conf.h"

static const char *TAG = "FocDriver";
#define _constrain(amt, low, high)                                             \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

FocDriver::FocDriver(gpio_num_t u_gpio, gpio_num_t v_gpio, gpio_num_t w_gpio,
                     gpio_num_t en_gpio, AS5600 *as5600, int pole_pairs)
    : en_gpio_(en_gpio), as5600_(as5600), pole_pairs_(pole_pairs) {
    // ====== 1. 初始化逆变器（创建但不启动）======
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

    ESP_ERROR_CHECK(svpwm_new_inverter(
        &cfg, &inverter_)); // 新建一个逆变器（此时 timer 未 enable）
    ESP_LOGI(TAG, "Inverter init OK");

    // ====== 2. 初始化使能引脚 ======
    gpio_config_t drv_en_config = {};
    drv_en_config.pin_bit_mask = 1ULL << en_gpio;
    drv_en_config.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&drv_en_config));

    // ====== 3. 创建 FOC 计算任务（先于 MCPWM
    // 启动，确保任务就绪后再接收通知）======
    xTaskCreatePinnedToCore(_foc_task_static, "foc_calc_task", 4096, this,
                            20, // configMAX_PRIORITIES - 1
                            &foc_task_handle_,
                            0 // Core 0
    );

    // ====== 4. 注册 MCPWM on_full 回调（必须在 timer enable 之前，即
    // inverter_start 之前）====== on_full: 定时器计数到顶部（UP_DOWN
    // 模式的峰值）时触发 此时 PWM 波形开始下降沿 —— 正是最佳采样/计算时刻
    mcpwm_timer_event_callbacks_t cbs = {};
    cbs.on_full = _mcpwm_on_full_cb;
    ESP_ERROR_CHECK(svpwm_inverter_register_cbs(inverter_, &cbs, this));

    // ====== 5. 启动逆变器（内部 enable + start timer，回调开始触发）======
    ESP_ERROR_CHECK(svpwm_inverter_start(inverter_, MCPWM_TIMER_START_NO_STOP));
    ESP_LOGI(TAG, "Inverter started with MCPWM sync @ %d Hz",
             (int)(FOC_MCPWM_TIMER_RESOLUTION_HZ / (2 * FOC_MCPWM_PERIOD)));
}

void FocDriver::bsp_bridge_driver_enable(bool enable) {
    foc_is_enabled_ = enable;
    ESP_LOGI(TAG, "%s MOSFET gate", foc_is_enabled_ ? "Enable" : "Disable");
    gpio_set_level((gpio_num_t) en_gpio_, foc_is_enabled_);
}

/**
 * @brief 电机自检函数：开环正转，检测编码器读数变化方向，设置零电角度
 */
void FocDriver::foc_motor_calibrate() {
    if (!foc_is_enabled_) {
        ESP_LOGW(TAG, "Please enable the motor driver first");
        return;
    }
    // 标记校准中，FOC 主循环跳过计算
    calibrating_ = true;

    // 停止 AS5600 连续读取任务，避免 I2C 总线冲突
    as5600_->stop_continuous_read();

    // 第一步: 确定电机的旋转方向
    ESP_LOGI(TAG, "Starting motor direction calibration...");
    float theta = 0;
    float delta_theta = M_TWOPI * 0.01; // 每次的角度增量
    int test_steps = 30;

    // 施加初始电压并等待稳定
    _set_dq_out_exec(0, FOC_MCPWM_CALIBRATE_VOLTAGE, 0); // 开环运行电机到0度
    vTaskDelay(pdMS_TO_TICKS(300)); // 延时300ms
    float initial_angle =
            as5600_->read_radian_from_sensor_with_no_update(); // 读取编码器角度

    for (int i = 0; i < test_steps; i++) {
        theta += delta_theta;
        _set_dq_out_exec(0, FOC_MCPWM_CALIBRATE_VOLTAGE, theta);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    float final_angle =
            as5600_->read_radian_from_sensor_with_no_update(); // 读取最终角度

    // 停止电机
    _set_dq_out_exec(0, 0, theta);
    float angle_difference = final_angle - initial_angle; // 计算角度差
    if (angle_difference < -M_PI) {
        // 角度差归一化
        angle_difference += 2 * M_PI;
    } else if (angle_difference > M_PI) {
        angle_difference -= 2 * M_PI;
    }

    // 判断电机旋转方向
    as5600_direction_ = (angle_difference > 0) ? 1.0f : -1.0f;
    ESP_LOGI(TAG, "Motor direction is %s", (as5600_direction_ > 0) ? "1" : "-1");
    // 设置零电角度
    /*
     * @brief 如果设置 Q 的话是不是代表超前 90 度就不是 0 电位角了, 所以需要设置 D
     * 轴 (犯的经典错误(整整搞了一整天!!)) 如果不进行零位校准, 就会发生什么 ?
     *        定子磁场与转子磁场不同步: 控制算法认为转子在零位置,
     * 但实际可能在其他位置。这导致定子产生的磁场与转子磁场之间存在相位差。
     *        扭矩产生效率低: 由于相位差, 产生的扭矩不是最大化的,
     * 需要更多的电流来产生相同的扭矩, 导致功率损耗增加。 运行不稳定: 可能出现震荡
     * 、 抖动 、 噪音等问题。 https://www.bilibili.com/video/BV1Bfq6YLEZ2
     */

    // 第二步: 设置零电角度
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Setting zero electrical angle...");

    // 施加D轴电压, 使电机定子磁场对准零位
    _set_dq_out_exec(FOC_MCPWM_CALIBRATE_VOLTAGE, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 读取编码器角度, 计算零电角度
    zero_electric_angle_ = as5600_->read_radian_from_sensor_with_no_update() *
                           (float) pole_pairs_ * as5600_direction_;
    vTaskDelay(pdMS_TO_TICKS(100));

    // 停止电机
    _set_dq_out_exec(0, 0, 0);
    ESP_LOGI(TAG, "Zero electrical angle is set to %.2f rad",
             zero_electric_angle_);
    ESP_LOGI(TAG, "Motor direction calibration done.");

    // 重新启动 AS5600 连续读取任务
    as5600_->start_continuous_read();

    // 解除校准标记，FOC 主循环恢复运行
    calibrating_ = false;
}

void FocDriver::set_free() { current_mode_ = Mode::None; }

void FocDriver::set_dq(float Ud, float Uq) {
    current_uq_ = Uq;
    current_ud_ = Ud;
    current_mode_ = Mode::TorqueControl;
}

void FocDriver::set_velocity(float speed_rad_s, PIDController *pid_velocity) {
    target_speed_rad_s_ = speed_rad_s;
    pid_velocity_ = pid_velocity;
    current_mode_ = Mode::VelocityControl;
}

void FocDriver::set_abs_position(float position_rad,
                                 PIDController *pid_position,
                                 PIDController *pid_position_v) {
    target_position_rad_ = position_rad;
    pid_position_ = pid_position;
    pid_position_velocity_ = pid_position_v;
    current_mode_ = Mode::AbsPositionControl;
}

void FocDriver::set_rel_position(float position_rad,
                                 PIDController *pid_position,
                                 PIDController *pid_position_v) {
    target_position_rad_ = position_rad;
    pid_position_ = pid_position;
    pid_position_velocity_ = pid_position_v;
    current_mode_ = Mode::RelPositionControl;
}

// private
float FocDriver::_normalize_angle(float angle) {
    float a =
            fmodf(angle, 2.0f * M_PI); // 取余运算可以用于归一化，列出特殊值例子算便知
    return a >= 0 ? a : (a + 2.0f * (float) M_PI);
}

/**
 * @brief 获取电角度 —— 使用缓存角度 + 速度前馈补偿
 *
 * 角度补偿原理：
 *   AS5600 的读取在 Core 1 后台进行，读到的角度有一定延迟（~100µs @400kHz）。
 *   通过 θ_compensated = θ_cached + ω × Δt 来补偿这个延迟。
 *   其中 ω 是低通滤波后的角速度，Δt 是 (当前时刻 - 角度被读取的时刻)。
 */
float FocDriver::_get_electrical_angle() {
    float cached_radian;
    int64_t cached_time_us;
    as5600_->get_cached_angle_us(&cached_radian, &cached_time_us);

    // 速度前馈补偿：用滤波速度 × 时间差 来预测当前角度
    int64_t now_us = esp_timer_get_time();
    float elapsed_s = (float) (now_us - cached_time_us) * 1e-6f;

    // 钳制补偿时间，防止异常（例如读取器刚启动时）
    if (elapsed_s < 0 || elapsed_s > 0.01f) {
        elapsed_s = 0;
    }

    // velocity_filter 是 AS5600 角度的变化率（弧度/秒），已包含方向
    // 关键修正：AS5600 内部硬件滤波器有大约 1.5ms~2.2ms 的固有延迟
    // 如果不补偿这个硬件延迟，在高速旋转时电角度会严重滞后，导致注入纯 D
    // 轴无功电流，引发 6.5W 级别的巨大发热！
    const float AS5600_INTERNAL_DELAY_S = 0.0015f;
    float total_delay_s = elapsed_s + AS5600_INTERNAL_DELAY_S;

    float compensated_radian =
            cached_radian + as5600_->get_velocity_filter() * total_delay_s;

    return compensated_radian * (float) pole_pairs_ * as5600_direction_ -
           zero_electric_angle_;
}

/**
 * @brief MCPWM 定时器到达峰值（on_full）时的 ISR 回调
 *
 * 在 UP_DOWN 计数模式下，on_full 发生在定时器计数到 period_ticks 的瞬间，
 * 此时 PWM 波形从高开始下降 —— 这是中心对齐 PWM 的对称中心，
 * 是最理想的电流采样/角度读取时刻。
 *
 * 此回调替代了原来的 esp_timer ISR，实现了 FOC 计算与 PWM 波形的精确同步。
 */
bool FocDriver::_mcpwm_on_full_cb(mcpwm_timer_handle_t timer,
                                  const mcpwm_timer_event_data_t *edata,
                                  void *user_ctx) {
    auto *self = static_cast<FocDriver *>(user_ctx);

    // 分频：每 FOC_MCPWM_SYNC_DIVIDER 个 PWM 周期才触发一次 FOC 计算
    // 防止 20kHz 通知频率饿死 IDLE 任务导致看门狗超时
    uint32_t cnt = self->mcpwm_div_counter_ + 1;
    if (cnt >= FOC_MCPWM_SYNC_DIVIDER) {
        self->mcpwm_div_counter_ = 0;
        BaseType_t high_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(self->foc_task_handle_, &high_task_woken);
        return high_task_woken; // 返回 true 时触发上下文切换
    }
    self->mcpwm_div_counter_ = cnt;
    return false;
}

void FocDriver::_foc_task_static(void *arg) {
    auto *self = static_cast<FocDriver *>(arg);
    self->_set_dq_out_loop();
}

void FocDriver::_set_dq_out_loop() {
    // MCPWM 同步循环，控制电机
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 校准期间跳过 FOC 计算（校准函数直接调用 _set_dq_out_exec）
        if (calibrating_) {
            continue;
        }

        switch (current_mode_) {
            case Mode::None:
                _set_dq_out_exec(0, 0, _get_electrical_angle());
                break;
            case Mode::TorqueControl:
                _set_dq_out_exec(current_ud_, as5600_direction_ * current_uq_,
                                 _get_electrical_angle());
                break;
            case Mode::VelocityControl: {
                float error = target_speed_rad_s_ - as5600_->get_velocity_filter();
                float Uq = as5600_direction_ * pid_velocity_->calculate(error);
                _set_dq_out_exec(0, Uq, _get_electrical_angle());
                break;
            }
            case Mode::AbsPositionControl: {
                float pos_error =
                        target_position_rad_ - as5600_->get_custom_total_radian();
                float target_speed = pid_position_velocity_->calculate(pos_error);
                float vel_error = target_speed - as5600_->get_velocity_filter();
                float Uq = as5600_direction_ * pid_position_->calculate(vel_error);
                _set_dq_out_exec(0, Uq, _get_electrical_angle());
                break;
            }
            case Mode::RelPositionControl: {
                float pos_error = std::fmod(target_position_rad_, (float) M_TWOPI) -
                                  as5600_->get_radian();
                if (pos_error > 0 && pos_error > M_PI) {
                    pos_error -= 2 * M_PI;
                } else if (pos_error < 0 && pos_error < -M_PI) {
                    pos_error += 2 * M_PI;
                }
                float target_speed = pid_position_velocity_->calculate(pos_error);
                float vel_error = target_speed - as5600_->get_velocity_filter();
                float Uq = as5600_direction_ * pid_position_->calculate(vel_error);
                _set_dq_out_exec(0, Uq, _get_electrical_angle());
                break;
            }
        }
    }
}

void FocDriver::_set_dq_out_exec(float Ud, float Uq, float e_theta_rad) {
    dq_out_.d = _constrain(Ud, -FOC_MCPWM_OUTPUT_LIMIT,
                           FOC_MCPWM_OUTPUT_LIMIT); // 限制Ud的范围
    dq_out_.q = _constrain(Uq, -FOC_MCPWM_OUTPUT_LIMIT,
                           FOC_MCPWM_OUTPUT_LIMIT); // 限制Uq的范围

    foc_inverse_park_transform(e_theta_rad, &dq_out_, &ab_out_);
    foc_svpwm_duty_calculate(&ab_out_, &uvw_out_); // SVPWM计算
    // foc_inverse_clarke_transform(&ab_out_, &uvw_out_); // 克拉克逆变换(SPWM)

    // 设置PWM
    uvw_duty_[0] = int(uvw_out_.u / 2) + FOC_MCPWM_PERIOD / 4;
    uvw_duty_[1] = int(uvw_out_.v / 2) + FOC_MCPWM_PERIOD / 4;
    uvw_duty_[2] = int(uvw_out_.w / 2) + FOC_MCPWM_PERIOD / 4;

    // 使能PWM
    ESP_ERROR_CHECK(svpwm_inverter_set_duty(inverter_, uvw_duty_[0], uvw_duty_[1],
        uvw_duty_[2]));
}
