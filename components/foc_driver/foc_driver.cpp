//
// 封装的 FOC 电机驱动器实现
//

#include "foc_driver.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "project_conf.h"
#include <cmath>
#include "fast_trig.h"

static const char *TAG = "FocMotor";

// 纯 float 常量，避免任何隐式 double 提升
static constexpr float PI_F = 3.14159265358979323846f;
static constexpr float TWOPI_F = 2.0f * PI_F;

#define SQRT3 1.7320508075688772935f
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

FocMotor::FocMotor(MT6835 *encoder, int pole_pairs, int u_gpio, int v_gpio, int w_gpio, int en_gpio)
    : encoder_(encoder), pole_pairs_(pole_pairs), en_gpio_(en_gpio) {
    // 初始化使能引脚
    gpio_config_t en_cfg = {};
    en_cfg.pin_bit_mask = 1ULL << en_gpio_;
    en_cfg.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&en_cfg));
    gpio_set_level((gpio_num_t) en_gpio_, 0); // 默认关闭

    // 注册编码器故障回调：连续读取失败 ≥20 次时由编码器直接触发紧急停机
    encoder_->register_fault_callback(encoder_fault_handler, this);

    // FOC 计算放在高优先级任务中，MCPWM ISR 只负责发通知
    // 与 MCPWM ISR 同核 (Core 1)，减少跨核唤醒延迟
    xTaskCreatePinnedToCore(foc_task, "foc_task", 4096, this, 20, &foc_task_handle_, 1);

    init_mcpwm(u_gpio, v_gpio, w_gpio);
    ESP_LOGI(TAG, "FocMotor initialized. MCPWM %d Hz", FOC_MCPWM_TIMER_RESOLUTION_HZ / FOC_MCPWM_PERIOD);
}

void FocMotor::init_mcpwm(int u, int v, int w) {
    mcpwm_timer_config_t timer_cfg = {};
    timer_cfg.group_id = 0;
    timer_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_cfg.resolution_hz = FOC_MCPWM_TIMER_RESOLUTION_HZ;
    timer_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN; // 中央对齐模式
    timer_cfg.period_ticks = FOC_MCPWM_PERIOD;

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer_));

    mcpwm_oper_handle_t operators[3] = {};
    mcpwm_operator_config_t oper_cfg = {};
    oper_cfg.group_id = 0;

    mcpwm_comparator_config_t cmp_cfg = {};
    cmp_cfg.flags.update_cmp_on_tez = true; // 在计数值为 0 时更新占空比 (即谷底)

    int gpios[3] = {u, v, w};

    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &operators[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer_));

        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &cmp_cfg, &comparators_[i]));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators_[i], 0));

        mcpwm_generator_config_t gen_cfg = {};
        gen_cfg.gen_gpio_num = gpios[i];

        mcpwm_gen_handle_t generator = nullptr;
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &gen_cfg, &generator));

        ESP_ERROR_CHECK(
            mcpwm_generator_set_action_on_compare_event(generator,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,comparators_[i], MCPWM_GEN_ACTION_LOW)
            )
        );

        ESP_ERROR_CHECK(
            mcpwm_generator_set_action_on_compare_event(generator,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN,comparators_[i],MCPWM_GEN_ACTION_HIGH)
            )
        );
    }

    // 注册 MCPWM on_empty 回调 (必须在 enable 之前)
    mcpwm_timer_event_callbacks_t cbs = {};
    cbs.on_full = mcpwm_on_full_cb;
    ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timer_, &cbs, this));

    // 启动 MCPWM 定时器
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer_));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP));
}

void FocMotor::park_inverse(float theta, float d, float q, float *alpha, float *beta) {
    float sin_val = fast_sinf(theta);
    float cos_val = fast_cosf(theta);
    *alpha = d * cos_val - q * sin_val;
    *beta = d * sin_val + q * cos_val;

    // *alpha = d * cosf(theta) - q * sinf(theta);
    // *beta  = d * sinf(theta) + q * cosf(theta);
}

void FocMotor::svpwm_calculate(float alpha, float beta, float *u, float *v, float *w) {
    int sextant;
    if (beta > 0.0f) {
        if (alpha > 0.0f) {
            sextant = (beta > (alpha * SQRT3)) ? 2 : 1;
        } else {
            sextant = (-beta > (alpha * SQRT3)) ? 3 : 2;
        }
    } else {
        if (alpha > 0.0f) {
            sextant = (-beta > (alpha * SQRT3)) ? 5 : 6;
        } else {
            sextant = (beta > (alpha * SQRT3)) ? 4 : 5;
        }
    }

    switch (sextant) {
        case 1: {
            float t1 = (-alpha * SQRT3) + beta;
            float t2 = -2.0f * beta;
            *u = (1.0f - t1 - t2) / 2.0f;
            *v = *u + t1;
            *w = *v + t2;
            break;
        }
        case 2: {
            float t2 = (-alpha * SQRT3) - beta;
            float t3 = (alpha * SQRT3) - beta;
            *v = (1.0f - t2 - t3) / 2.0f;
            *u = *v + t3;
            *w = *u + t2;
            break;
        }
        case 3: {
            float t3 = -2.0f * beta;
            float t4 = (alpha * SQRT3) + beta;
            *v = (1.0f - t3 - t4) / 2.0f;
            *w = *v + t3;
            *u = *w + t4;
            break;
        }
        case 4: {
            float t4 = (alpha * SQRT3) - beta;
            float t5 = 2.0f * beta;
            *w = (1.0f - t4 - t5) / 2.0f;
            *v = *w + t5;
            *u = *v + t4;
            break;
        }
        case 5: {
            float t5 = (alpha * SQRT3) + beta;
            float t6 = (-alpha * SQRT3) + beta;
            *w = (1.0f - t5 - t6) / 2.0f;
            *u = *w + t5;
            *v = *u + t6;
            break;
        }
        case 6: {
            float t6 = 2.0f * beta;
            float t1 = (-alpha * SQRT3) - beta;
            *u = (1.0f - t6 - t1) / 2.0f;
            *w = *u + t1;
            *v = *w + t6;
            break;
        }
        default:
            *u = 0.0f;
            *v = 0.0f;
            *w = 0.0f;
            break;
    }
}

void FocMotor::set_pwm_duties(float su, float sv, float sw) {
    int du = (int) (su / 2.0f) + FOC_MCPWM_PERIOD / 4;
    int dv = (int) (sv / 2.0f) + FOC_MCPWM_PERIOD / 4;
    int dw = (int) (sw / 2.0f) + FOC_MCPWM_PERIOD / 4;

    du = _constrain(du, 0, FOC_MCPWM_PERIOD / 2);
    dv = _constrain(dv, 0, FOC_MCPWM_PERIOD / 2);
    dw = _constrain(dw, 0, FOC_MCPWM_PERIOD / 2);

    mcpwm_comparator_set_compare_value(comparators_[0], du);
    mcpwm_comparator_set_compare_value(comparators_[1], dv);
    mcpwm_comparator_set_compare_value(comparators_[2], dw);
}

void FocMotor::set_dq_voltage(float ud, float uq, float e_theta) {
    ud = _constrain(ud, -(float)FOC_MCPWM_OUTPUT_LIMIT, (float)FOC_MCPWM_OUTPUT_LIMIT);
    uq = _constrain(uq, -(float)FOC_MCPWM_OUTPUT_LIMIT, (float)FOC_MCPWM_OUTPUT_LIMIT);

    float alpha, beta;
    park_inverse(e_theta, ud, uq, &alpha, &beta);

    float su, sv, sw;
    svpwm_calculate(alpha, beta, &su, &sv, &sw);

    set_pwm_duties(su, sv, sw);
}

bool FocMotor::mcpwm_on_full_cb(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx) {
    const auto *self = static_cast<FocMotor *>(user_ctx); // ISR 只做触发/通知
    BaseType_t high_task_woken = pdFALSE;
    if (self->foc_task_handle_) {
        vTaskNotifyGiveFromISR(self->foc_task_handle_, &high_task_woken);
    }
    return high_task_woken;
}

void FocMotor::foc_task(void *arg) {
    auto *self = static_cast<FocMotor *>(arg);
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!self->enabled_ || self->calibrating_) {
            continue;
        }

        float e_theta = self->encoder_->read_electrical_angle_compensated(
            self->pole_pairs_, self->direction_, self->zero_electric_angle_
        );

        if (std::isnan(e_theta)) {
            continue; // 读取失败，不更新 PWM，沿用之前的输出
        }

        float uq = self->direction_ * self->uq_;
        MusicSampleFn music_fn = self->music_fn_;
        if (music_fn) {
            uq += music_fn();
        }

        self->set_dq_voltage(self->ud_, uq, e_theta);
    }
}

/// 编码器故障回调（由 MT6835 在连续读取失败时调用）
void FocMotor::encoder_fault_handler(void *ctx) {
    auto *self = static_cast<FocMotor *>(ctx);
    gpio_set_level((gpio_num_t) self->en_gpio_, 0);
    self->enabled_ = false;
    self->set_dq_voltage(0, 0, 0);
}

void FocMotor::set_voltage(float ud, float uq) {
    ud_ = ud;
    uq_ = uq;
}

void FocMotor::set_music_source(MusicSampleFn fn) {
    music_fn_ = fn;
}

float FocMotor::measure_speed_diff(float test_uq) {
    constexpr int N_SAMPLES = 16;

    // 正转测试: 加速到稳态后多次采样平均，抑制负载波动
    set_voltage(0, test_uq);
    vTaskDelay(pdMS_TO_TICKS(1200));
    float fwd = 0.0f;
    for (int i = 0; i < N_SAMPLES; i++) {
        fwd += std::fabs(encoder_->get_velocity_filtered());
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    fwd /= (float) N_SAMPLES;

    // 反转测试
    set_voltage(0, -test_uq);
    vTaskDelay(pdMS_TO_TICKS(1200));
    float rev = 0.0f;
    for (int i = 0; i < N_SAMPLES; i++) {
        rev += std::fabs(encoder_->get_velocity_filtered());
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    rev /= (float) N_SAMPLES;

    set_voltage(0, 0);
    ESP_LOGI(TAG, "Speed test: fwd=%.2f rev/s, rev=%.2f rev/s", fwd, rev);
    return fwd - rev;
}

void FocMotor::set_calibration_params(float direction, float zero_electric_angle) {
    direction_ = direction;
    zero_electric_angle_ = zero_electric_angle;
    ESP_LOGI(TAG, "Manual Calibration Params: Dir=%.1f, ZeroAngle=%.4f rad", direction_, zero_electric_angle_);
}

void FocMotor::enable() {
    gpio_set_level((gpio_num_t) en_gpio_, 1);
    enabled_ = true;
    ESP_LOGI(TAG, "Motor driver enabled.");
}

void FocMotor::disable() {
    gpio_set_level((gpio_num_t) en_gpio_, 0);
    enabled_ = false;
    set_dq_voltage(0, 0, 0);
    ESP_LOGI(TAG, "Motor driver disabled.");
}

void FocMotor::calibrate() {
    ESP_LOGI(TAG, "=== Motor Calibration Start ===");
    calibrating_ = true;
    enabled_ = true;
    gpio_set_level((gpio_num_t) en_gpio_, 1);

    run_direction_detection();
    run_zero_calibration();

    ESP_LOGI(TAG, "=== Calibration Complete ===");
    calibrating_ = false;
}

void FocMotor::run_direction_detection() {
    ESP_LOGI(TAG, "Step 1: Detecting motor direction...");
    float theta = 0;
    float delta_theta = TWOPI_F * 0.01f;
    int test_steps = 30;

    set_dq_voltage(0, FOC_MCPWM_CALIBRATE_VOLTAGE, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    float initial_angle = encoder_->read_angle_no_update();

    for (int i = 0; i < test_steps; i++) {
        theta += delta_theta;
        set_dq_voltage(0, FOC_MCPWM_CALIBRATE_VOLTAGE, theta);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    float final_angle = encoder_->read_angle_no_update();
    set_dq_voltage(0, 0, theta);

    float angle_diff = final_angle - initial_angle;
    if (angle_diff < -(float) PI_F)
        angle_diff += TWOPI_F;
    else if (angle_diff > (float) PI_F)
        angle_diff -= TWOPI_F;

    direction_ = (angle_diff > 0) ? 1.0f : -1.0f;
    ESP_LOGI(TAG, "Motor direction: %.1f", direction_);
}

void FocMotor::run_zero_calibration() {
    // ---- Step 2: 零电角度（双向逼近，抵消齿槽/摩擦死区） ----
    ESP_LOGI(TAG, "Step 2: Zero electrical angle (bidirectional alignment)...");
    vTaskDelay(pdMS_TO_TICKS(500));

    // 正向锁定: 转子 d 轴对齐 α 轴（0° 电角度）
    set_dq_voltage(FOC_MCPWM_CALIBRATE_VOLTAGE, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(3000)); // 锁定 3 秒，确保转子稳定后再采样
    float theta1 = encoder_->read_angle21_30_no_update();

    // 反向锁定: 转子 d 轴对齐 -α 轴（180° 电角度），负载角符号翻转
    set_dq_voltage(-FOC_MCPWM_CALIBRATE_VOLTAGE, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(3000)); // 锁定 3 秒，确保转子稳定后再采样
    float theta2 = encoder_->read_angle21_30_no_update();

    set_dq_voltage(0, 0, 0);

    if (std::isnan(theta1) || std::isnan(theta2)) {
        ESP_LOGE(TAG, "Zero electrical angle read failed, keeping previous offset");
    } else {
        // 机械角 → 电角度
        float alpha1 = theta1 * (float) pole_pairs_ * direction_;
        float alpha2 = theta2 * (float) pole_pairs_ * direction_;

        // 两次锁定应相差 180° 电角度，实际偏差 = 2×负载角（正常只有几度）
        float dev = alpha2 - alpha1 - PI_F;
        dev = std::fmod(dev, TWOPI_F);
        if (dev < 0.0f) dev += TWOPI_F;
        if (dev > PI_F) dev -= TWOPI_F; // 归一化到 (-π, π]
        if (std::fabs(dev) > 0.3f) {
            ESP_LOGW(TAG, "Bidirectional alignment deviation %.1f deg, reverse lock may have failed",
                     std::fabs(dev) * 180.0f / PI_F);
        }

        // 真实零电角度 = (α1 + (α2 − 180°)) / 2，负载角在平均中抵消
        zero_electric_angle_ = (alpha1 + alpha2 - PI_F) * 0.5f;
        // 归一化到 [0, 2π)
        zero_electric_angle_ = std::fmod(zero_electric_angle_, TWOPI_F);
        if (zero_electric_angle_ < 0.0f) {
            zero_electric_angle_ += TWOPI_F;
        }
        ESP_LOGI(TAG, "theta1=%.5f, theta2=%.5f, dev=%.2f deg, zero=%.8f rad",
                 theta1, theta2, dev * 180.0f / PI_F, zero_electric_angle_);
    }

    // ---- Step 3: 动态转速对称自整定（过冲减半 + 符号自适应） ----
    // 双向锁定只能消掉负载角死区，残余的 INL/迟滞/延迟偏差仍会造成正反转速度差。
    // 这里直接以"正反转转速一致"为闭环目标微调零电角度，一次性抵消所有残余系统误差。
    ESP_LOGI(TAG, "Step 3: Dynamic symmetry auto-tune...");
    calibrating_ = false; // 释放 FOC 任务，让闭环电压驱动电机旋转

    const float test_uq = 700.0f;     // 测试电压：中等偏上，拉开速度差又不过流
    const float probe_step = 0.06f;   // 符号探测步长 ~3.4° 电角度
    const float conv_step = 0.1f;     // 收敛起始步长 ~5.7° 电角度
    const float min_step = 0.0008f;   // 最小步长 ~0.046° 电角度
    const float diff_threshold = 1.0f; // 目标: 正反转速差 < 1 rev/s
    constexpr int MAX_ITER = 15;

    // 1) 探测 d(z) 斜率：d = 正转速度 − 反转速度 随 zero 的增减方向
    //    对 direction_/安装方向不敏感，避免方向检测翻转时整定发散
    float d0 = measure_speed_diff(test_uq);
    if (std::fabs(d0) <= diff_threshold) {
        ESP_LOGI(TAG, "Speeds already symmetric (diff=%.2f rev/s), skip tuning", d0);
    } else {
        zero_electric_angle_ += probe_step;
        zero_electric_angle_ = std::fmod(zero_electric_angle_, TWOPI_F);
        if (zero_electric_angle_ < 0.0f) {
            zero_electric_angle_ += TWOPI_F;
        }
        float d1 = measure_speed_diff(test_uq);

        // slope > 0: zero 增大时正转相对变快；slope < 0: 相反
        float slope = (d1 > d0) ? 1.0f : -1.0f;
        ESP_LOGI(TAG, "Slope probe: d0=%.2f, d1=%.2f, slope=%+.0f", d0, d1, slope);

        // 2) 过冲减半逼近：仅当快慢关系反转（越过最优点）时砍半步长
        //    初始偏差大时能一直大步走到过零点，比"每轮无脑减半"收敛更快
        float step = conv_step;
        float last_d = d1; // 探测后位置的偏差，作为过冲判断基准
        for (int i = 0; i < MAX_ITER; i++) {
            float d = measure_speed_diff(test_uq);
            if (std::fabs(d) <= diff_threshold) {
                ESP_LOGI(TAG, "Symmetry achieved: diff=%.2f rev/s, zero=%.6f rad",
                         d, (float) zero_electric_angle_);
                break;
            }
            // 过冲检测: d 与上一次符号相反说明跨过了零点，步长减半
            if (i > 0 && (d * last_d < 0.0f)) {
                step *= 0.5f;
                ESP_LOGI(TAG, "Overshoot detected, halving step to %.4f rad", step);
            }
            // d>0 正转快: slope>0 说明 zero 偏大要减小, slope<0 说明 zero 偏小要增大
            float correction = -slope * (d > 0.0f ? step : -step);
            zero_electric_angle_ += correction;
            zero_electric_angle_ = std::fmod(zero_electric_angle_, TWOPI_F);
            if (zero_electric_angle_ < 0.0f) {
                zero_electric_angle_ += TWOPI_F;
            }
            ESP_LOGI(TAG, "Iter %d: diff=%.2f rev/s, step=%.4f rad, zero=%.6f rad",
                     i + 1, d, step, (float) zero_electric_angle_);
            last_d = d;
            if (step < min_step) {
                ESP_LOGW(TAG, "Step reached minimum limit, stopping tune");
                break;
            }
        }
    }
    set_voltage(0, 0); // 整定结束停机

    // 完整打印，方便抄下来以后用 set_calibration_params 直接跳过校准
    ESP_LOGI(TAG, "Zero electrical angle: %.9f rad (%.6f deg)",
             (float) zero_electric_angle_,
             (float) (zero_electric_angle_ * 180.0f / PI_F));
    ESP_LOGI(TAG, "Copy: set_calibration_params(%+.1ff, %.9ff);",
             direction_, (float) zero_electric_angle_);
}
