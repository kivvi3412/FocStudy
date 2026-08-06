//
// 封装的 FOC 电机驱动器实现
//

#include "foc_driver.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "project_conf.h"
#include <cmath>

static const char *TAG = "FocMotor";

#ifndef M_TWOPI
#define M_TWOPI (2.0f * (float)M_PI)
#endif

#define SQRT3 1.7320508075688772935f
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

FocMotor::FocMotor(MT6816 *encoder, int pole_pairs, int u_gpio, int v_gpio, int w_gpio, int en_gpio)
    : encoder_(encoder), pole_pairs_(pole_pairs), en_gpio_(en_gpio) {
    // 初始化使能引脚
    gpio_config_t en_cfg = {};
    en_cfg.pin_bit_mask = 1ULL << en_gpio_;
    en_cfg.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&en_cfg));
    gpio_set_level((gpio_num_t) en_gpio_, 0); // 默认关闭

    // 初始化 MCPWM
    init_mcpwm(u_gpio, v_gpio, w_gpio);

    // 注册 MCPWM on_empty 回调 (必须在 enable 之前)
    mcpwm_timer_event_callbacks_t cbs = {};
    cbs.on_empty = mcpwm_on_empty_cb;
    ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timer_, &cbs, this));

    // 启动 MCPWM 定时器
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer_));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP));

    // 创建 FOC 任务 (高优先级, 绑核 1)
    // CPU1 的 IDLE 看门狗默认关闭，适合实时忙等循环
    xTaskCreatePinnedToCore(foc_task, "foc_task", 4096, this, 20, &foc_task_handle_, 1);

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
}

void FocMotor::park_inverse(float theta, float d, float q, float *alpha, float *beta) {
    *alpha = d * cosf(theta) - q * sinf(theta);
    *beta = d * sinf(theta) + q * cosf(theta);
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

bool FocMotor::mcpwm_on_empty_cb(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx) {
    const auto *self = static_cast<FocMotor *>(user_ctx);
    BaseType_t high_task_woken = pdFALSE;
    if (self->foc_task_handle_) {
        vTaskNotifyGiveFromISR(self->foc_task_handle_, &high_task_woken);
    }
    return high_task_woken;
}

void FocMotor::foc_task(void *arg) {
    auto *self = static_cast<FocMotor *>(arg);
    uint32_t error_count = 0;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!self->enabled_ || self->calibrating_) {
            error_count = 0;
            continue;
        }

        float e_theta = self->encoder_->read_electrical_angle_compensated(
            self->pole_pairs_, self->direction_, self->zero_electric_angle_
        );

        if (std::isnan(e_theta)) {
            error_count++;
            if (error_count > 20) {
                self->disable();
                ESP_LOGE(TAG, "Encoder error (Parity/Magnet) > 20 times! Emergency Stop.");
            }
            continue;
        }

        error_count = 0;
        self->set_dq_voltage(self->ud_, self->direction_ * self->uq_, e_theta);
    }
}

void FocMotor::set_voltage(float ud, float uq) {
    ud_ = ud;
    uq_ = uq;
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

    // ---- Step 1: 方向检测 ----
    ESP_LOGI(TAG, "Step 1: Detecting motor direction...");
    float theta = 0;
    float delta_theta = M_TWOPI * 0.01f;
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
    if (angle_diff < -(float) M_PI)
        angle_diff += M_TWOPI;
    else if (angle_diff > (float) M_PI)
        angle_diff -= M_TWOPI;

    direction_ = (angle_diff > 0) ? 1.0f : -1.0f;
    ESP_LOGI(TAG, "Motor direction: %.1f", direction_);

    // ---- Step 2: 零电角度 ----
    ESP_LOGI(TAG, "Step 2: Setting zero electrical angle...");
    vTaskDelay(pdMS_TO_TICKS(500));

    set_dq_voltage(FOC_MCPWM_CALIBRATE_VOLTAGE, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    float align_angle = encoder_->read_angle_no_update();
    zero_electric_angle_ = align_angle * (float) pole_pairs_ * direction_;

    vTaskDelay(pdMS_TO_TICKS(100));
    set_dq_voltage(0, 0, 0);

    ESP_LOGI(TAG, "Zero electrical angle: %.8f rad", zero_electric_angle_);
    ESP_LOGI(TAG, "=== Calibration Complete ===");

    calibrating_ = false;
}
