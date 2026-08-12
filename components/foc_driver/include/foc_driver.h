//
// 封装的 FOC 电机驱动器 (基于 ESP-IDF mcpwm)
//

#ifndef FOC_DRIVER_H
#define FOC_DRIVER_H

#include "mt6835_driver.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

class FocMotor {
public:
    /**
     * @brief 构造函数，初始化 MCPWM 硬件和引脚
     * @param encoder MT6835 编码器指针
     * @param pole_pairs 电机极对数
     * @param u_gpio, v_gpio, w_gpio MCPWM 三相输出引脚
     * @param en_gpio 电机驱动使能引脚
     */
    FocMotor(MT6835 *encoder, int pole_pairs, int u_gpio, int v_gpio, int w_gpio, int en_gpio);

    /**
     * @brief 自动检测电机旋转方向和零电角度校准
     */
    void calibrate();

    /**
     * @brief 手动设置校准参数（跳过自动校准）
     * @param direction 电机方向 (1.0 或 -1.0)
     * @param zero_electric_angle 零电角度偏移 (弧度)
     */
    void set_calibration_params(float direction, float zero_electric_angle);

    /**
     * @brief 使能电机驱动并开始 FOC 任务循环
     */
    void enable();

    /**
     * @brief 关闭电机驱动并暂停 FOC 输出
     */
    void disable();

    /**
     * @brief 设置开环电压 Ud, Uq (范围 -limit ~ limit)
     * @param ud D轴电压
     * @param uq Q轴电压
     */
    void set_voltage(float ud, float uq);

    float get_direction() const { return direction_; }
    float get_zero_electric_angle() const { return zero_electric_angle_; }

private:
    MT6835 *encoder_;
    int pole_pairs_;
    int en_gpio_;

    float direction_{1.0f};
    float zero_electric_angle_{0.0f};

    volatile float ud_{0.0f};
    volatile float uq_{0.0f};
    volatile bool enabled_{false};
    volatile bool calibrating_{false};

    mcpwm_timer_handle_t timer_{nullptr};
    mcpwm_cmpr_handle_t comparators_[3]{};
    TaskHandle_t foc_task_handle_{nullptr};

    void init_mcpwm(int u, int v, int w);

    void IRAM_ATTR set_pwm_duties(float su, float sv, float sw);

    void IRAM_ATTR set_dq_voltage(float ud, float uq, float e_theta);

    static void IRAM_ATTR park_inverse(float theta, float d, float q, float *alpha, float *beta);

    static void IRAM_ATTR svpwm_calculate(float alpha, float beta, float *u, float *v, float *w);

    static bool IRAM_ATTR mcpwm_on_full_cb(
        mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx);

    static void IRAM_ATTR foc_task(void *arg);

    /// 编码器故障回调（由 MT6835 在连续读取失败时调用）
    static void encoder_fault_handler(void *ctx);
};

#endif // FOC_DRIVER_H
