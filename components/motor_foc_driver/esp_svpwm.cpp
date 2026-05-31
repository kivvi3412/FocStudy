//
// Created by HAIRONG ZHU on 25-1-1.
//

#include "esp_svpwm.h"

#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "esp_svpwm";


// mcpwm handler type
typedef struct mcpwm_svpwm_ctx {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operators[3];
    mcpwm_cmpr_handle_t comparators[3];
    mcpwm_gen_handle_t generators[3];
} mcpwm_svpwm_ctx_t;

/**
 * @brief 创建一个新的 SVPWM 逆变器实例
 *
 * @param config 逆变器的配置参数
 * @param ret_inverter 返回的逆变器句柄
 * @return esp_err_t ESP_OK 成功，其它失败错误码
 */
esp_err_t svpwm_new_inverter(const inverter_config_t *config, inverter_handle_t *ret_inverter) {
    esp_err_t ret;
    mcpwm_generator_config_t gen_config = {};

    // 检查输入参数是否有效，如果无效则返回错误
    ESP_RETURN_ON_FALSE(config && ret_inverter, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    // 分配内存用于存储 MCPWM SVPWM 上下文结构体，并初始化为0
    auto *svpwm_dev = (mcpwm_svpwm_ctx_t *) calloc(1, sizeof(mcpwm_svpwm_ctx_t));
    if (!svpwm_dev) {
        ESP_LOGE(TAG, "no memory");
        return ESP_ERR_NO_MEM; // 内存分配失败，返回错误
    }

    // 创建 MCPWM 定时器，并将其句柄存储在上下文结构体中
    ESP_GOTO_ON_ERROR(
        mcpwm_new_timer(&config->timer_config, &svpwm_dev->timer),
        err, TAG, "Create MCPWM timer failed"
    );

    // 为每个相位创建 MCPWM 运算器，并将其连接到同一个定时器
    for (int i = 0; i < 3; i++) {
        // 创建 MCPWM 运算器
        ESP_GOTO_ON_ERROR(
            mcpwm_new_operator(&config->operator_config, &svpwm_dev->operators[i]),
            err, TAG, "Create MCPWM operator failed"
        );
        // 将运算器连接到定时器
        ESP_GOTO_ON_ERROR(
            mcpwm_operator_connect_timer(svpwm_dev->operators[i], svpwm_dev->timer),
            err, TAG, "Connect operators to the same timer failed"
        );
    }

    // 为每个相位创建 MCPWM 比较器，并初始化比较值为0
    for (int i = 0; i < 3; i++) {
        // 创建 MCPWM 比较器
        ESP_GOTO_ON_ERROR(
            mcpwm_new_comparator(svpwm_dev->operators[i], &config->compare_config, &svpwm_dev->comparators[i]),
            err, TAG, "Create comparators failed"
        );
        // 设置比较器的比较值为0
        ESP_GOTO_ON_ERROR(
            mcpwm_comparator_set_compare_value(svpwm_dev->comparators[i], 0),
            err, TAG, "Set comparators failed"
        );
    }

    // 为每个相位的上桥臂创建 MCPWM 生成器
    for (int i = 0; i < 3; i++) {
        // 设置生成器的 GPIO 引脚号
        gen_config.gen_gpio_num = config->gen_gpios[i];
        // 创建 MCPWM 生成器，并将其句柄存储在上下文结构体中
        ESP_GOTO_ON_ERROR(
            mcpwm_new_generator(svpwm_dev->operators[i], &gen_config, &svpwm_dev->generators[i]),
            err, TAG, "Create PWM generator pin %d failed", gen_config.gen_gpio_num
        );
    }

    // 为每个相位的上桥臂生成器设置比较事件的动作
    for (int i = 0; i < 3; i++) {
        ESP_GOTO_ON_ERROR(
            mcpwm_generator_set_action_on_compare_event(
                svpwm_dev->generators[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, svpwm_dev->comparators[i],MCPWM_GEN_ACTION_LOW)
            ),
            err, TAG, "Set generator UP action failed"
        );
        ESP_GOTO_ON_ERROR(
            mcpwm_generator_set_action_on_compare_event(
                svpwm_dev->generators[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, svpwm_dev->comparators[i],
                    MCPWM_GEN_ACTION_HIGH)
            ),
            err, TAG, "Set generator DOWN action failed"
        );
    }

    *ret_inverter = svpwm_dev;
    return ESP_OK; // 成功返回
err:
    free(svpwm_dev); // 释放已分配的内存
    return ret; // 返回错误码
}

esp_err_t svpwm_inverter_start(inverter_handle_t handle, mcpwm_timer_start_stop_cmd_t command) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    if ((command != MCPWM_TIMER_STOP_EMPTY) && (command != MCPWM_TIMER_STOP_FULL)) {
        ESP_RETURN_ON_ERROR(mcpwm_timer_enable(handle->timer), TAG, "mcpwm timer enable failed");
    }
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(handle->timer, command), TAG, "mcpwm timer start failed");
    return ESP_OK;
}

esp_err_t svpwm_inverter_set_duty(inverter_handle_t handle, uint16_t u, uint16_t v, uint16_t w) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(handle->comparators[0], u), TAG, "set duty failed");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(handle->comparators[1], v), TAG, "set duty failed");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(handle->comparators[2], w), TAG, "set duty failed");
    return ESP_OK;
}

esp_err_t svpwm_del_inverter(inverter_handle_t handle) {
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    ESP_RETURN_ON_ERROR(mcpwm_timer_disable(handle->timer), TAG, "mcpwm timer disable failed");
    for (int i = 0; i < 3; i++) {
        ESP_RETURN_ON_ERROR(mcpwm_del_generator(handle->generators[i]), TAG, "free mcpwm positive generator failed");
        ESP_RETURN_ON_ERROR(mcpwm_del_comparator(handle->comparators[i]), TAG, "free mcpwm comparator failed");
        ESP_RETURN_ON_ERROR(mcpwm_del_operator(handle->operators[i]), TAG, "free mcpwm operator failed");
    }
    ESP_RETURN_ON_ERROR(mcpwm_del_timer(handle->timer), TAG, "free mcpwm timer failed");
    free(handle);
    return ESP_OK;
}
