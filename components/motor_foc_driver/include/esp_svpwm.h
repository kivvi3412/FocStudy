//
// Created by HAIRONG ZHU on 25-1-1.
//

#ifndef FOCBUTTON_ESP_SVPWM_H
#define FOCBUTTON_ESP_SVPWM_H

#include "driver/mcpwm_prelude.h"
/**
 * @brief   svpwm inverter config struct type
 */
typedef struct inverter_config {
    mcpwm_timer_config_t timer_config;
    mcpwm_operator_config_t operator_config;
    mcpwm_comparator_config_t compare_config;
    int gen_gpios[3];
} inverter_config_t;

/*
 * @brief   svpwm inverter handler type
 */
typedef struct mcpwm_svpwm_ctx  *inverter_handle_t;

/**
 * @brief Config mcpwm as a inverter with corresponding config value
 *
 * @param config        config value for mcpwm peripheral
 * @param ret_inverter  return handler for corresponding mcpwm
 *
 * @return  - ESP_OK: Create invertor successfully
 *          - ESP_ERR_INVALID_ARG: NULL arguments
 *          - ESP_ERR_NO_MEM: no free memory
 */
esp_err_t svpwm_new_inverter(const inverter_config_t *config, inverter_handle_t *ret_inverter);

/**
 * @brief start/stop a svpwm invertor
 *
 * @param handle    svpwm invertor handler
 * @param command   see "mcpwm_timer_start_stop_cmd_t"
 *
 * @return  - ESP_OK: start inverter successfully
 *          - ESP_ERR_INVALID_ARG: NULL arguments
 */
esp_err_t svpwm_inverter_start(inverter_handle_t handle, mcpwm_timer_start_stop_cmd_t command);

/**
 * @brief set 3 channels pwm comparator value for invertor
 *
 * @param handle  svpwm invertor handler
 * @param u comparator value for channel UH and UL
 * @param v comparator value for channel VH and VL
 * @param w comparator value for channel WH and WL
 *
 * @return  - ESP_OK: set compare value successfully
 *          - ESP_ERR_INVALID_ARG: NULL arguments
 */
esp_err_t svpwm_inverter_set_duty(inverter_handle_t handle, uint16_t u, uint16_t v, uint16_t w);

/**
 * @brief free a svpwm invertor
 *
 * @param handle  svpwm invertor handler
 *
 * @return  - ESP_OK: free inverter successfully
 *          - ESP_ERR_INVALID_ARG: NULL arguments
 */
esp_err_t svpwm_del_inverter(inverter_handle_t handle);


#endif //FOCBUTTON_ESP_SVPWM_H
