//
// Created by HAIRONG ZHU on 25-1-3.
//

#ifndef FOCKNOB_PROJECT_CONF_H
#define FOCKNOB_PROJECT_CONF_H

#define FOC_MCPWM_EN_GPIO               GPIO_NUM_12
#define FOC_MCPWM_POLE_PAIRS            7
#define FOC_MCPWM_U_GPIO                GPIO_NUM_32
#define FOC_MCPWM_V_GPIO                GPIO_NUM_33
#define FOC_MCPWM_W_GPIO                GPIO_NUM_25
#define FOC_CALC_PERIOD                 2000                // 电机控制周期，单位(us)，当前代表1ms
#define FOC_MCPWM_TIMER_RESOLUTION_HZ   80000000
#define FOC_MCPWM_PERIOD                2000                // 最大力矩为 FOC_MCPWM_PERIOD / 2
#define FOC_MCPWM_OUTPUT_LIMIT          (FOC_MCPWM_PERIOD / 2.0 - 1)


#endif //FOCKNOB_PROJECT_CONF_H
