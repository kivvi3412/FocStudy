//
// Created by HAIRONG ZHU on 25-1-3.
//

#ifndef FOCKNOB_PROJECT_CONF_H
#define FOCKNOB_PROJECT_CONF_H

#define IIC_MASTER_NUM                  I2C_NUM_0         // IIC port number for master
#define IIC_MASTER_FREQ_HZ              400000            // IIC master clock frequency
#define IIC_MASTER_SDA0_IO              GPIO_NUM_19
#define IIC_MASTER_SCL0_IO              GPIO_NUM_18
#define IIC_MASTER_SDA1_IO              GPIO_NUM_23
#define IIC_MASTER_SCL1_IO              GPIO_NUM_5

#define IIC_AS5600_ADDR                 0x36
#define IIC_AS5600_RAW_ANGLE_REG        0x0C
#define IIC_AS5600_RESOLUTION           4096

#define FOC_MOTOR_POLE_PAIRS            7
#define FOC_MCPWM_U0_GPIO               GPIO_NUM_32
#define FOC_MCPWM_V0_GPIO               GPIO_NUM_33
#define FOC_MCPWM_W0_GPIO               GPIO_NUM_25
#define FOC_MCPWM_U1_GPIO               GPIO_NUM_26
#define FOC_MCPWM_V1_GPIO               GPIO_NUM_27
#define FOC_MCPWM_W1_GPIO               GPIO_NUM_14
#define FOC_DRV_EN_GPIO                 GPIO_NUM_12

#define FOC_CALC_PERIOD                 200                // 电机控制周期，单位(us)
#define FOC_MCPWM_TIMER_RESOLUTION_HZ   80000000
#define FOC_MCPWM_PERIOD                2000                // 最大力矩为 FOC_MCPWM_PERIOD / 2
#define FOC_MCPWM_OUTPUT_LIMIT          (FOC_MCPWM_PERIOD / 2.0 - 1)
#define FOC_MCPWM_CALIBRATE_VOLTAGE     (FOC_MCPWM_PERIOD / 10.0)
#define FOC_MCPWM_STATIC_FRIC_TORQUE    28.0                // 电机启动静摩擦力矩
#define FOC_LOW_PASS_FILTER_ALPHA       0.3

#endif //FOCKNOB_PROJECT_CONF_H
