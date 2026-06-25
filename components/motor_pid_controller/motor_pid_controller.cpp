//
// Created by HAIRONG ZHU on 25-1-16.
//

#include "motor_pid_controller.h"
#include "project_conf.h"

#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

PIDController::PIDController(float kp, float ki, float kd, float output_limit, float integral_limit,
                             float static_friction_torque)
        : kp_(kp), ki_(ki), kd_(kd), output_limit_(output_limit), integral_limit_(integral_limit),
          static_friction_torque_(static_friction_torque) {}

float PIDController::calculate(float error) {
    float Ts = FOC_CALC_PERIOD * 1e-6f; // 单位: 秒

    // 计算 P 、 I 、 D 项
    proportional_ = kp_ * error;
    integral_ += ki_ * error * Ts;
    integral_ = _constrain(integral_, -integral_limit_, integral_limit_);
    derivative_ = kd_ * (error - previousError_) / Ts;
    previousError_ = error;

    // 计算未修正的控制输出
    float u = proportional_ + integral_ + derivative_;

    // 加入静摩擦力补偿
    if (u != 0) {
        u += (u > 0) ? static_friction_torque_ : -static_friction_torque_;
    }

    // 限制输出范围
    u = _constrain(u, -output_limit_, output_limit_);

    return u;
}

void PIDController::setPID(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}


