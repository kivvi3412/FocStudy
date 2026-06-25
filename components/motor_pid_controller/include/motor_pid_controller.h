//
// Created by HAIRONG ZHU on 25-1-16.
//

#ifndef FOCKNOB_MOTOR_PID_CONTROLLER_H
#define FOCKNOB_MOTOR_PID_CONTROLLER_H


class PIDController {
public:
    PIDController(float kp, float ki, float kd, float output_limit, float integral_limit, float static_friction_torque);

    void setPID(float kp, float ki, float kd);

    float calculate(float error);

private:
    float kp_{};
    float ki_{};
    float kd_{};
    float output_limit_{};  // 输出限幅
    float integral_limit_{};    // 积分限幅
    float static_friction_torque_{};

    float proportional_{};
    float integral_{};
    float derivative_{};

    float previousError_{};
};


#endif //FOCKNOB_MOTOR_PID_CONTROLLER_H
