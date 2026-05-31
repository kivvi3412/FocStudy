//
// Created by HAIRONG ZHU on 25-1-1.
//

#include "esp_foc.h"

#define SQRT3 1.7320508075688772935f

float calculate_electrical_angle(float mechanical_angle_rad, int pole_pairs) {
    return mechanical_angle_rad * (float)pole_pairs;
}

void foc_inverse_park_transform(float e_theta_rad, const foc_dq_coord_t *v_dq, foc_ab_coord_t *v_ab) {
    v_ab->alpha = v_dq->d * cosf(e_theta_rad) - v_dq->q * sinf(e_theta_rad);
    v_ab->beta = v_dq->d * sinf(e_theta_rad) + v_dq->q * cosf(e_theta_rad);
}

// SPWM调制, 实际使用时记得增加偏置
void foc_inverse_clarke_transform(const foc_ab_coord_t *v_ab, foc_uvw_coord_t *v_uvw) {
    v_uvw->u = v_ab->alpha;
    v_uvw->v = -0.5f * v_ab->alpha + sqrtf(3.0f) * 0.5f * v_ab->beta;
    v_uvw->w = -0.5f * v_ab->alpha - sqrtf(3.0f) * 0.5f * v_ab->beta;
}

// SVPWM 调制，实际使用时记得增加偏置
void foc_svpwm_duty_calculate(const foc_ab_coord_t *v_ab, foc_uvw_coord_t *out_uvw) {
    int sextant;

    if (v_ab->beta > 0.0f) {
        if (v_ab->alpha > 0.0f) {
            // 第一象限
            if (v_ab->beta > (v_ab->alpha * SQRT3)) {
                sextant = 2;    // 六分区 v2-v3
            } else {
                sextant = 1;    // 六分区 v1-v2
            }
        } else {
            // 第二象限
            if (-v_ab->beta > (v_ab->alpha * SQRT3)) {
                sextant = 3;    // 六分区 v3-v4
            } else {
                sextant = 2;    // 六分区 v2-v3
            }
        }
    } else {
        if (v_ab->alpha > 0.0f) {
            // 第四象限
            if (-v_ab->beta > (v_ab->alpha * SQRT3)) {
                sextant = 5;    // 六分区 v5-v6
            } else {
                sextant = 6;    // 六分区 v6-v1
            }
        } else {
            // 第三象限
            if (v_ab->beta > (v_ab->alpha * SQRT3)) {
                sextant = 4;    // 六分区 v4-v5
            } else {
                sextant = 5;    // 六分区 v5-v6
            }
        }
    }

    switch (sextant) {
        // 六分区 v1-v2
        case 1: {
            float t1 = (-v_ab->alpha * SQRT3) + v_ab->beta;
            float t2 = -2.0f * v_ab->beta;

            // PWM 时序
            out_uvw->u = (1.0f - t1 - t2) / 2.0f;
            out_uvw->v = out_uvw->u + t1;
            out_uvw->w = out_uvw->v + t2;
        }
            break;

            // 六分区 v2-v3
        case 2: {
            float t2 = (-v_ab->alpha * SQRT3) - v_ab->beta;
            float t3 = (v_ab->alpha * SQRT3) - v_ab->beta;

            // PWM 时序
            out_uvw->v = (1.0f - t2 - t3) / 2.0f;
            out_uvw->u = out_uvw->v + t3;
            out_uvw->w = out_uvw->u + t2;
        }
            break;

            // 六分区 v3-v4
        case 3: {
            float t3 = -2.0f * v_ab->beta;
            float t4 = (v_ab->alpha * SQRT3) + v_ab->beta;

            // PWM 时序
            out_uvw->v = (1.0f - t3 - t4) / 2.0f;
            out_uvw->w = out_uvw->v + t3;
            out_uvw->u = out_uvw->w + t4;
        }
            break;

            // 六分区 v4-v5
        case 4: {
            float t4 = (v_ab->alpha * SQRT3) - v_ab->beta;
            float t5 = 2.0f * v_ab->beta;

            // PWM 时序
            out_uvw->w = (1.0f - t4 - t5) / 2.0f;
            out_uvw->v = out_uvw->w + t5;
            out_uvw->u = out_uvw->v + t4;
        }
            break;

            // 六分区 v5-v6
        case 5: {
            float t5 = (v_ab->alpha * SQRT3) + v_ab->beta;
            float t6 = (-v_ab->alpha * SQRT3) + v_ab->beta;

            // PWM 时序
            out_uvw->w = (1.0f - t5 - t6) / 2.0f;
            out_uvw->u = out_uvw->w + t5;
            out_uvw->v = out_uvw->u + t6;
        }
            break;

            // 六分区 v6-v1
        case 6: {
            float t6 = 2.0f * v_ab->beta;
            float t1 = (-v_ab->alpha * SQRT3) - v_ab->beta;

            // PWM 时序
            out_uvw->u = (1.0f - t6 - t1) / 2.0f;
            out_uvw->w = out_uvw->u + t1;
            out_uvw->v = out_uvw->w + t6;
        }
            break;

        default:
            // 默认情况下，将所有相位设为零
            out_uvw->u = 0.0f;
            out_uvw->v = 0.0f;
            out_uvw->w = 0.0f;
            break;
    }
}

