//
// Created by HAIRONG ZHU on 25-1-1.
//

#ifndef FOCBUTTON_ESP_FOC_H
#define FOCBUTTON_ESP_FOC_H

#include <cmath>

// 3-phase uvw coord data type
typedef struct foc_uvw_coord {
    float u;      // U phase data
    float v;      // V phase data
    float w;      // W phase data
} foc_uvw_coord_t;

// alpha-beta axis static coord data type
typedef struct foc_ab_coord {
    float alpha;  // alpha axis data
    float beta;   // beta axis data
} foc_ab_coord_t;

//d-q (direct-quadrature) axis rotate coord data type
typedef struct foc_dq_coord {
    float d;      // direct axis data
    float q;      // quadrature axis data
} foc_dq_coord_t;

/**
 * @brief Calculate electrical angle
 *
 * @param mechanical_angle_rad  Mechanical angle in radians
 * @param pole_pairs            Number of pole pairs
 * @return float                Electrical angle in radians
 */
float calculate_electrical_angle(float mechanical_angle_rad, int pole_pairs);

/**
 * @brief inverse park transform, to transform value in rotate d-q system to alpha_beta system
 *
 * @param[in] e_theta_rad     theta of dq_coord refer to alpha-beta coord, in rad (电角度, 电角度 = 机械角度 * 极对数)
 * @param[in] v_dq          data in dq coord to be transformed
 * @param[out] v_ab         output data in alpha-beta coord
 */
void foc_inverse_park_transform(float e_theta_rad, const foc_dq_coord_t *v_dq, foc_ab_coord_t *v_ab);


/**
 * @brief inverse clark transform, to transform value in alpha_beta system to 3phase uvw system
 *
 * @param[in] v_ab      data in alpha-beta coord to be transformed
 * @param[out] v_uvw    output data in 3-phase coord
 */
void foc_inverse_clarke_transform(const foc_ab_coord_t *v_ab, foc_uvw_coord_t *v_uvw);

/**
 * @brief 7-segment svpwm modulation
 *
 * @param v_ab[in]      input value in alpha-beta coord
 * @param out_uvw[out]  output modulated pwm duty
 */
void foc_svpwm_duty_calculate(const foc_ab_coord_t *v_ab, foc_uvw_coord_t *out_uvw);

#endif //FOCBUTTON_ESP_FOC_H
