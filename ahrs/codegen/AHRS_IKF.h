/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: AHRS_IKF.h
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 07-Nov-2026 17:12:12
 */

#ifndef AHRS_IKF_H
#define AHRS_IKF_H

/* Include Files */
#include "AHRS_IKF_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /* Function Declarations */
  extern void AHRS_IKF_AHRS_IKF(AHRS_IKF *obj, float gyro_noise,
                                float gyro_bias, const float mag_noise[3],
                                const float accel_noise[3],
                                const float global_mag_ref[3],
                                float small_angle_threshold, float b_time);

  extern void AHRS_IKF_get_accel_noise(const AHRS_IKF *obj, float result[3]);

  extern quaternion AHRS_IKF_get_attitude(const AHRS_IKF *obj);

  extern float AHRS_IKF_get_gyro_bias(const AHRS_IKF *obj);

  extern float AHRS_IKF_get_gyro_noise(const AHRS_IKF *obj);

  extern float AHRS_IKF_get_last_update_time(const AHRS_IKF *obj);

  extern void AHRS_IKF_get_mag_noise(const AHRS_IKF *obj, float result[3]);

  extern float AHRS_IKF_get_small_angle_threshold(const AHRS_IKF *obj);

  extern void AHRS_IKF_set_accel_noise(AHRS_IKF *obj, const float val[3]);

  extern void AHRS_IKF_set_gyro_bias(AHRS_IKF *obj, float val);

  extern void AHRS_IKF_set_gyro_noise(AHRS_IKF *obj, float val);

  extern void AHRS_IKF_set_last_update_time(AHRS_IKF *obj, float val);

  extern void AHRS_IKF_set_mag_noise(AHRS_IKF *obj, const float val[3]);

  extern void AHRS_IKF_set_small_angle_threshold(AHRS_IKF *obj, float val);

  extern void AHRS_IKF_update_accel(AHRS_IKF *obj, const float accel[3]);

  extern void AHRS_IKF_update_gyro(AHRS_IKF *obj, const float omega_m[3],
                                   float b_time);

  extern void AHRS_IKF_update_mag(AHRS_IKF *obj, const float mag[3]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for AHRS_IKF.h
 *
 * [EOF]
 */
