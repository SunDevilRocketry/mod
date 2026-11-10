/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: AHRS_IKF_types.h
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 07-Nov-2026 17:12:12
 */

#ifndef AHRS_IKF_TYPES_H
#define AHRS_IKF_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_quaternion
#define typedef_quaternion
typedef struct
{
  float a;
  float b;
  float c;
  float d;
} quaternion;
#endif /* typedef_quaternion */

#ifndef typedef_AHRS_IKF
#define typedef_AHRS_IKF
typedef struct
{
  float gyro_noise;
  float gyro_bias;
  float mag_noise[3];
  float accel_noise[3];
  float small_angle_threshold;
  float last_update_time;
  float state_covariance_P[36];
  float bias_b[3];
  quaternion q;
  float G_mag[3];
} AHRS_IKF;
#endif /* typedef_AHRS_IKF */

#endif
/*
 * File trailer for AHRS_IKF_types.h
 *
 * [EOF]
 */
