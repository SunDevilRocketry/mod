/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kalman_init_types.h
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 29-Aug-2026 21:27:14
 */

#ifndef KALMAN_INIT_TYPES_H
#define KALMAN_INIT_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_KALMAN_OUTPUT
#define typedef_KALMAN_OUTPUT
typedef struct
{
  float alt;
  float velo;
  float accel;
} KALMAN_OUTPUT;
#endif /* typedef_KALMAN_OUTPUT */

#ifndef typedef_AltKalman
#define typedef_AltKalman
typedef struct
{
  float sigma_j_sq;
  float R[4];
  float H[6];
  float x[3];
  float P[9];
} AltKalman;
#endif /* typedef_AltKalman */

#endif
/*
 * File trailer for kalman_init_types.h
 *
 * [EOF]
 */
