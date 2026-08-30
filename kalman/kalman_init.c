/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kalman_init.c
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 29-Aug-2026 19:35:42
 */

/* Include Files */
#include "kalman_init.h"
#include "kalman_init_data.h"
#include "kalman_init_initialize.h"
#include "kalman_init_types.h"

/* Function Definitions */
/*
 * Arguments    : float jerkNoiseVar
 *                float baroNoiseVar
 *                float accelNoiseVar
 * Return Type  : void
 */
void kalman_init(float jerkNoiseVar, float baroNoiseVar, float accelNoiseVar)
{
  static const signed char afterFunctorCast[6] = {1, 0, 0, 0, 0, 1};
  int i;
  if (!isInitialized_kalman_init)
  {
    kalman_init_initialize();
  }
  /*  private function to use kalman filter object */
  instance_not_empty = true;
  /*  Expected physical acceleration change rate (m^2/s^5) */
  /*  Baro altitude noise variance (m^2) */
  /*  Accelerometer noise variance (m^2/s^4) */
  instance.x[0] = 0.0F;
  instance.x[1] = 0.0F;
  instance.x[2] = 0.0F;
  instance.sigma_j_sq = jerkNoiseVar;
  /*  Measurement covariance matrix R */
  instance.R[0] = baroNoiseVar;
  instance.R[1] = 0.0F;
  instance.R[2] = 0.0F;
  instance.R[3] = accelNoiseVar;
  /*  Measurement matrix: altitude (row 1) and acceleration (row 2) */
  for (i = 0; i < 6; i++)
  {
    instance.H[i] = afterFunctorCast[i];
  }
  /*  Initial error covariance */
  for (i = 0; i < 9; i++)
  {
    instance.P[i] = iv[i];
  }
}

/*
 * File trailer for kalman_init.c
 *
 * [EOF]
 */
