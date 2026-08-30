/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kalman_step.c
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 29-Aug-2026 19:35:42
 */

/* Include Files */
#include "kalman_step.h"
#include "kalman_init_data.h"
#include "kalman_init_initialize.h"
#include "kalman_init_types.h"
#include "mrdivide_helper.h"
#include "mw_cmsis.h"
#include "mw_cmsis_impl.h"

/* Function Definitions */
/*
 * Arguments    : float z_alt
 *                float z_acc
 *                float dt
 * Return Type  : KALMAN_OUTPUT
 */
KALMAN_OUTPUT kalman_step(float z_alt, float z_acc, float dt)
{
  KALMAN_OUTPUT kalman_outputs;
  float A[9];
  float PPred[9];
  float fv[9];
  float fv1[9];
  float fv2[9];
  float K[6];
  float fv3[6];
  float fv4[6];
  float fv5[4];
  float fv6[4];
  float G[3];
  float fv10[3];
  float xPred[3];
  float fv7[2];
  float fv8[2];
  float fv9[2];
  float A_tmp;
  int i;
  if (!isInitialized_kalman_init)
  {
    kalman_init_initialize();
  }
  /*  Measured altitude (m) */
  /*  Measured acceleration (m/s^2) */
  /*  Time step (s) */
  /*  private function to use kalman filter object */
  if (!instance_not_empty)
  {
    instance.sigma_j_sq = 0.0F;
    instance.R[0] = 0.0F;
    instance.R[1] = 0.0F;
    instance.R[2] = 0.0F;
    instance.R[3] = 0.0F;
    for (i = 0; i < 6; i++)
    {
      instance.H[i] = 0.0F;
    }
    instance.x[0] = 0.0F;
    instance.x[1] = 0.0F;
    instance.x[2] = 0.0F;
    for (i = 0; i < 9; i++)
    {
      instance.P[i] = 0.0F;
    }
    instance_not_empty = true;
  }
  /*  Measured altitude (m) */
  /*  Measured acceleration (m/s^2) */
  /*  Time step (s) */
  /*  1. State Transition Matrix (Kinematic Constant Acceleration Model) */
  A[0] = 1.0F;
  A[1] = dt;
  A_tmp = 0.5F * (dt * dt);
  A[2] = A_tmp;
  A[3] = 0.0F;
  A[4] = 1.0F;
  A[5] = dt;
  A[6] = 0.0F;
  A[7] = 0.0F;
  A[8] = 1.0F;
  /*  Continuous White Noise Jerk (Piecewise Constant Model) */
  G[0] = A_tmp;
  G[1] = dt;
  G[2] = 1.0F;
  /*  2. Predict step */
  mw_arm_mat_mult_f32(&A[0], &instance.x[0], &xPred[0], 3U, 3U, 1U);
  mw_arm_mat_trans_f32(&A[0], &fv[0], 3U, 3U);
  mw_arm_mat_mult_f32(&A[0], &instance.P[0], &fv1[0], 3U, 3U, 3U);
  mw_arm_mat_mult_f32(&fv1[0], &fv[0], &fv2[0], 3U, 3U, 3U);
  mw_arm_mat_mult_f32(&G[0], &G[0], &fv[0], 3U, 1U, 3U);
  mw_arm_scale_2_f32(&fv[0], &instance.sigma_j_sq, &fv1[0], 9U);
  mw_arm_add_f32(&fv2[0], &fv1[0], &PPred[0], 9U);
  /*  3. Measurement vector [altitude; acceleration] */
  /*  4. Compute Kalman Gain */
  mw_arm_mat_trans_f32(&instance.H[0], &K[0], 2U, 3U);
  mw_arm_mat_mult_f32(&PPred[0], &K[0], &fv3[0], 3U, 3U, 2U);
  mw_arm_mat_mult_f32(&instance.H[0], &PPred[0], &fv4[0], 2U, 3U, 3U);
  mw_arm_mat_mult_f32(&fv4[0], &K[0], &fv5[0], 2U, 3U, 2U);
  mw_arm_add_f32(&fv5[0], &instance.R[0], &fv6[0], 4U);
  mrdiv(fv3, fv6, K);
  /*  5. State update */
  /*  innovation vector */
  mw_arm_mat_mult_f32(&instance.H[0], &xPred[0], &fv7[0], 2U, 3U, 1U);
  fv8[0] = z_alt;
  fv8[1] = z_acc;
  mw_arm_sub_f32(&fv8[0], &fv7[0], &fv9[0], 2U);
  mw_arm_mat_mult_f32(&K[0], &fv9[0], &fv10[0], 3U, 2U, 1U);
  mw_arm_add_f32(&xPred[0], &fv10[0], &G[0], 3U);
  instance.x[0] = G[0];
  instance.x[1] = G[1];
  instance.x[2] = G[2];
  /*  6. Error covariance update (Joseph form) */
  mw_arm_mat_mult_f32(&K[0], &instance.H[0], &fv[0], 3U, 2U, 3U);
  for (i = 0; i < 9; i++)
  {
    fv1[i] = iv[i];
  }
  mw_arm_sub_f32(&fv1[0], &fv[0], &A[0], 9U);
  mw_arm_mat_trans_f32(&A[0], &fv[0], 3U, 3U);
  mw_arm_mat_mult_f32(&A[0], &PPred[0], &fv1[0], 3U, 3U, 3U);
  mw_arm_mat_mult_f32(&fv1[0], &fv[0], &fv2[0], 3U, 3U, 3U);
  mw_arm_mat_trans_f32(&K[0], &fv4[0], 3U, 2U);
  mw_arm_mat_mult_f32(&K[0], &instance.R[0], &fv3[0], 3U, 2U, 2U);
  mw_arm_mat_mult_f32(&fv3[0], &fv4[0], &fv[0], 3U, 2U, 3U);
  mw_arm_add_f32(&fv2[0], &fv[0], &instance.P[0], 9U);
  /*  Outputs */
  /*   alt, vel, accel */
  /*  ignoring P for now */
  kalman_outputs.alt = G[0];
  kalman_outputs.velo = G[1];
  kalman_outputs.accel = G[2];
  return kalman_outputs;
}

/*
 * File trailer for kalman_step.c
 *
 * [EOF]
 */
