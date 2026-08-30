/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kalman_step.c
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 29-Aug-2026 21:27:14
 */

/* Include Files */
#include "kalman_step.h"
#include "kalman_init_data.h"
#include "kalman_init_initialize.h"
#include "kalman_init_types.h"
#include "mrdivide_helper.h"
#include <string.h>

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
  float b_G[9];
  float b_PPred[9];
  float K[6];
  float b_K[6];
  float c_PPred[6];
  float c_K[4];
  float G[3];
  float b_z_alt[3];
  float xPred[3];
  float b_xPred[2];
  float c_z_alt[2];
  float A_tmp;
  float f;
  float f1;
  int PPred_tmp;
  int b_PPred_tmp;
  int b_i;
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
  memset(&b_z_alt[0], 0, 3U * sizeof(float));
  memset(&PPred[0], 0, 9U * sizeof(float));
  for (i = 0; i < 3; i++)
  {
    A_tmp = b_z_alt[i];
    f = PPred[3 * i];
    PPred_tmp = 3 * i + 1;
    b_PPred_tmp = 3 * i + 2;
    for (b_i = 0; b_i < 3; b_i++)
    {
      int i1;
      i1 = b_i + 3 * i;
      f1 = A[i1];
      A_tmp += instance.x[b_i] * f1;
      f += instance.P[3 * b_i] * f1;
      PPred[PPred_tmp] += instance.P[3 * b_i + 1] * f1;
      PPred[b_PPred_tmp] += instance.P[3 * b_i + 2] * f1;
      b_G[i1] = G[b_i] * G[i];
    }
    PPred[3 * i] = f;
    b_z_alt[i] = A_tmp;
    xPred[i] = A_tmp;
  }
  /*  3. Measurement vector [altitude; acceleration] */
  /*  4. Compute Kalman Gain */
  for (b_i = 0; b_i < 3; b_i++)
  {
    A_tmp = A[3 * b_i];
    f = A[3 * b_i + 1];
    f1 = A[3 * b_i + 2];
    for (i = 0; i < 3; i++)
    {
      PPred_tmp = b_i + 3 * i;
      b_PPred[PPred_tmp] = ((A_tmp * PPred[3 * i] + f * PPred[3 * i + 1]) +
                            f1 * PPred[3 * i + 2]) +
                           b_G[PPred_tmp] * instance.sigma_j_sq;
    }
    PPred_tmp = b_i << 1;
    K[PPred_tmp] = instance.H[b_i];
    K[PPred_tmp + 1] = instance.H[b_i + 3];
  }
  memset(&c_PPred[0], 0, 6U * sizeof(float));
  for (i = 0; i < 2; i++)
  {
    A_tmp = c_PPred[3 * i];
    PPred_tmp = 3 * i + 1;
    b_PPred_tmp = 3 * i + 2;
    for (b_i = 0; b_i < 3; b_i++)
    {
      f = instance.H[b_i + 3 * i];
      A_tmp += b_PPred[3 * b_i] * f;
      c_PPred[PPred_tmp] += b_PPred[3 * b_i + 1] * f;
      c_PPred[b_PPred_tmp] += b_PPred[3 * b_i + 2] * f;
    }
    c_PPred[3 * i] = A_tmp;
  }
  memset(&b_K[0], 0, 6U * sizeof(float));
  for (i = 0; i < 3; i++)
  {
    PPred_tmp = i << 1;
    for (b_i = 0; b_i < 3; b_i++)
    {
      A_tmp = b_PPred[b_i + 3 * i];
      b_PPred_tmp = b_i << 1;
      b_K[PPred_tmp] += K[b_PPred_tmp] * A_tmp;
      b_K[PPred_tmp + 1] += K[b_PPred_tmp + 1] * A_tmp;
    }
  }
  /*  5. State update */
  /*  innovation vector */
  memset(&b_xPred[0], 0, sizeof(float) << 1);
  c_z_alt[0] = z_alt;
  c_z_alt[1] = z_acc;
  for (i = 0; i < 2; i++)
  {
    A_tmp = K[i];
    f = K[i + 2];
    f1 = K[i + 4];
    for (b_i = 0; b_i < 2; b_i++)
    {
      PPred_tmp = i + (b_i << 1);
      c_K[PPred_tmp] = ((A_tmp * c_PPred[3 * b_i] + f * c_PPred[3 * b_i + 1]) +
                        f1 * c_PPred[3 * b_i + 2]) +
                       instance.R[PPred_tmp];
    }
    b_xPred[i] = c_z_alt[i] - (((b_xPred[i] + xPred[0] * instance.H[3 * i]) +
                                xPred[1] * instance.H[3 * i + 1]) +
                               xPred[2] * instance.H[3 * i + 2]);
  }
  mrdiv(b_K, c_K, K);
  memset(&b_z_alt[0], 0, 3U * sizeof(float));
  /*  6. Error covariance update (Joseph form) */
  for (b_i = 0; b_i < 3; b_i++)
  {
    PPred_tmp = b_i << 1;
    A_tmp = (b_z_alt[b_i] + b_xPred[0] * K[PPred_tmp]) +
            b_xPred[1] * K[PPred_tmp + 1];
    b_z_alt[b_i] = A_tmp;
    A_tmp += xPred[b_i];
    G[b_i] = A_tmp;
    instance.x[b_i] = A_tmp;
    A_tmp = instance.H[b_i];
    f = instance.H[b_i + 3];
    for (i = 0; i < 3; i++)
    {
      PPred_tmp = i << 1;
      b_PPred_tmp = b_i + 3 * i;
      A[b_PPred_tmp] = (float)iv[b_PPred_tmp] -
                       (A_tmp * K[PPred_tmp] + f * K[PPred_tmp + 1]);
    }
  }
  memset(&PPred[0], 0, 9U * sizeof(float));
  memset(&b_K[0], 0, 6U * sizeof(float));
  for (b_i = 0; b_i < 3; b_i++)
  {
    float f2;
    float f3;
    f2 = PPred[3 * b_i];
    PPred_tmp = 3 * b_i + 1;
    b_PPred_tmp = 3 * b_i + 2;
    for (i = 0; i < 3; i++)
    {
      A_tmp = A[i + 3 * b_i];
      f2 += b_PPred[3 * i] * A_tmp;
      PPred[PPred_tmp] += b_PPred[3 * i + 1] * A_tmp;
      PPred[b_PPred_tmp] += b_PPred[3 * i + 2] * A_tmp;
    }
    PPred[3 * b_i] = f2;
    PPred_tmp = b_i << 1;
    A_tmp = K[PPred_tmp];
    f1 = b_K[PPred_tmp] + instance.R[0] * A_tmp;
    f3 = b_K[PPred_tmp + 1] + instance.R[1] * A_tmp;
    A_tmp = K[PPred_tmp + 1];
    f1 += instance.R[2] * A_tmp;
    b_K[PPred_tmp] = f1;
    f3 += instance.R[3] * A_tmp;
    b_K[PPred_tmp + 1] = f3;
    A_tmp = PPred[3 * b_i + 1];
    f = PPred[3 * b_i + 2];
    for (i = 0; i < 3; i++)
    {
      b_G[i + 3 * b_i] =
          (A[3 * i] * f2 + A[3 * i + 1] * A_tmp) + A[3 * i + 2] * f;
    }
    for (i = 0; i < 3; i++)
    {
      PPred_tmp = i << 1;
      PPred[i + 3 * b_i] = K[PPred_tmp] * f1 + K[PPred_tmp + 1] * f3;
    }
  }
  for (i = 0; i < 9; i++)
  {
    instance.P[i] = b_G[i] + PPred[i];
  }
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
