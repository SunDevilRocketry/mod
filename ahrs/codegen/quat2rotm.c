/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: quat2rotm.c
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 07-Nov-2026 17:12:12
 */

/* Include Files */
#include "quat2rotm.h"
#include "AHRS_IKF_data.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : float q_a
 *                float q_b
 *                float q_c
 *                float q_d
 *                float R[9]
 * Return Type  : void
 */
void quat2rotm(float q_a, float q_b, float q_c, float q_d, float R[9])
{
  float tempR2[9];
  float b_tempR2_tmp;
  float c_tempR2_tmp;
  float d_tempR2_tmp;
  float e_tempR2_tmp;
  float f_tempR2_tmp;
  float q_idx_0;
  float q_idx_1;
  float q_idx_2;
  float q_idx_3;
  float tempR2_tmp;
  float z;
  int i;
  int i1;
  int i2;
  regObj_not_empty = true;
  z = 1.0F / sqrtf(((powf(q_a, 2.0F) + powf(q_b, 2.0F)) + powf(q_c, 2.0F)) +
                   powf(q_d, 2.0F));
  q_idx_0 = q_a * z;
  q_idx_1 = q_b * z;
  q_idx_2 = q_c * z;
  q_idx_3 = q_d * z;
  tempR2_tmp = q_idx_3 * q_idx_3;
  b_tempR2_tmp = q_idx_2 * q_idx_2;
  tempR2[0] = 1.0F - 2.0F * (b_tempR2_tmp + tempR2_tmp);
  z = q_idx_1 * q_idx_2;
  c_tempR2_tmp = q_idx_0 * q_idx_3;
  tempR2[1] = 2.0F * (z + c_tempR2_tmp);
  d_tempR2_tmp = q_idx_1 * q_idx_3;
  e_tempR2_tmp = q_idx_0 * q_idx_2;
  tempR2[2] = 2.0F * (d_tempR2_tmp - e_tempR2_tmp);
  tempR2[3] = 2.0F * (z - c_tempR2_tmp);
  f_tempR2_tmp = q_idx_1 * q_idx_1;
  tempR2[4] = 1.0F - 2.0F * (f_tempR2_tmp + tempR2_tmp);
  c_tempR2_tmp = q_idx_2 * q_idx_3;
  z = q_idx_0 * q_idx_1;
  tempR2[5] = 2.0F * (c_tempR2_tmp + z);
  tempR2[6] = 2.0F * (d_tempR2_tmp + e_tempR2_tmp);
  tempR2[7] = 2.0F * (c_tempR2_tmp - z);
  tempR2[8] = 1.0F - 2.0F * (f_tempR2_tmp + b_tempR2_tmp);
  i = 0;
  i1 = 0;
  for (i2 = 0; i2 < 9; i2++)
  {
    R[i1 + 3 * i] = tempR2[i2];
    i++;
    if (i > 2)
    {
      i = 0;
      i1++;
    }
  }
}

/*
 * File trailer for quat2rotm.c
 *
 * [EOF]
 */
