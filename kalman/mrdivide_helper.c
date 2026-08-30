/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mrdivide_helper.c
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 29-Aug-2026 19:35:42
 */

/* Include Files */
#include "mrdivide_helper.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const float A[6]
 *                const float B[4]
 *                float Y[6]
 * Return Type  : void
 */
void mrdiv(const float A[6], const float B[4], float Y[6])
{
  float a21;
  float a22;
  float a22_tmp;
  float c_a21_tmp;
  int a21_tmp;
  int b_a21_tmp;
  int r1;
  int r2;
  if (fabsf(B[2]) > fabsf(B[0]))
  {
    r1 = 1;
    r2 = 0;
  }
  else
  {
    r1 = 0;
    r2 = 1;
  }
  a21_tmp = r2 << 1;
  b_a21_tmp = r1 << 1;
  c_a21_tmp = B[b_a21_tmp];
  a21 = B[a21_tmp] / c_a21_tmp;
  a22_tmp = B[b_a21_tmp + 1];
  a22 = B[a21_tmp + 1] - a21 * a22_tmp;
  Y[r1] = A[0] / c_a21_tmp;
  Y[r2] = (A[1] - Y[r1] * a22_tmp) / a22;
  Y[r1] -= Y[r2] * a21;
  Y[r1 + 2] = A[2] / c_a21_tmp;
  Y[r2 + 2] = (A[3] - Y[r1 + 2] * a22_tmp) / a22;
  Y[r1 + 2] -= Y[r2 + 2] * a21;
  Y[r1 + 4] = A[4] / c_a21_tmp;
  Y[r2 + 4] = (A[5] - Y[r1 + 4] * a22_tmp) / a22;
  Y[r1 + 4] -= Y[r2 + 4] * a21;
}

/*
 * File trailer for mrdivide_helper.c
 *
 * [EOF]
 */
