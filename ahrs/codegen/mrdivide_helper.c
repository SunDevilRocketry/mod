/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mrdivide_helper.c
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 07-Nov-2026 17:12:12
 */

/* Include Files */
#include "mrdivide_helper.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const float A[18]
 *                const float B[9]
 *                float Y[18]
 * Return Type  : void
 */
void mrdiv(const float A[18], const float B[9], float Y[18])
{
  float b_A[9];
  float a21;
  float maxval;
  int A_tmp;
  int b_A_tmp;
  int c_A_tmp;
  int d_A_tmp;
  int e_A_tmp;
  int k;
  int r1;
  int r2;
  int r3;
  int rtemp;
  for (k = 0; k < 9; k++)
  {
    b_A[k] = B[k];
  }
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabsf(B[0]);
  a21 = fabsf(B[3]);
  if (a21 > maxval)
  {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (fabsf(B[6]) > maxval)
  {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  b_A[3 * r2] = B[3 * r2] / B[3 * r1];
  b_A[3 * r3] /= b_A[3 * r1];
  A_tmp = 3 * r2 + 1;
  b_A_tmp = 3 * r1 + 1;
  b_A[A_tmp] -= b_A[3 * r2] * b_A[b_A_tmp];
  c_A_tmp = 3 * r3 + 1;
  b_A[c_A_tmp] -= b_A[3 * r3] * b_A[b_A_tmp];
  rtemp = 3 * r2 + 2;
  d_A_tmp = 3 * r1 + 2;
  b_A[rtemp] -= b_A[3 * r2] * b_A[d_A_tmp];
  rtemp = 3 * r3 + 2;
  b_A[rtemp] -= b_A[3 * r3] * b_A[d_A_tmp];
  if (fabsf(b_A[c_A_tmp]) > fabsf(b_A[A_tmp]))
  {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  rtemp = 3 * r3 + 1;
  A_tmp = 3 * r2 + 1;
  b_A[rtemp] /= b_A[A_tmp];
  c_A_tmp = 3 * r3 + 2;
  e_A_tmp = 3 * r2 + 2;
  b_A[c_A_tmp] -= b_A[rtemp] * b_A[e_A_tmp];
  for (k = 0; k < 6; k++)
  {
    int Y_tmp;
    int b_Y_tmp;
    int c_Y_tmp;
    Y_tmp = r1 + 3 * k;
    Y[Y_tmp] = A[3 * k] / b_A[3 * r1];
    b_Y_tmp = r2 + 3 * k;
    Y[b_Y_tmp] = A[3 * k + 1] - Y[Y_tmp] * b_A[b_A_tmp];
    c_Y_tmp = r3 + 3 * k;
    Y[c_Y_tmp] = A[3 * k + 2] - Y[Y_tmp] * b_A[d_A_tmp];
    Y[b_Y_tmp] /= b_A[A_tmp];
    Y[c_Y_tmp] -= Y[b_Y_tmp] * b_A[e_A_tmp];
    Y[c_Y_tmp] /= b_A[c_A_tmp];
    Y[b_Y_tmp] -= Y[c_Y_tmp] * b_A[rtemp];
    Y[Y_tmp] -= Y[c_Y_tmp] * b_A[3 * r3];
    Y[Y_tmp] -= Y[b_Y_tmp] * b_A[3 * r2];
  }
}

/*
 * File trailer for mrdivide_helper.c
 *
 * [EOF]
 */
