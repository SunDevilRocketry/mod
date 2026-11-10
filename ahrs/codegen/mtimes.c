/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mtimes.c
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 07-Nov-2026 17:12:12
 */

/* Include Files */
#include "mtimes.h"

/* Function Definitions */
/*
 * Arguments    : float x_a
 *                float x_b
 *                float x_c
 *                float x_d
 *                float y_a
 *                float y_b
 *                float y_c
 *                float y_d
 *                float *o_b
 *                float *o_c
 *                float *o_d
 * Return Type  : float
 */
float quaternionBase_mtimes(float x_a, float x_b, float x_c, float x_d,
                            float y_a, float y_b, float y_c, float y_d,
                            float *o_b, float *o_c, float *o_d)
{
  float o_a;
  o_a = ((x_a * y_a - x_b * y_b) - x_c * y_c) - x_d * y_d;
  *o_b = ((x_a * y_b + x_b * y_a) + x_c * y_d) - x_d * y_c;
  *o_c = ((x_a * y_c - x_b * y_d) + x_c * y_a) + x_d * y_b;
  *o_d = ((x_a * y_d + x_b * y_c) - x_c * y_b) + x_d * y_a;
  return o_a;
}

/*
 * File trailer for mtimes.c
 *
 * [EOF]
 */
