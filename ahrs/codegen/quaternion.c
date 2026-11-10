/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: quaternion.c
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 07-Nov-2026 17:12:12
 */

/* Include Files */
#include "quaternion.h"

/* Function Definitions */
/*
 * Arguments    : const float varargin_1[4]
 *                float *obj_b
 *                float *obj_c
 *                float *obj_d
 * Return Type  : float
 */
float quaternion_quaternion(const float varargin_1[4], float *obj_b,
                            float *obj_c, float *obj_d)
{
  float obj_a;
  obj_a = varargin_1[0];
  *obj_b = varargin_1[1];
  *obj_c = varargin_1[2];
  *obj_d = varargin_1[3];
  return obj_a;
}

/*
 * File trailer for quaternion.c
 *
 * [EOF]
 */
