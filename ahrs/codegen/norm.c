/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: norm.c
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 07-Nov-2026 17:12:12
 */

/* Include Files */
#include "norm.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const float x[3]
 * Return Type  : float
 */
float b_norm(const float x[3])
{
  float absxk;
  float scale;
  float t;
  float y;
  scale = 1.2924697E-26F;
  absxk = fabsf(x[0]);
  if (absxk > 1.2924697E-26F)
  {
    y = 1.0F;
    scale = absxk;
  }
  else
  {
    t = absxk / 1.2924697E-26F;
    y = t * t;
  }
  absxk = fabsf(x[1]);
  if (absxk > scale)
  {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  }
  else
  {
    t = absxk / scale;
    y += t * t;
  }
  absxk = fabsf(x[2]);
  if (absxk > scale)
  {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  }
  else
  {
    t = absxk / scale;
    y += t * t;
  }
  return scale * sqrtf(y);
}

/*
 * File trailer for norm.c
 *
 * [EOF]
 */
