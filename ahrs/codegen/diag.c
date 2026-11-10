/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: diag.c
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 07-Nov-2026 17:12:12
 */

/* Include Files */
#include "diag.h"

/* Function Definitions */
/*
 * Arguments    : const float v[3]
 *                float d[9]
 * Return Type  : void
 */
void diag(const float v[3], float d[9])
{
  int i;
  for (i = 0; i < 9; i++)
  {
    d[i] = 0.0F;
  }
  d[0] = v[0];
  d[4] = v[1];
  d[8] = v[2];
}

/*
 * File trailer for diag.c
 *
 * [EOF]
 */
