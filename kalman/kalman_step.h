/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kalman_step.h
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 29-Aug-2026 21:27:14
 */

#ifndef KALMAN_STEP_H
#define KALMAN_STEP_H

/* Include Files */
#include "kalman_init_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /* Function Declarations */
  extern KALMAN_OUTPUT kalman_step(float z_alt, float z_acc, float dt);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for kalman_step.h
 *
 * [EOF]
 */
