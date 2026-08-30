/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kalman_init_initialize.c
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 29-Aug-2026 21:27:14
 */

/* Include Files */
#include "kalman_init_initialize.h"
#include "get_kalman.h"
#include "kalman_init_data.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void kalman_init_initialize(void)
{
  get_kalman_init();
  isInitialized_kalman_init = true;
}

/*
 * File trailer for kalman_init_initialize.c
 *
 * [EOF]
 */
