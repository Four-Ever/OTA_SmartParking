/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: norm.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2025-02-10 21:14:29
 */

/* Include Files */
#include "norm.h"
#include <math.h>
#include <float.h>

/* Function Definitions */
/*
 * Arguments    : const double x[2]
 * Return Type  : double
 */
double b_norm(const double x[2]) {
    return sqrt(x[0] * x[0] + x[1] * x[1]);
}
/*
 * File trailer for norm.c
 *
 * [EOF]
 */
