/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: atan2.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2025-02-10 21:14:29
 */

/* Include Files */
#include "atan2.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double y
 *                double x
 * Return Type  : double
 */
double b_atan2(double y, double x)
{
  double r;
  if (rtIsNaN(y) || rtIsNaN(x)) {
    r = rtNaN;
  } else if (rtIsInf(y) && rtIsInf(x)) {
    int i;
    int i1;
    if (y > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (x > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    r = atan2(i, i1);
  } else if (x == 0.0) {
    if (y > 0.0) {
      r = RT_PI / 2.0;
    } else if (y < 0.0) {
      r = -(RT_PI / 2.0);
    } else {
      r = 0.0;
    }
  } else {
    r = atan2(y, x);
  }
  return r;
}

/*
 * File trailer for atan2.c
 *
 * [EOF]
 */
