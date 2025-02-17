/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: wrapToPi.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2025-02-10 21:14:29
 */

/* Include Files */
#include "wrapToPi.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double *lambda
 * Return Type  : void
 */
void wrapToPi(double *lambda)
{
  double tmp_data;
  int i;
  int trueCount;
  bool b;
  trueCount = 0;
  b = ((*lambda < -3.1415926535897931) || (*lambda > 3.1415926535897931));
  if (b) {
    double q;
    trueCount = 1;
    q = *lambda + 3.1415926535897931;
    if (rtIsNaN(q) || rtIsInf(q)) {
      tmp_data = rtNaN;
    } else if (q == 0.0) {
      tmp_data = 0.0;
    } else {
      bool rEQ0;
      tmp_data = fmod(q, 6.2831853071795862);
      rEQ0 = (tmp_data == 0.0);
      if (!rEQ0) {
        q = fabs(q / 6.2831853071795862);
        rEQ0 = !(fabs(q - floor(q + 0.5)) > 2.2204460492503131E-16 * q);
      }
      if (rEQ0) {
        tmp_data = 0.0;
      } else if (tmp_data < 0.0) {
        tmp_data += 6.2831853071795862;
      }
    }
  }
  for (i = 0; i < trueCount; i++) {
    if ((tmp_data == 0.0) && (*lambda + 3.1415926535897931 > 0.0)) {
      tmp_data = 6.2831853071795862;
    }
  }
  if (b) {
    *lambda = tmp_data - 3.1415926535897931;
  }
}

/*
 * File trailer for wrapToPi.c
 *
 * [EOF]
 */
