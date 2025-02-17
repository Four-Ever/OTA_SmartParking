/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gitstanley.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2025-02-10 21:14:29
 */

#ifndef GITSTANLEY_H
#define GITSTANLEY_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>
#include <math.h>

#include "Ifx_Types.h"
#include "IfxAsclin_Asc.h"
#include "Ifx_Shell.h"
#include "Ifx_Console.h"
#include "IfxPort.h"
#include "decision_stateflow.h"
#include "STM_interrupt.h"

extern int IsWPTrackingFinish;
extern int Update_finished;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
float gitstanley(void);
void initStanley(void);
void updateWaypoints(float new_waypoints[][2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gitstanley.h
 *
 * [EOF]
 */
