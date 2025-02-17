/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: vel_filter.c
 *
 * Code generated for Simulink model 'vel_filter'.
 *
 * Model version                  : 1.77
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Thu Feb 13 21:14:01 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "vel_filter.h"
#include <math.h>
#include "rtwtypes.h"

/* Block states (default storage) */
DW_vel_filter_T vel_filter_DW;

/* External inputs (root inport signals with default storage) */
ExtU_vel_filter_T vel_filter_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_vel_filter_T vel_filter_Y;

/* Real-time model */
static RT_MODEL_vel_filter_T vel_filter_M_;
RT_MODEL_vel_filter_T *const vel_filter_M = &vel_filter_M_;

/* Model step function */
ExtY_vel_filter_T vel_filter_step(float accel_x,float accel_y)
{
    ExtY_vel_filter_T nowvelloc={0,0,0,0,0};
//  real_T rtb_accel_x;
//  real_T rtb_accel_y;

    //엑셀 값
    real_T rtb_accel_x = (double)accel_x;
    real_T rtb_accel_y = (double)accel_y;

  /* MATLAB Function: '<Root>/MATLAB Function8' incorporates:
   *  Inport: '<Root>/ACCEL_X'
   *  Inport: '<Root>/ACCEL_Y'
   */
//  rtb_accel_x = vel_filter_U.ACCEL_X;
//  rtb_accel_y = vel_filter_U.ACCEL_Y;
    //입력 노이즈 제거
  if (fabs(rtb_accel_x) < 0.02) {
    rtb_accel_x = 0.0;
  }

  if (fabs(rtb_accel_y) < 0.02) {
    rtb_accel_y = 0.0;
  }
  //단위 변경 m/s^2 to cm/s^2
  rtb_accel_x *= 981.0;
  rtb_accel_y *= 981.0;

  /* End of MATLAB Function: '<Root>/MATLAB Function8' */

  /* Switch: '<Root>/Switch2' incorporates:
   *  Constant: '<Root>/Constant2'
   */
  //엑셀x값 튀는 경우 보정
  if (!(rtb_accel_x > -1.0E+6)) {
    rtb_accel_x = 0.0;
  }

  /* DiscreteIntegrator: '<Root>/DTI_velocity_x' incorporates:
   *  Switch: '<Root>/Switch2'
   */
  ///DTI 필터 속도x
  vel_filter_DW.DTI_velocity_x_DSTATE += 0.01 * rtb_accel_x;

  /* Switch: '<Root>/Switch3' incorporates:
   *  Constant: '<Root>/Constant5'
   */
  //엑셀y값 튀는 경우 보정
  if (!(rtb_accel_y > -1.0E+6)) {
    rtb_accel_y = 0.0;
  }

  /* DiscreteIntegrator: '<Root>/DTLvelocity_y' incorporates:
   *  Switch: '<Root>/Switch3'
   */
  ///DTI 필터 속도y
  vel_filter_DW.DTLvelocity_y_DSTATE += 0.01 * rtb_accel_y;

  //속도 노이즈 제거
  /* MATLAB Function: '<Root>/MATLAB Function7' */
  vel_filter_Y.filtered_velocity_x = vel_filter_DW.DTI_velocity_x_DSTATE;
  vel_filter_Y.filtered_velocity_y = vel_filter_DW.DTLvelocity_y_DSTATE;
  if ((vel_filter_DW.DTI_velocity_x_DSTATE < 0.02) &&
      (vel_filter_DW.DTI_velocity_x_DSTATE > -0.02)) {
    vel_filter_Y.filtered_velocity_x = 0.0;
  }

  if ((vel_filter_DW.DTLvelocity_y_DSTATE < 0.02) &&
      (vel_filter_DW.DTLvelocity_y_DSTATE > -0.02)) {
    vel_filter_Y.filtered_velocity_y = 0.0;
  }

  //////////////////////////////////////////////////////위치

  /* End of MATLAB Function: '<Root>/MATLAB Function7' */

  /* DiscreteIntegrator: '<Root>/DTI_location_x' */
  //DTI 필터 위치x
  vel_filter_DW.DTI_location_x_DSTATE += 0.01 *
    vel_filter_Y.filtered_velocity_x;

  /* Sum: '<Root>/Add3' incorporates:
   *  Memory: '<Root>/Memory5'
   */
  vel_filter_Y.filtered_location_x += vel_filter_DW.DTI_location_x_DSTATE;

  /* Outport: '<Root>/speed' incorporates:
   *  Math: '<Root>/Square2'
   *  Math: '<Root>/Square3'
   *  Sqrt: '<Root>/Sqrt1'
   *  Sum: '<Root>/Add'
   */
  vel_filter_Y.speed = sqrt(vel_filter_Y.filtered_velocity_x *
    vel_filter_Y.filtered_velocity_x + vel_filter_Y.filtered_velocity_y *
    vel_filter_Y.filtered_velocity_y);

  /* DiscreteIntegrator: '<Root>/DTI_location_y' */
  //DTI 필터 위치y
  vel_filter_DW.DTI_location_y_DSTATE += 0.01 *
    vel_filter_Y.filtered_velocity_y;

  /* Sum: '<Root>/Add4' incorporates:
   *  Memory: '<Root>/Memory6'
   */
  vel_filter_Y.filtered_location_y += vel_filter_DW.DTI_location_y_DSTATE;
  nowvelloc.filtered_location_x = vel_filter_Y.filtered_location_x;
  nowvelloc.filtered_location_y = vel_filter_Y.filtered_location_y;
  nowvelloc.filtered_velocity_x = vel_filter_Y.filtered_velocity_x;
  nowvelloc.filtered_velocity_y = vel_filter_Y.filtered_velocity_y;
  nowvelloc.speed = vel_filter_Y.speed;

  return nowvelloc;
}

/* Model initialize function */
void vel_filter_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void vel_filter_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
