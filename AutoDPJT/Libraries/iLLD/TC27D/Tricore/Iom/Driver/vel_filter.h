/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: vel_filter.h
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

#ifndef vel_filter_h_
#define vel_filter_h_
#ifndef vel_filter_COMMON_INCLUDES_
#define vel_filter_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* vel_filter_COMMON_INCLUDES_ */

#include "vel_filter_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DTI_velocity_x_DSTATE;        /* '<Root>/DTI_velocity_x' */
  real_T DTLvelocity_y_DSTATE;         /* '<Root>/DTLvelocity_y' */
  real_T DTI_location_x_DSTATE;        /* '<Root>/DTI_location_x' */
  real_T DTI_location_y_DSTATE;        /* '<Root>/DTI_location_y' */
} DW_vel_filter_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T ACCEL_X;                      /* '<Root>/ACCEL_X' */
  real_T ACCEL_Y;                      /* '<Root>/ACCEL_Y' */
} ExtU_vel_filter_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T filtered_velocity_x;          /* '<Root>/filtered_velocity_x' */
  real_T filtered_location_x;          /* '<Root>/filtered_location_x' */
  real_T speed;                        /* '<Root>/speed' */
  real_T filtered_velocity_y;          /* '<Root>/filtered_velocity_y' */
  real_T filtered_location_y;          /* '<Root>/filtered_location_y' */
} ExtY_vel_filter_T;

/* Real-time Model Data Structure */
struct tag_RTM_vel_filter_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_vel_filter_T vel_filter_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_vel_filter_T vel_filter_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_vel_filter_T vel_filter_Y;

/* Model entry point functions */
extern void vel_filter_initialize(void);
extern ExtY_vel_filter_T vel_filter_step(float, float);
extern void vel_filter_terminate(void);

/* Real-time Model object */
extern RT_MODEL_vel_filter_T *const vel_filter_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Scope10' : Unused code path elimination
 * Block '<Root>/Scope11' : Unused code path elimination
 * Block '<Root>/Scope9' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'vel_filter'
 * '<S1>'   : 'vel_filter/MATLAB Function7'
 * '<S2>'   : 'vel_filter/MATLAB Function8'
 */
#endif                                 /* vel_filter_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
