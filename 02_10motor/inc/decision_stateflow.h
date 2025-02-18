/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: decision_stateflow.h
 *
 * Code generated for Simulink model 'decision_stateflow'.
 *
 * Model version                  : 1.3
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Thu Feb 13 12:39:53 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef decision_stateflow_h_
#define decision_stateflow_h_
#ifndef decision_stateflow_COMMON_INCLUDES_
#define decision_stateflow_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* decision_stateflow_COMMON_INCLUDES_ */

#include "decision_stateflow_types.h"
#include "Ourcan.h"
#include "EncMotor.h"
#include "PID_CON.h"
#include "gitstanley.h"
#include "STM_Interrupt.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#define decision_sta_IN_NO_ACTIVE_CHILD   ((uint8_T)0U)
#define decision_stateflow_IN_DRIVER_Mode   ((uint8_T)1U)
#define decision_stateflow_IN_DRIVER_D    ((uint8_T)1U)
#define decision_stateflow_IN_DRIVER_P    ((uint8_T)2U)
#define decision_stateflow_IN_DRIVER_R    ((uint8_T)3U)

#define decision_stateflow_IN_INIT_Mode   ((uint8_T)2U)

#define decision_stateflow_IN_RSPA_Mode        ((uint8_T)3U)
#define decision_stateflow_IN_RSPA_D           ((uint8_T)1U)
#define decision_stateflow_IN_RSPA_R           ((uint8_T)2U)
#define decision_stateflow_IN_RSPA_P           ((uint8_T)3U)
#define decision_stateflow_IN_RSPA_LANE_D      ((uint8_T)4U)
#define decision_stateflow_IN_RSPA_LANE_R      ((uint8_T)5U)
#define decision_stateflow_IN_RSPA_IS_SLOT     ((uint8_T)6U)


#define decision_stateflow_IN_SAFE_FCA           ((uint8_T)4U)
#define decision_stateflow_IN_FCA_EMERGENCY      ((uint8_T)1U)
#define decision_stateflow_IN_FCA_DECEL          ((uint8_T)2U)
#define decision_stateflow_IN_FCA_EXIT           ((uint8_T)3U)

#define decision_stateflow_IN_SAFE_RCA           ((uint8_T)5U)
#define decision_stateflow_IN_RCA_EMERGENCY      ((uint8_T)1U)
#define decision_stateflow_IN_RCA_DECEL          ((uint8_T)2U)
#define decision_stateflow_IN_RCA_EXIT           ((uint8_T)3U)

#define decision_stateflow_IN_FIND_CAR             ((uint8_T)6U)



/* Block states (default storage) for system '<Root>' */
typedef struct {
  uint8_T is_active_c3_decision_stateflow;/* '<Root>/decision' */
  uint8_T is_c3_decision_stateflow;    /* '<Root>/decision' */
  uint8_T is_DRIVER_Mode;              /* '<Root>/decision' */
  uint8_T is_RSPA_Mode;                /* '<Root>/decision' */
  uint8_T is_SAFE_RCA;
  uint8_T is_SAFE_FCA;
  uint8_T is_FIND_CAR;
} DW_decision_stateflow_T;

typedef enum{
  InitDriverState = 0,
  Parking,    // 1
  Driving,    // 2
  Reversing   // 3
}DriverState;

typedef enum{
  InitCAState = 0,
  Emergency,  // 1
  Decel       // 2
}CAState;

typedef enum{
  InitRSPAState = 0,
  Searching,  // 1
  Forward, // 2
  Backward,  // 3
  Backward_Assist, // 4
  Forward_Assist, // 5
  Parking_Complete // 6  
}RSPAState;

typedef enum{
  InitIsPrk = 0,
  LEFT,
  RIGHT
}IsPrk;

/* Real-time Model Data Structure */
struct tag_RTM_decision_stateflow_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */

extern double initVel;
extern Transmission U8IsTrButton;
extern double U8Curr_vel;
extern double U8Ref_vel;
extern double DInputVD;
extern double DInputVR;
extern int IsRSPAButton;
extern int IsOTAFinished;
extern int U8IsWp_R;
extern int U8IsStopline;
extern int U8IsPrkFinished;
extern int U8IsOb_R;
extern DriverState U8DriverState;
extern RSPAState U8RSPAState;
extern int U8Driver;
extern int U8RSPA;
extern int U8Engine;
extern CAState U8FCAState;
extern CAState U8RCAState;
extern double DTTC_D; // �썑諛� �옣�븷臾� ttc
extern double DTTC_R; // �쟾諛� �옣�븷臾� ttc
extern double DObs_dis_D; //�쟾諛� �옣�븷臾� �긽��嫄곕━
extern double DObs_dis_R; //�썑諛� �옣�븷臾� �긽��嫄곕━
extern sint8 DSteeringinput;
extern int calDis;
extern int U8PrkFinished;
extern int ExitCAR_request;
extern double D_Ref_vel;
extern int CameraSwitchRequest;
extern int First_Set;
extern int lanecheck_request;
extern IsPrk IsPrk_LR;
extern int U8IsConerline;
/* Model entry point functions */
extern void decision_stateflow_initialize(void);
extern void decision_stateflow_step(void);
extern void decision_stateflow_terminate(void);

/* Real-time Model object */
extern RT_MODEL_decision_stateflow_T *const decision_stateflow_M;
extern DW_decision_stateflow_T decision_stateflow_DW;
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
 * '<Root>' : 'decision_stateflow'
 * '<S1>'   : 'decision_stateflow/decision'
 */
#endif                                 /* decision_stateflow_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
