/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.
 * Not for government, commercial, or other organizational use.
 *
 * File: decision_stateflow.c
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

#include "decision_stateflow.h"
#include "rtwtypes.h"


/* Block states (default storage) */
DW_decision_stateflow_T decision_stateflow_DW;

/* Real-time model */
RT_MODEL_decision_stateflow_T decision_stateflow_M_;
RT_MODEL_decision_stateflow_T *const decision_stateflow_M = &decision_stateflow_M_;

/*variabls*/
int ModeOff = 0;
int ModeOn = 1;
char initState = '0';
int initVel = 0;
int U8IsEngineButton = 0;
int U8IsTrButton = 0;
double U8Curr_vel = 0;
double U8Ref_vel = 0;
double DInputVD = 0.1;  //전진주차 속도
double DInputVR = 0.1;   //후진주차 속도
int IsRSPAButton = 0;
int IsOTAFinished = 0;
int U8IsWp_R = 0;
int U8IsStopline = 0;
int U8IsPrkFinished = 0;
int U8IsOb_R = 0;
char U8DriverState = '0';
char U8RSPAState = '0';
int U8Driver = 0;
int U8RSPA = 0;
int U8Engine = 0;



/* Model step function */
void decision_stateflow_step(void)
{
    /* Chart: '<Root>/decision' */
    if (decision_stateflow_DW.is_active_c3_decision_stateflow == 0)
    {
        decision_stateflow_DW.is_active_c3_decision_stateflow = 1U;
        decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_INIT_Mode;
    }
    else
    {
        switch (decision_stateflow_DW.is_c3_decision_stateflow)
        {
            case decision_stateflow_IN_INIT_Mode:
                U8Driver=ModeOff;
                U8RSPA=ModeOff;
                U8Engine=ModeOff;
                U8DriverState=initState;
                U8RSPAState=initState;

                if (U8IsEngineButton == 1)
                {
                    U8Engine=ModeOn;
                    decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_DRIVER_Mode;
                    decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_P;

                }
                break;

            case decision_stateflow_IN_DRIVER_Mode:
                U8Driver=ModeOn;
                U8RSPA=ModeOff;
                //U8Engine=ModeOff;
                U8DriverState=initState;
                U8RSPAState=initState;

                switch (decision_stateflow_DW.is_DRIVER_Mode)
                {
                    case decision_stateflow_IN_DRIVER_P:
                        U8DriverState = 'P';
                        U8Ref_vel = initVel;

                        if (U8IsTrButton == 1 && U8Curr_vel == 0)
                        {
                            decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_D;
                        }
                        else if(U8IsEngineButton==0){  //시동 off
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_INIT_Mode;
                            }
                        }
                        break;

                    case decision_stateflow_IN_DRIVER_D:
                        U8DriverState = 'D';
                        U8Ref_vel = DInputVD;

                        if (U8IsTrButton == 0)
                        {
                            U8Ref_vel = 0;

                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_P;
                            }

                        }
                        else if (U8IsTrButton == 2)
                        {
                            U8Ref_vel=0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_R;
                            }
                        }
                        else if (IsRSPAButton == 1 && IsOTAFinished == 1){
                            U8Ref_vel=0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_D;
                                U8DriverState=initState;
                            }
                        }
                        break;

                    case decision_stateflow_IN_DRIVER_R:
                        U8DriverState = 'R';
                        U8Ref_vel = DInputVR;

                        if (U8IsTrButton == 1)
                        {
                            U8Ref_vel=0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_D;
                            }
                        }
                        else if(U8IsTrButton == 0){
                            U8Ref_vel=0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_P;
                            }
                        }

                        else if (IsRSPAButton == 1 && IsOTAFinished == 1)
                        {
                            U8Ref_vel=0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_D;
                                U8DriverState=initState;
                            }
                        }
                        break;
                }
                break;

            //주차보조시스템
            case decision_stateflow_IN_RSPA_Mode:
                U8Driver=ModeOff;
                U8RSPA=ModeOn;
                U8DriverState=initState;
                U8RSPAState=initState;

                switch (decision_stateflow_DW.is_RSPA_Mode)
                {
                    default:
                        decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_D;
                        break;

                    case decision_stateflow_IN_RSPA_D:
                        U8RSPAState='D';
                        U8Ref_vel=DInputVD;

                        if (U8IsWp_R == 1)
                        {
                            U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_R;
                            }
                        }
                        else if(U8IsStopline==1 && U8IsPrkFinished==1){
                            U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_P;
                            }
                        }
                        break;

                    case decision_stateflow_IN_RSPA_R:
                        U8RSPAState='P';
                        U8Ref_vel=DInputVD;

                        if (U8IsStopline == 1 && U8IsPrkFinished == 1)
                        {
                            U8Ref_vel = 0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_P;
                            }
                        }
                        else if (U8IsOb_R == 1 || (U8IsPrkFinished == 1 && U8IsStopline == 0)){
                            U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_D;
                            }
                        }
                        break;

                    case decision_stateflow_IN_RSPA_P:
                        U8RSPAState='P';
                        U8Ref_vel = initVel;

                        if (U8IsEngineButton==0){
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_INIT_Mode;
                            }
                        }

                        break;
                }
                break;

        }
    }
}



/* Model initialize function */
void decision_stateflow_initialize(void)
{
    /* (no initialization code required) */
}

/* Model terminate function */
void decision_stateflow_terminate(void)
{
    /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
