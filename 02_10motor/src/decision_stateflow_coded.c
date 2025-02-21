/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree gKnting institutions only.
 * Not for government, commercial, or other organizational use.
 *
 * File: decision_stateflow.c
 *
 * Code geneKted for Simulink model 'decision_stateflow'.
 *
 * Model version                  : 1.3
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code geneKted on : Thu Feb 13 12:39:53 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code geneKtion objectives: Unspecified
 * ValiJtion result: Not run
 */

#include "decision_stateflow.h"

#include "rtwtypes.h"
#include "IfxGpt12_IncrEnc.h"


/* Block states (default stoKge) */
DW_decision_stateflow_T decision_stateflow_DW;

/* Real-time model */
RT_MODEL_decision_stateflow_T decision_stateflow_M_;
RT_MODEL_decision_stateflow_T *const decision_stateflow_M = &decision_stateflow_M_;

/*variabls*/
int ModeOff = 0;
int ModeOn = 1;
//char initState = '0';
double initVel = 0;
Transmission U8IsTrButton = 0;
double U8Ref_vel = 0;
double DInputVD = 0.1;
double DInputVR = -0.1;
int IsRSPAButton = 0;
int U8IsWp_R = 0;
int U8IsStopline = 0;
int U8IsPrkFinished = 0;
DriverState U8DriverState = InitDriverState;
RSPAState U8RSPAState = InitRSPAState;
int U8Driver = 0;
int U8RSPA = 0;
int U8Engine = 0;
int U8Parkingfail=0;
int U8PrkFinished=0;
int ExitCAR_request=0;
double D_Ref_vel=0;
int lanecheck_request=0;

int CameraSwitchRequest=0;
int Isprkslot;
sint8 DSteeringinput=0;
double DMoveDis=0;
int calDis=0;
int First_Set = 1;
int md_flag=0;

IsPrk IsPrk_LR = InitIsPrk;

int U8IsConerline = 0;
ConerState U8ConerState=InitConering;

CAState U8FCAState = InitCAState;
CAState U8RCAState = InitCAState;
double gainTTC=0.0;

int ToController_Prkstate=0;
int ToController_Exitstate=0;



/* Model step function */
void decision_stateflow_step_c(void)
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
                U8DriverState = InitDriverState;
                U8RSPAState = InitRSPAState;
                U8FCAState = InitCAState;
                U8RCAState = InitCAState;
                ToController_Prkstate=0;
                ToController_Exitstate=0;


                if (U8IsTrButton == ENGINE_ON && ExitCAR_request == 0 )
                {
                    U8Engine=ModeOn;

                    decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_DRIVER_Mode;
                    decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_P;


                }

                if (vehicle_status.engine_state == ENGINE_OFF && ExitCAR_request==1){
                    decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_FIND_CAR;
                }

                if (IsRSPAButton == 1){
                    D_RefRPM=0;
                    if (U8Curr_vel==0){
                        decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                        decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_IS_SLOT;

                        U8DriverState = InitDriverState;
                        First_Set=1;
                        IsRSPAButton = 0;
                    }
                }

                break;

            case decision_stateflow_IN_FIND_CAR:
                ToController_Prkstate=3;
                U8Driver=ModeOff;
                U8RSPA=ModeOff;
                U8Engine=ModeOff;
                U8DriverState = InitDriverState;
                U8RSPAState = Parking_Complete;
                U8FCAState = InitCAState;
                U8RCAState = InitCAState;
                ToController_Exitstate=1;

                if (ExitCAR_request==1){
                    U8Ref_vel = DInputVD;
                    if (move_distance(200) == REACHED_TARGET_DIS)
                    {
                        U8Ref_vel=0;
                        ExitCAR_request=0;
                        U8PrkFinished=0;
                        U8Parkingfail=0;

                        decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_DRIVER_Mode;
                        decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_P;
                        ToController_Exitstate=0;
                      }
                }
                break;


            case decision_stateflow_IN_SAFE_FCA:

                switch (decision_stateflow_DW.is_SAFE_FCA)
                {
                    case decision_stateflow_IN_FCA_EMERGENCY:
                        U8FCAState = Emergency;
                        U8Ref_vel = initVel;

                        if (U8Curr_vel == 0 && ((double)obstacle[F_OBSTACLE]/1000) == 0)
                        {
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EXIT;
                        }

                        break;
                    case decision_stateflow_IN_FCA_DECEL:
                        U8FCAState = Decel;
                        U8Ref_vel = U8Ref_vel-U8Ref_vel*(1/(Cal_TTCD(U8Curr_vel)+gainTTC));

                        if (((double)obstacle[F_OBSTACLE]/1000) == 0)
                        {
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EXIT;
                        }
                        break;

                    case decision_stateflow_IN_FCA_EXIT:
                        U8FCAState = InitCAState;

                        if (U8DriverState == Driving)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_DRIVER_Mode;
                            decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_D;
                        }
                        else if(U8RSPAState == Forward)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_D;
                        }
                        else if(U8RSPAState == Forward_Assist)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_D;
                        }
                        else if(U8RSPAState == Searching)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_IS_SLOT;
                            First_Set=1;
                        }
                        break;
                }
                break;

            case decision_stateflow_IN_SAFE_RCA:

                switch (decision_stateflow_DW.is_SAFE_RCA){
                    case decision_stateflow_IN_RCA_EMERGENCY:
                        U8RCAState = Emergency;
                        U8Ref_vel = initVel;

                        if (U8Curr_vel == 0 && ((double)obstacle[B_OBSTACLE]/1000) == 0)
                        {
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EXIT;
                        }

                        break;
                    case decision_stateflow_IN_RCA_DECEL:
                        U8RCAState = Decel;
                        U8Ref_vel = U8Ref_vel-U8Ref_vel*(1/(Cal_TTCR(U8Curr_vel)+gainTTC));

                        if (((double)obstacle[B_OBSTACLE]/1000)==0)
                        {
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EXIT;
                        }
                        break;
                    case decision_stateflow_IN_RCA_EXIT:
                        U8RCAState = InitCAState;

                        if (U8DriverState == Reversing)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_DRIVER_Mode;
                            decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_R;
                        }
                        else if(U8RSPAState == Backward){
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_R;
                        }
                        else if(U8RSPAState == Backward_Assist){
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_R;
                            First_Set=1;
                            //CameraSwitchRequest=2;
                        }
                        break;
                }
                break;

            case decision_stateflow_IN_DRIVER_Mode:
                U8Driver=ModeOn;
                U8RSPA=ModeOff;
                ToController_Prkstate=0;
                //U8Engine=ModeOff;

                U8DriverState = InitDriverState;
                U8RSPAState = InitRSPAState;

                switch (decision_stateflow_DW.is_DRIVER_Mode)
                {
                    case decision_stateflow_IN_DRIVER_P:
                        U8DriverState = Parking;
                        U8Ref_vel = initVel;

                        if (U8IsTrButton== DRIVING && U8Curr_vel == 0)
                        {
                            decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_D;
                        }
                        else if(vehicle_status.engine_state == ENGINE_OFF){
                            D_RefRPM = 0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_INIT_Mode;
                            }
                        }
                        break;

                    case decision_stateflow_IN_DRIVER_D:
                        U8DriverState = Driving;
                        U8Ref_vel = D_Ref_vel;

//
//                        if(Cal_TTCD(U8Curr_vel) > 0 && Cal_TTCD(U8Curr_vel) <= 1.0)
//                        {
//                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
//                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EMERGENCY;
//                        }
//                        else if(Cal_TTCD(U8Curr_vel) < 3.0)
//                        {
//                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
//                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_DECEL;
//                        }

                        if (U8IsTrButton == PARKING)
                        {
                            D_RefRPM=0;
                            if (U8Curr_vel == 0){
                                decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_P;
                            }

                        }
                        else if (U8IsTrButton == REVERSE)
                        {
                            D_RefRPM=0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_R;
                            }
                        }
                        else if (IsRSPAButton == 1){
                            D_RefRPM=0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;

                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_IS_SLOT;

                                U8DriverState = InitDriverState;
                                First_Set=1;
                                IsRSPAButton = 0;
                                CameraSwitchRequest = 1;
                                
                            }
                        }
                        break;

                    case decision_stateflow_IN_DRIVER_R:
                        U8DriverState = Reversing;
                        U8Ref_vel = D_Ref_vel;


//                        if(Cal_TTCD(U8Curr_vel) > 0 && Cal_TTCD(U8Curr_vel) <= 1.0)
//                        {
//                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
//                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EMERGENCY;
//
//                        }
//                        else if(Cal_TTCR(U8Curr_vel) < 3.0)
//                        {
//                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
//                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_DECEL;
//
//                        }

                        if (U8IsTrButton == DRIVING)
                        {
                            D_RefRPM=0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_D;
                            }
                        }
                        else if(U8IsTrButton == PARKING){
                            D_RefRPM=0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_P;
                            }
                        }

                        else if (IsRSPAButton == 1)
                        {
                            D_RefRPM=0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_IS_SLOT;

                                U8DriverState = InitDriverState;
                                First_Set=1;
                                IsRSPAButton = 0;
                                CameraSwitchRequest = 1;
                            }
                        }
                        break;
                }
                break;


            case decision_stateflow_IN_RSPA_Mode:
                U8Driver=ModeOff;
                U8RSPA=ModeOn;
                U8DriverState=InitDriverState;
                U8RSPAState=InitRSPAState;

                switch (decision_stateflow_DW.is_RSPA_Mode)
                {
                    case decision_stateflow_IN_RSPA_IS_SLOT:
                        U8RSPAState= Searching;
                        U8Ref_vel=0.15;   //DInputVD
                        ToController_Prkstate=0;


//                        if(Cal_TTCD(U8Curr_vel) > 0 && Cal_TTCD(U8Curr_vel) <= 1.0)
//                        {
//                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
//                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EMERGENCY;
//
//                        }
//                        else if(Cal_TTCD(U8Curr_vel) < 3.0)
//                        {
//                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
//                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_DECEL;
//
//                        }

                        if ( detecting_spot[R_ULTRA] == 1 )
                        {
                            U8Ref_vel = 0;

                            if (U8Curr_vel==0){
                               IsPrk_LR=RIGHT;
                               decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_D;
                            }
                        }



                        break;

                    case decision_stateflow_IN_RSPA_D:
                        U8RSPAState=Forward;
                        U8Ref_vel=DInputVD;
                        ToController_Prkstate=1;
//
//                        if(Cal_TTCD(U8Curr_vel) > 0 && Cal_TTCD(U8Curr_vel) <= 1.0)
//                        {
//                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
//                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EMERGENCY;
//
//                        }
//                        else if(Cal_TTCD(U8Curr_vel) < 3.0)
//                        {
//                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
//                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_DECEL;
//
//                        }


                        if(md_flag==0){
                            if(move_distance(320) == REACHED_TARGET_DIS) //100mm
                            {
                                DInputVD= 0;
                                md_flag=1;
                            }
                        }
                        else if(md_flag==1) {
                            md_flag++;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_R;
                        }
                        else if (md_flag==4){
                             if(move_distance(180) == REACHED_TARGET_DIS){
                                DInputVD= 0;
                                md_flag=5;
                             }
                        }
                        else if(md_flag==5) {
                            md_flag++;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_R;
                        }
                        else if (md_flag==8){
                            if(move_distance(-250) == REACHED_TARGET_DIS){
                                DInputVD= 0;
                                md_flag=9;
                            }
                        }
                        else if(md_flag==9) {
                            md_flag++;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_P;
                        }

                        break;

                    case decision_stateflow_IN_RSPA_LANE_D:
                        U8RSPAState=Forward_Assist;
                        ToController_Prkstate=1;
                        U8Ref_vel=DInputVD;
////                        if(Cal_TTCD(U8Curr_vel) > 0 && Cal_TTCD(U8Curr_vel) <= 1.0)
////                        {
////                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
////                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EMERGENCY;
////
////                        }
////                        else if(Cal_TTCD(U8Curr_vel) < 3.0)
////                        {
////                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
////                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_DECEL;
////
////                        }
//
//                        if(move_distance(300) == REACHED_TARGET_DIS) //100mm
//                        {
//                            DInputVD= 0;
//
//                            if (U8Curr_vel==0){
//                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_R;
//                                //CameraSwitchRequest = 2;
//                            }
//                        }

                        break;

                    case decision_stateflow_IN_RSPA_R:
                        U8RSPAState=Backward;
                        ToController_Prkstate=2;
                        U8Ref_vel=DInputVR;

//

//                        if(Cal_TTCD(U8Curr_vel) > 0 && Cal_TTCD(U8Curr_vel) <= 1.0)
//                        {
//                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
//                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EMERGENCY;
//
//                        }
//                        else if(Cal_TTCR(U8Curr_vel) < 3.0)
//                        {
//                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
//                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_DECEL;
//
//                        }

                        if(md_flag==2){
                            if(move_distance(-310) == REACHED_TARGET_DIS) //100mm
                            {
                                DInputVR= 0;
                                md_flag=3;
                            }
                        }
                        else if(md_flag==3) {
                            md_flag++;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_D;
                        }
                        else if (md_flag==6){
                             if(move_distance(-200) == REACHED_TARGET_DIS){
                                DInputVR= 0;
                                md_flag=7;
                                //CameraSwitchRequest = 2;
                             }
                        }
                        else if(md_flag==7) {
                            md_flag++;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_D;
                        }
                        break;

                    case decision_stateflow_IN_RSPA_LANE_R:
                        U8RSPAState=Backward_Assist;
                        ToController_Prkstate=2;
                        U8Ref_vel=DInputVR;
                        U8Parkingfail=0;

//                        if(Cal_TTCD(U8Curr_vel) > 0 && Cal_TTCD(U8Curr_vel) <= 1.0)
//                        {
//                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
//                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EMERGENCY;
//
//                        }
//                        else if(Cal_TTCR(U8Curr_vel) < 3.0)
//                        {
//                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
//                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_DECEL;
//
//                        }

                        if (U8IsStopline == 1 && IsWPTrackingFinish == 1)
                        {
                            U8Ref_vel = 0;
                            if (U8Curr_vel==0){

                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_P;
                            }
                        }
                        else if (IsWPTrackingFinish == 1 && U8IsStopline == 0){
                            U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                U8Parkingfail=1;
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_D;
                            }
                        }

                        /*else if (obstacle[B_OBSTACLE] == 0 ){
                             U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                U8Parkingfail=1;
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_D;
                            }
                        }*/
                        break;

                    case decision_stateflow_IN_RSPA_P:
                        U8RSPAState = Parking_Complete;
                        U8Ref_vel = initVel;
                        U8IsStopline=0;
                        lanecheck_request=0;
                        ToController_Prkstate=3;

                        if (vehicle_status.engine_state == ENGINE_OFF){
                            if (U8Curr_vel==0){
                                U8PrkFinished = 1;
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
 * File tKiler for geneKted code.
 *
 * [EOF]
 */
