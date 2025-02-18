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
double DInputVD = 0.1;  //
double DInputVR = -0.1;   //
int IsRSPAButton = 0;
int IsOTAFinished = 0;
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
sint8 DSteeringinput=0;
int First_Set = 1; //占쏙옙占쏙옙占싸쏙옙 占쏙옙占쏙옙
int U8IsConerline=0;

IsPrk IsPrk_LR = InitIsPrk;

//占쏙옙占쏙옙
CAState U8FCAState = InitCAState;
CAState U8RCAState = InitCAState;
double DObs_dis_D= 100.0; //占쏙옙占쏙옙 占쏙옙岺占� 占쏙옙占신몌옙 占쏙옙占쏙옙 占쏙옙占쏙옙占싶곤옙 占쏙옙占쏙옙占쏙옙 占쏙옙占쏙옙 占쏙옙, state 占쏙옙占쏙옙 占쏙옙占쏙옙 占쏙옙占쏙옙占쏙옙 占십깍옙화
double DObs_dis_R= 100.0; //占식뱄옙 占쏙옙岺占� 占쏙옙占신몌옙 占쏙옙占쏙옙 占쏙옙占쏙옙占싶곤옙 占쏙옙占쏙옙占쏙옙 占쏙옙占쏙옙 占쏙옙, state 占쏙옙占쏙옙 占쏙옙占쏙옙 占쏙옙占쏙옙占쏙옙 占십깍옙화
double gainTTC=0.0; //튜占쏙옙 占식띰옙占쏙옙占�


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
                U8DriverState = InitDriverState;
                U8RSPAState = InitRSPAState;

                if (vehicle_status.engine_state == ENGINE_ON && ExitCAR_request == 0 )   //占시듸옙 占쏙옙占쏙옙占쏙옙 占쏙옙占쏙옙占쌘몌옙占쏙옙 占쏙옙환
                {
                    U8Engine=ModeOn;

                    decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_DRIVER_Mode;
                    decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_P;


                }

                if (vehicle_status.engine_state == ENGINE_OFF && ExitCAR_request==1){   //占쏙옙占쏙옙占쏙옙청
                    decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_FIND_CAR;
                }

                break;

            case decision_stateflow_IN_FIND_CAR:

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
                      }
                }
                break;


            case decision_stateflow_IN_SAFE_FCA:

                switch (decision_stateflow_DW.is_SAFE_FCA)
                {
                    case decision_stateflow_IN_FCA_EMERGENCY:
                        U8FCAState = Emergency;
                        U8Ref_vel = initVel;

                        if (U8Curr_vel == 0 && ((double)obstacle[F_OBSTACLE]/1000) == (double)FBOBSTACLE_WARNING)
                        {
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EXIT;
                        }

                        break;
                    case decision_stateflow_IN_FCA_DECEL:
                        U8FCAState = Decel;
                        if (U8Curr_vel !=0){
                            U8Ref_vel=0;
                        }
                        else{
                            U8Ref_vel = U8Ref_vel-U8Ref_vel*(1/(Cal_TTCD(U8Curr_vel)+gainTTC));
                        }

                        if (((double)obstacle[F_OBSTACLE]/1000) == (double)FBOBSTACLE_WARNING)
                        {
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EXIT;
                        }
                        break;
                    case decision_stateflow_IN_FCA_EXIT:   //占쏙옙占쏙옙占쏙옙占� 占싹깍옙 占쏙옙占쏙옙 MODE 占쏙옙 占쏙옙占쏙옙
                                                           //탈占쏙옙 占쏙옙占쏙옙 DObs_dis_D 占쏙옙占쏙옙占싹댐옙占쏙옙 확占쏙옙
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
                            //CameraSwitchRequest=1;
                        }
                        break;
                }
                break;

            case decision_stateflow_IN_SAFE_RCA:

                switch (decision_stateflow_DW.is_SAFE_RCA){
                    case decision_stateflow_IN_RCA_EMERGENCY:
                        U8RCAState = Emergency;
                        U8Ref_vel = initVel;

                        if (U8Curr_vel == 0 && ((double)obstacle[B_OBSTACLE]/1000) == (double)RLOBSTACLE_WARNING) //占쏙옙占쏙옙 占쏙옙占쏙옙 + 占식뱄옙 占쏙옙岺占� 占쏙옙占쏙옙占�
                        {
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EXIT;
                        }

                        break;
                    case decision_stateflow_IN_RCA_DECEL:
                        U8RCAState = Decel;
                        if (U8Curr_vel !=0){
                            U8Ref_vel=0;
                        }
                        else{
                            U8Ref_vel = U8Ref_vel-U8Ref_vel*(1/(Cal_TTCR(U8Curr_vel)+gainTTC));
                        }

                        if (((double)obstacle[B_OBSTACLE]/1000) == (double)RLOBSTACLE_WARNING)
                        {
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EXIT;
                        }
                        break;
                    case decision_stateflow_IN_RCA_EXIT:   //占쏙옙占쏙옙占쏙옙占� 占싹깍옙 占쏙옙占쏙옙 MODE 占쏙옙 占쏙옙占쏙옙
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
                //U8Engine=ModeOff;

                U8DriverState = InitDriverState;
                U8RSPAState = InitRSPAState;

                switch (decision_stateflow_DW.is_DRIVER_Mode)
                {
                    case decision_stateflow_IN_DRIVER_P:
                        U8DriverState = Parking;
                        U8Ref_vel = initVel;

                        if (vehicle_status.transmission == DRIVING && U8Curr_vel == 0)
                        {
                            decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_D;
                        }
                        else if(vehicle_status.engine_state == ENGINE_OFF){  //占시듸옙 off
                            D_RefRPM = 0;
                            if (U8Curr_vel==0){
                                //U8Ref_vel=(0);
                                decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_INIT_Mode;
                            }
                        }
                        break;

                    case decision_stateflow_IN_DRIVER_D:
                        U8DriverState = Driving;
                        U8Ref_vel = D_Ref_vel;

                        //占쏙옙占쏙옙占쏙옙占�
                        if(Cal_TTCD(U8Curr_vel) <= 1.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EMERGENCY;
                        }
                        else if(Cal_TTCD(U8Curr_vel) < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_DECEL;
                        }
                        //占쏙옙咀�占쏙옙
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
                        else if (IsRSPAButton == 1 && IsOTAFinished == 1){
                            D_RefRPM=0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;

                                //占쌩곤옙占쏙옙 占싸븝옙 占쏙옙占쏙옙
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_IS_SLOT;

                                U8DriverState = InitDriverState;
                                First_Set=1;
                                IsRSPAButton = 0;
                                //VCU 占쏙옙占쏙옙 占쏙옙占쏙옙占쏙옙占쏙옙占쏙옙占쏙옙 占쏙옙占쏙옙카占쌨띰옙 on 占쌔쇽옙 waypoint 占쏙옙占쏙옙占쌨띰옙占� 占쌔억옙占쏙옙 (msg 占쌜쏙옙)
                                //{
                                CameraSwitchRequest = 1;  // 카占쌨띰옙 占쏙옙환 占쏙옙청 占시뤄옙占쏙옙(占쏙옙占쏙옙카占쌨띰옙 on) (make_can_message占쏙옙占쏙옙 처占쏙옙)
                                //}
                                //占쌩곤옙占쏙옙 占싸븝옙 占쏙옙
                                
                            }
                        }
                        break;

                    case decision_stateflow_IN_DRIVER_R:
                        U8DriverState = Reversing;
                        U8Ref_vel = D_Ref_vel;

                        //占쏙옙占쏙옙占쏙옙占�
                        if(Cal_TTCR(U8Curr_vel) <= 1.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EMERGENCY;

                        }
                        else if(Cal_TTCR(U8Curr_vel) < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_DECEL;

                        }
                        //占쏙옙咀�占쏙옙
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

                        else if (IsRSPAButton == 1 && IsOTAFinished == 1)
                        {
                            D_RefRPM=0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_IS_SLOT;

                                U8DriverState = InitDriverState;

                                First_Set=1;
                                IsRSPAButton = 0; // 占쌩곤옙
                                //VCU 占쏙옙占쏙옙 占쏙옙占쏙옙占쏙옙占쏙옙占쏙옙占쏙옙 占쏙옙占쏙옙카占쌨띰옙 on 占쌔쇽옙 waypoint 占쏙옙占쏙옙占쌨띰옙占� 占쌔억옙占쏙옙 (msg 占쌜쏙옙)
                                //{
                                CameraSwitchRequest = 1;  // 카占쌨띰옙 占쏙옙환 占쏙옙청 占시뤄옙占쏙옙(占쏙옙占쏙옙카占쌨띰옙 on) (make_can_message占쏙옙占쏙옙 처占쏙옙)
                                //}
                            }
                        }
                        break;
                }
                break;

            //占쏙옙占쏙옙占쏙옙占쏙옙占시쏙옙占쏙옙
            case decision_stateflow_IN_RSPA_Mode:
                U8Driver=ModeOff;
                U8RSPA=ModeOn;

                U8DriverState=InitDriverState;
                U8RSPAState=InitRSPAState;

                switch (decision_stateflow_DW.is_RSPA_Mode)
                {
                    case decision_stateflow_IN_RSPA_IS_SLOT:
                        U8RSPAState= Searching;
                        U8Ref_vel=DInputVD;

                        //占쏙옙占쏙옙占쏙옙占�
                        if(Cal_TTCD(U8Curr_vel) <= 1.0)    //ttc 占쏙옙占쏙옙求占� 占쏙옙占쏙옙 占쏙옙占쏙옙 占쏙옙짬
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EMERGENCY;

                        }
                        else if(Cal_TTCD(U8Curr_vel) < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_DECEL;

                        }

                        if ((parking_spot[R_ULTRA] == 1) || (parking_spot[L_ULTRA] == 1) )   // 占쏙옙占쏙옙占쏙옙칸 찾占쏙옙占쏙옙 STATE 占쏙옙占쏙옙
                        {
                            U8Ref_vel = 0;

                            if (parking_spot[R_ULTRA] == 1){
                                IsPrk_LR = RIGHT;
                            }
                            else if(parking_spot[L_ULTRA] == 1) {
                                IsPrk_LR = LEFT;
                                }

                            //
                            if (U8Curr_vel==0){

                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_D;

                            }
                        }



                        break;

                    case decision_stateflow_IN_RSPA_D:
                        U8RSPAState=Forward;
                        U8Ref_vel=DInputVD;

                        //占쏙옙占쏙옙占쏙옙占�
                        if(Cal_TTCD(U8Curr_vel) <= 1.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EMERGENCY;

                        }
                        else if(Cal_TTCD(U8Curr_vel) < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_DECEL;

                        }

                        //if (DMoveDis == 0.10)  // 10CM 占쏙옙占쏙옙占쏙옙占쏙옙 占싱듸옙占쏙옙占쏙옙占쏙옙
                        if(move_distance(100) == REACHED_TARGET_DIS) //100mm
                        {
                            DInputVD= 0;
                            
                            if (U8Curr_vel==0){
                                // 占쏙옙占쏙옙占쏙옙占쏙옙 占쏙옙占쏙옙 占싹뤄옙풔占� 占쏙옙占쏙옙 占시곤옙 占쌩곤옙??
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_R;
                                //占쌩곤옙占쏙옙 占싸븝옙
                                CameraSwitchRequest = 2; // 占식뱄옙 카占쌨띰옙
                                //占쌩곤옙占쏙옙 占싸븝옙 占쏙옙
                            }
                        }
                        break;

                    case decision_stateflow_IN_RSPA_LANE_D:
                        U8RSPAState=Forward_Assist;
                        U8Ref_vel=DInputVD;
                        //占쏙옙占쏙옙占쏙옙占�
                        if(Cal_TTCD(U8Curr_vel) <= 1.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EMERGENCY;

                        }
                        else if(Cal_TTCD(U8Curr_vel) < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_DECEL;

                        }

                        if (U8Parkingfail==1 )  //占싼뱄옙占쏙옙 占쏙옙占쏙옙 占쏙옙占쌔쇽옙 占쌩깍옙薩占�, STEERING 占쏙옙 占쏙옙占쏙옙 占쏙옙占쏙옙莩占쏙옙 * (-1)
                        {
                            DInputVD= 0;
                            if (U8Curr_vel==0){

                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_R;
                            }
                        }
                        else if(U8IsStopline==1 && U8IsPrkFinished==1){
                            DInputVD= 0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_P;
                            }
                        }
                        break;

                    case decision_stateflow_IN_RSPA_R:  //steering 占쏙옙 3占쏙옙 占쏙옙占쏙옙
                        U8RSPAState=Backward;
                        U8Ref_vel=DInputVR;

                        move_distance(700);

                        //占쏙옙占쏙옙占쏙옙占�
                        if(Cal_TTCR(U8Curr_vel) <= 1.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EMERGENCY;

                        }
                        else if(Cal_TTCR(U8Curr_vel) < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_DECEL;

                        }

                        if (CameraSwitchRequest == 2)  // 占식뱄옙카占쌨띰옙 占쏙옙占쏙옙 占쏙옙占쏙옙占쏙옙占쏙옙占쏙옙 WP 占쌨억옙占쏙옙占쏙옙
                        {
                            U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_R; //占쏙옙占쏙옙
                            }
                        }
                        break;

                    case decision_stateflow_IN_RSPA_LANE_R:  //占싱띰옙占쏙옙 占쏙옙占쏙옙-STANELY
                        U8RSPAState=Backward_Assist;
                        U8Ref_vel=DInputVR;
                        U8Parkingfail=0;

                        //占쏙옙占쏙옙占쏙옙占�
                        if(Cal_TTCR(U8Curr_vel) <= 1.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EMERGENCY;

                        }
                        else if(Cal_TTCR(U8Curr_vel) < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_DECEL;

                        }
                        //

                        if (U8IsStopline == 1 && IsWPTrackingFinish == 1) //占쏙옙占쏙옙 占싹뤄옙
                        {
                            U8Ref_vel = 0;
                            if (U8Curr_vel==0){

                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_P;
                            }
                        }
                        else if (IsWPTrackingFinish == 1 && U8IsStopline == 0){ //占쏙옙占쏙옙 WP占쏙옙 占쏙옙 占쏙옙占쏙옙占쌩는듸옙, 占쏙옙占쏙옙 占쌩뚤삐띰옙占쌀띰옙
                            U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                U8Parkingfail=1;
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_D;
                            }
                        }

                        /*else if (obstacle[B_OBSTACLE] == 0 ){ //占쏙옙灌占쏙옙 占쏙옙占쏙옙占싱다곤옙 占식뱄옙 占썸돌占실댐옙 占쏙옙占쏙옙占쏙옙 占쏙옙占쏙옙占싹곤옙 占쏙옙占� 占쏙옙占쏙옙
                            U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                U8Parkingfail=1;
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_D;
                            }
                        }*///占싱뤄옙占쏙옙황 占쏙옙占쏙옙占쏙옙占싹깍옙占쏙옙占�!!
                        break;

                    case decision_stateflow_IN_RSPA_P:
                        U8RSPAState = Parking_Complete;
                        U8Ref_vel = initVel;
                        U8IsStopline=0;
                        lanecheck_request=0;

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
