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
char initState = '0';
double initVel = 0;
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
int U8IsOb_D=0;
char U8DriverState = '0';
char U8RSPAState = '0';
int U8Driver = 0;
int U8RSPA = 0;
int U8Engine = 0;
int U8Parkingfail=0;
int U8PrkFinished=0;
int ExitCAR_request=0;
double D_Ref_vel=0;


int CameraSwitchRequest=0;
int Isprkslot; //초음파 헤더파일에서 값 받아와야 함
int movedDistance;  //stanely 헤더파일에서 계산?
int U8Isprkslot=0;
double DSteeringinput=0.0;
double DMoveDis=0;
int calDis=0;  //1일때 이동거리 계산 시작 요청 전달
int First_Set = 1; //차선인식 시작

//안전
char U8FCAState='0';
char U8RCAState='0';
double DTTC_D=0.0; // 전방 장애물 ttc
double DTTC_R=0.0; // 후방 장애물 ttc
double DObs_dis_D=0.0; //전방 장애물 상대거리
double DObs_dis_R=0.0; //후방 장애물 상대거리
double gainTTC=0.0; //튜닝 파라미터




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

                if (motor_enable == 1 && ExitCAR_request==0 )   //시동 켜지면 운전자모드로 전환
                {
                    U8Engine=ModeOn;
                    if (vehicle_status.user_mode == USER_DRIVE_MODE){
                        decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_DRIVER_Mode;
                        decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_P;
                    }

                }

                if (ExitCAR_request==1){   //출차요청
                    decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_FIND_CAR;
                }

                break;

            case decision_stateflow_IN_FIND_CAR:

                if (ExitCAR_request==1){
                    U8Ref_vel = DInputVD;
                    if (calDis==0.2){ //20cm 정도 출차했으면    calDis 계산하는 로직 아직 안짬
                        U8Ref_vel=0;
                        ExitCAR_request=0;
                        U8PrkFinished=0;
                        U8Parkingfail=0;

                        decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_INIT_Mode;

                    }
                }
               break;


            case decision_stateflow_IN_SAFE_FCA:

                switch (decision_stateflow_DW.is_SAFE_FCA)
                {
                    case decision_stateflow_IN_FCA_EMERGENCY:
                        U8FCAState = 'E';
                        U8Ref_vel = initVel;

                        if (U8Curr_vel == 0 && DObs_dis_D >=100) //내차 정지 + 전방 장애물 사라짐
                        {
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EXIT;
                        }

                        break;
                    case decision_stateflow_IN_FCA_DECEL:
                        U8FCAState = 'D';
                        U8Ref_vel = U8Ref_vel-U8Ref_vel*(1/DTTC_D+gainTTC);

                        if (DObs_dis_D >=100)
                        {
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EXIT;
                        }
                        break;
                    case decision_stateflow_IN_FCA_EXIT:   //긴급제동 하기 전의 MODE 로 복귀
                        U8FCAState = initState;

                        if (U8DriverState == 'D')
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_DRIVER_Mode;
                            decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_D;
                        }
                        else if(U8RSPAState == 'D'){
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_D;
                        }
                        else if(U8RSPAState == 'J'){
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_D;
                        }
                        else if(U8RSPAState == 'S'){
                            First_Set=1;
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_IS_SLOT;
                            CameraSwitchRequest=1;
                        }
                        break;
                }
                break;

            case decision_stateflow_IN_SAFE_RCA:

                switch (decision_stateflow_DW.is_SAFE_RCA){
                    case decision_stateflow_IN_RCA_EMERGENCY:
                        U8RCAState = 'E';
                        U8Ref_vel = initVel;

                        if (U8Curr_vel == 0 && DObs_dis_R >=100) //내차 정지 + 후방 장애물 사라짐
                        {
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EXIT;
                        }

                        break;
                    case decision_stateflow_IN_RCA_DECEL:
                        U8RCAState = 'D';
                        U8Ref_vel = U8Ref_vel-U8Ref_vel*(1/DTTC_R+gainTTC);

                        if (DObs_dis_R >=100)
                        {
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EXIT;
                        }
                        break;
                    case decision_stateflow_IN_RCA_EXIT:   //긴급제동 하기 전의 MODE 로 복귀
                        U8RCAState = initState;

                        if (U8DriverState == 'R')
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_DRIVER_Mode;
                            decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_R;
                        }
                        else if(U8RSPAState == 'R'){
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_R;
                        }
                        else if(U8RSPAState == 'K'){
                            First_Set=1;
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_RSPA_Mode;
                            decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_R;
                            CameraSwitchRequest=2;
                        }
                        break;
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

                        if (D_trans == 1 && U8Curr_vel == 0)
                        {
                            decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_D;
                        }
                        else if(motor_enable==0){  //시동 off
                            if (U8Curr_vel==0){
                                U8Ref_vel=(0);
                                decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_INIT_Mode;
                            }
                        }
                        break;

                    case decision_stateflow_IN_DRIVER_D:
                        U8DriverState = 'D';
                        U8Ref_vel = D_Ref_vel;

                        //긴급제동
                        if(DTTC_D <= 1.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EMERGENCY;

                        }
                        else if(DTTC_D < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_DECEL;

                        }
                        //기어변경
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
                                IsRSPAButton=0;
                            }
                        }
                        break;

                    case decision_stateflow_IN_DRIVER_R:
                        U8DriverState = 'R';
                        U8Ref_vel = D_Ref_vel;

                        //긴급제동
                        if(DTTC_R <= 1.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EMERGENCY;

                        }
                        else if(DTTC_R < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_DECEL;

                        }
                        //기어변경
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
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_IS_SLOT;
                                U8DriverState=initState;
                                First_Set=1;
                                //VCU 에서 젯슨나노한테 전방카메라 on 해서 waypoint 보내달라고 해야함 (msg 송신)
                                {
                                    CameraSwitchRequest = 1;  // 카메라 전환 요청 플래그(전방카메라 on)
                                }
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


                    case decision_stateflow_IN_RSPA_IS_SLOT:
                        U8RSPAState='S';
                        U8Ref_vel=DInputVD;

                        //긴급제동
                        if(DTTC_D <= 1.0)    //ttc 계산하는 로직 아직 안짬
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EMERGENCY;

                        }
                        else if(DTTC_D < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_DECEL;

                        }

                        if (U8Isprkslot == 1)   // 빈주차칸 찾으면 STATE 변경
                        {
                            U8Ref_vel = 0;
                            if (U8Curr_vel==0){
                                DSteeringinput=20;
                                calDis=1; //거리 계산 요청
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_D;
                            }
                        }

                        break;

                    case decision_stateflow_IN_RSPA_D:
                        U8RSPAState='D';
                        U8Ref_vel=DInputVD;

                        //긴급제동
                        if(DTTC_D <= 1.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EMERGENCY;

                        }
                        else if(DTTC_D < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_DECEL;

                        }


                        if (DMoveDis == 0.10)  // 10CM 좌측으로 이동했으면
                        {
                            U8Ref_vel= 0;
                            calDis=0;
                            if (U8Curr_vel==0){

                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_R;
                            }
                        }
                        break;

                    case decision_stateflow_IN_RSPA_LANE_D:
                        U8RSPAState='J';
                        U8Ref_vel=DInputVD;
                        //긴급제동
                        if(DTTC_D <= 1.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EMERGENCY;

                        }
                        else if(DTTC_D < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_DECEL;

                        }

                        if (U8Parkingfail==1 )  //한번에 주차 못해서 삐까삐까, STEERING 은 이전 조향반대로 * (-1)
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

                    case decision_stateflow_IN_RSPA_R:  //steering 은 -10씩 감소
                        U8RSPAState='R';
                        U8Ref_vel=DInputVR;
                        //긴급제동
                        if(DTTC_R <= 1.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EMERGENCY;

                        }
                        else if(DTTC_R < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_DECEL;

                        }

                        if (U8IsWp_R == 1)  // 후방카메라 차선 인지했으면 WP 받았으면
                        {
                            U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_R;
                            }
                        }
                        break;

                    case decision_stateflow_IN_RSPA_LANE_R:  //이때만 차선-STANELY
                        U8RSPAState='K';
                        U8Ref_vel=DInputVR;
                        U8Parkingfail=0;
                        //긴급제동
                        if(DTTC_R <= 1.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EMERGENCY;

                        }
                        else if(DTTC_R < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_RCA;
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_DECEL;

                        }
                        //
                        if (U8IsStopline == 1 && U8IsPrkFinished == 1) //주차 완료
                        {
                            U8Ref_vel = 0;
                            if (U8Curr_vel==0){

                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_P;
                            }
                        }
                        else if (U8IsPrkFinished == 1 && U8IsStopline == 0){ //주차 WP는 다 추종했는데, 줄이 삐뚤삐뚤할때
                            U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                U8Parkingfail=1;
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_D;
                            }
                        }

                        else if (U8IsOb_R == 0 ){ //경로대로 움직이다가 후방 충돌판단 감지시 정지하고 모드 변경
                            U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                U8Parkingfail=1;
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_D;
                            }
                        }
                        break;

                    case decision_stateflow_IN_RSPA_P:
                        U8RSPAState='P';
                        U8Ref_vel = initVel;

                        if (motor_enable==0){
                            if (U8Curr_vel==0){
                                U8PrkFinished=1;
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
