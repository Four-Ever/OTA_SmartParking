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
double U8Curr_vel = 0;
double U8Ref_vel = 0;
double DInputVD = 0.1;  //전진주차 속도
double DInputVR = -0.1;   //후진주차 속도
int IsRSPAButton = 0;
int IsOTAFinished = 0;
int U8IsWp_R = 0;
int U8IsStopline = 0;
int U8IsPrkFinished = 0;
int U8IsOb_R = 0;
int U8IsOb_D=0;
DriverState U8DriverState = InitDriverState;
RSPAState U8RSPAState = InitRSPAState;
int U8Driver = 0;
int U8RSPA = 0;
int U8Engine = 0;
int U8Parkingfail=0;
int U8PrkFinished=0;
int ExitCAR_request=0;
double D_Ref_vel=0;


int CameraSwitchRequest=0;
int Isprkslot; //초음파 헤더파일에서 값 받아와야 함
int U8Isprkslot=0;
sint8 DSteeringinput=0;
double DMoveDis=0;
int calDis=0;  //1일때 이동거리 계산 시작 요청 전달
int First_Set = 1; //차선인식 시작

IsPrk IsPrk_LR = InitIsPrk;

//안전
CAState U8FCAState = InitCAState;
CAState U8RCAState = InitCAState;
double DTTC_D= 10.0; // 전방 장애물 ttc 센서 데이터가 들어오지 않을 때, state 영향 없는 값으로 초기화
double DTTC_R= 10.0; // 후방 장애물 ttc 센서 데이터가 들어오지 않을 때, state 영향 없는 값으로 초기화
double DObs_dis_D= 100.0; //전방 장애물 상대거리 센서 데이터가 들어오지 않을 때, state 영향 없는 값으로 초기화
double DObs_dis_R= 100.0; //후방 장애물 상대거리 센서 데이터가 들어오지 않을 때, state 영향 없는 값으로 초기화
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
                U8DriverState = InitDriverState;
                U8RSPAState = InitRSPAState;

                if (vehicle_status.engine_state == ENGINE_ON && ExitCAR_request == 0 )   //시동 켜지면 운전자모드로 전환
                {
                    U8Engine=ModeOn;

                    decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_DRIVER_Mode;
                    decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_P;


                }

                if (vehicle_status.engine_state == ENGINE_OFF && ExitCAR_request==1){   //출차요청
                    decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_FIND_CAR;
                }

                break;

            case decision_stateflow_IN_FIND_CAR:

                if (ExitCAR_request==1){
                    U8Ref_vel = DInputVD;
                    //if (calDis==0.2){ //20cm 정도 출차했으면    calDis 계산하는 로직 아직 안짬
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

                        if (U8Curr_vel == 0 && DObs_dis_D >=100) //내차 정지 + 전방 장애물 사라짐
                        {
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EXIT;
                        }

                        break;
                    case decision_stateflow_IN_FCA_DECEL:
                        U8FCAState = Decel;
                        U8Ref_vel = U8Ref_vel-U8Ref_vel*(1/(DTTC_D+gainTTC)); //DTTC_D : 0 이면 에러

                        if (DObs_dis_D >=100)
                        {
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EXIT;
                        }
                        break;
                    case decision_stateflow_IN_FCA_EXIT:   //긴급제동 하기 전의 MODE 로 복귀
                                                           //탈출 조건 DObs_dis_D 만족하는지 확인
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

                        if (U8Curr_vel == 0 && DObs_dis_R >=100) //내차 정지 + 후방 장애물 사라짐
                        {
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EXIT;
                        }

                        break;
                    case decision_stateflow_IN_RCA_DECEL:
                        U8RCAState = Decel;
                        U8Ref_vel = U8Ref_vel-U8Ref_vel*(1/(DTTC_R+gainTTC));

                        if (DObs_dis_R >=100)
                        {
                            decision_stateflow_DW.is_SAFE_RCA = decision_stateflow_IN_RCA_EXIT;
                        }
                        break;
                    case decision_stateflow_IN_RCA_EXIT:   //긴급제동 하기 전의 MODE 로 복귀
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
                        else if(vehicle_status.engine_state == ENGINE_OFF){  //시동 off
                            U8Ref_vel = 0;
                            if (U8Curr_vel==0){
                                //U8Ref_vel=(0);
                                decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_INIT_Mode;
                            }
                        }
                        break;

                    case decision_stateflow_IN_DRIVER_D:
                        U8DriverState = Driving;
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
                        if (U8IsTrButton == PARKING)
                        {
                            U8Ref_vel = 0;
                            if (U8Curr_vel == 0){
                                decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_P;
                            }

                        }
                        else if (U8IsTrButton == REVERSE)
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

                                //추가한 부분 시작
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_IS_SLOT;

                                U8DriverState = InitDriverState;
                                First_Set=1;
                                IsRSPAButton = 0;
                                //VCU 에서 젯슨나노한테 전방카메라 on 해서 waypoint 보내달라고 해야함 (msg 송신)
                                //{
                                CameraSwitchRequest = 1;  // 카메라 전환 요청 플래그(전방카메라 on) (make_can_message에서 처리)
                                //}
                                //추가한 부분 끝
                                
                            }
                        }
                        break;

                    case decision_stateflow_IN_DRIVER_R:
                        U8DriverState = Reversing;
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
                        if (U8IsTrButton == DRIVING)
                        {
                            U8Ref_vel=0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_D;
                            }
                        }
                        else if(U8IsTrButton == PARKING){
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

                                U8DriverState = InitDriverState;

                                First_Set=1;
                                IsRSPAButton = 0; // 추가
                                //VCU 에서 젯슨나노한테 전방카메라 on 해서 waypoint 보내달라고 해야함 (msg 송신)
                                //{
                                CameraSwitchRequest = 1;  // 카메라 전환 요청 플래그(전방카메라 on) (make_can_message에서 처리)
                                //}
                            }
                        }
                        break;
                }
                break;

            //주차보조시스템
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
                                if(IsPrk_LR == LEFT)
                                {
                                    DSteeringinput = 20;
                                }
                                if(IsPrk_LR == RIGHT)
                                {
                                    DSteeringinput = -20;
                                }
                                //calDis=1; //거리 계산 요청
                                move_distance(100);
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_D;
                            }
                        }

                        break;

                    case decision_stateflow_IN_RSPA_D:
                        U8RSPAState=Forward;
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

                        //if (DMoveDis == 0.10)  // 10CM 좌측으로 이동했으면
                        if(move_distance(100) == REACHED_TARGET_DIS) //100mm
                        {
                            U8Ref_vel= 0;
                            
                            if(IsPrk_LR == LEFT)
                            {
                                DSteeringinput = -20;
                            }
                            if(IsPrk_LR == RIGHT)
                            {
                                DSteeringinput = 20;
                            }

                            //calDis=0;
                            if (U8Curr_vel==0){
                                // 서보모터 변경 완료되는 여유 시간 추가??
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_R;
                                //추가한 부분
                                CameraSwitchRequest = 2; // 후방 카메라
                                //추가한 부분 끝
                            }
                        }
                        break;

                    case decision_stateflow_IN_RSPA_LANE_D:
                        U8RSPAState=Forward_Assist;
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
                        U8RSPAState=Backward;
                        U8Ref_vel=DInputVR;

                        if(IsPrk_LR == LEFT && DSteeringinput < 0)
                        {
                            DSteeringinput += 1; // 초기 DSteeringinput -20
                                                 // 움직이면서 servo 조정이 항상 같은 곳으로 갈 것 같지 않음
                                                 // 조향도 그렇고, 모터가 일정하지 않아서..
                        }
                        else if(IsPrk_LR == RIGHT && DSteeringinput > 0)
                        {
                            DSteeringinput -= 1; // 초기 DSteeringinput +20
                                                 // 움직이면서 servo 조정이 항상 같은 곳으로 갈 것 같지 않음
                                                 // 조향도 그렇고, 모터가 일정하지 않아서..
                        }

                        // DSteeringinput 하나로 고정

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

                        if (U8IsWp_R == 2)  // 후방카메라 차선 인지했으면 WP 받았으면
                        {
                            U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_R; //변경
                            }
                        }
                        break;

                    case decision_stateflow_IN_RSPA_LANE_R:  //이때만 차선-STANELY
                        U8RSPAState=Backward_Assist;
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



                        if (U8IsStopline == 1 && IsWPTrackingFinish == 1) //주차 완료
                        {
                            U8Ref_vel = 0;
                            if (U8Curr_vel==0){

                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_P;
                            }
                        }
                        else if (IsWPTrackingFinish == 1 && U8IsStopline == 0){ //주차 WP는 다 추종했는데, 줄이 삐뚤삐뚤할때
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
                        U8RSPAState = Parking_Complete;
                        U8Ref_vel = initVel;
                        U8IsStopline=0;

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
