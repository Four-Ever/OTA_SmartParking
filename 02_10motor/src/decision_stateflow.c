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
//DW_decision_stateflow_T decision_stateflow_DW;

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
double DInputVD = 0.1;  //�������� �ӵ�
double DInputVR = -0.1;   //�������� �ӵ�
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
int lanecheck_request=0;

int CameraSwitchRequest=0;
int Isprkslot; //������ ������Ͽ��� �� �޾ƿ;� ��
int U8Isprkslot=0;
sint8 DSteeringinput=0;
double DMoveDis=0;
int calDis=0;  //1�϶� �̵��Ÿ� ��� ���� ��û ����
int First_Set = 1; //�����ν� ����
int U8IsConerline=0;

IsPrk IsPrk_LR = InitIsPrk;

//����
CAState U8FCAState = InitCAState;
CAState U8RCAState = InitCAState;
double DTTC_D= 10.0; // ���� ��ֹ� ttc ���� �����Ͱ� ������ ���� ��, state ���� ���� ������ �ʱ�ȭ
double DTTC_R= 10.0; // �Ĺ� ��ֹ� ttc ���� �����Ͱ� ������ ���� ��, state ���� ���� ������ �ʱ�ȭ
double DObs_dis_D= 100.0; //���� ��ֹ� ���Ÿ� ���� �����Ͱ� ������ ���� ��, state ���� ���� ������ �ʱ�ȭ
double DObs_dis_R= 100.0; //�Ĺ� ��ֹ� ���Ÿ� ���� �����Ͱ� ������ ���� ��, state ���� ���� ������ �ʱ�ȭ
double gainTTC=0.0; //Ʃ�� �Ķ����
DW_decision_stateflow_T decision_stateflow_DW={0,0,0,0,0,0,0};



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

                if (vehicle_status.engine_state == ENGINE_ON && ExitCAR_request == 0 )   //�õ� ������ �����ڸ��� ��ȯ
                {
                    U8Engine=ModeOn;

                    decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_DRIVER_Mode;
                    decision_stateflow_DW.is_DRIVER_Mode = decision_stateflow_IN_DRIVER_P;


                }

                if (vehicle_status.engine_state == ENGINE_OFF && ExitCAR_request==1){   //������û
                    decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_FIND_CAR;
                }

                break;

            case decision_stateflow_IN_FIND_CAR:

                if (ExitCAR_request==1){
                    U8Ref_vel = DInputVD;
                    //if (calDis==0.2){ //20cm ���� ����������    calDis ����ϴ� ���� ���� ��«
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

                        if (U8Curr_vel == 0 && DObs_dis_D >=100) //���� ���� + ���� ��ֹ� �����
                        {
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EXIT;
                        }

                        break;
                    case decision_stateflow_IN_FCA_DECEL:
                        U8FCAState = Decel;
                        U8Ref_vel = U8Ref_vel-U8Ref_vel*(1/(DTTC_D+gainTTC)); //DTTC_D : 0 �̸� ����

                        if (DObs_dis_D >=100)
                        {
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EXIT;
                        }
                        break;
                    case decision_stateflow_IN_FCA_EXIT:   //������� �ϱ� ���� MODE �� ����
                                                           //Ż�� ���� DObs_dis_D �����ϴ��� Ȯ��
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

                        if (U8Curr_vel == 0 && DObs_dis_R >=100) //���� ���� + �Ĺ� ��ֹ� �����
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
                    case decision_stateflow_IN_RCA_EXIT:   //������� �ϱ� ���� MODE �� ����
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
                        else if(vehicle_status.engine_state == ENGINE_OFF){  //�õ� off
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

                        //�������
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
                        //����
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

                                //�߰��� �κ� ����
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_IS_SLOT;

                                U8DriverState = InitDriverState;
                                First_Set=1;
                                IsRSPAButton = 0;
                                //VCU ���� ������������ ����ī�޶� on �ؼ� waypoint �����޶�� �ؾ��� (msg �۽�)
                                //{
                                CameraSwitchRequest = 1;  // ī�޶� ��ȯ ��û �÷���(����ī�޶� on) (make_can_message���� ó��)
                                //}
                                //�߰��� �κ� ��
                                
                            }
                        }
                        break;

                    case decision_stateflow_IN_DRIVER_R:
                        U8DriverState = Reversing;
                        U8Ref_vel = D_Ref_vel;

                        //�������
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
                        //����
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
                                IsRSPAButton = 0; // �߰�
                                //VCU ���� ������������ ����ī�޶� on �ؼ� waypoint �����޶�� �ؾ��� (msg �۽�)
                                //{
                                CameraSwitchRequest = 1;  // ī�޶� ��ȯ ��û �÷���(����ī�޶� on) (make_can_message���� ó��)
                                //}
                            }
                        }
                        break;
                }
                break;

            //���������ý���
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

                        //�������
                        if(DTTC_D <= 1.0)    //ttc ����ϴ� ���� ���� ��«
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_EMERGENCY;

                        }
                        else if(DTTC_D < 3.0)
                        {
                            decision_stateflow_DW.is_c3_decision_stateflow = decision_stateflow_IN_SAFE_FCA;
                            decision_stateflow_DW.is_SAFE_FCA = decision_stateflow_IN_FCA_DECEL;

                        }

                        if (U8Isprkslot == 1)   // ������ĭ ã���� STATE ����
                        {
                            U8Ref_vel = 0;
                            if (U8Curr_vel==0){

                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_D;

                            }
                        }



                        break;

                    case decision_stateflow_IN_RSPA_D:
                        U8RSPAState=Forward;
                        U8Ref_vel=DInputVD;

                        //�������
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

                        //if (DMoveDis == 0.10)  // 10CM �������� �̵�������
                        if(move_distance(100) == REACHED_TARGET_DIS) //100mm
                        {
                            DInputVD= 0;
                            
                            if (U8Curr_vel==0){
                                // �������� ���� �Ϸ�Ǵ� ���� �ð� �߰�??
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_R;
                                //�߰��� �κ�
                                CameraSwitchRequest = 2; // �Ĺ� ī�޶�
                                //�߰��� �κ� ��
                            }
                        }
                        break;

                    case decision_stateflow_IN_RSPA_LANE_D:
                        U8RSPAState=Forward_Assist;
                        U8Ref_vel=DInputVD;
                        //�������
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

                        if (U8Parkingfail==1 )  //�ѹ��� ���� ���ؼ� �߱�߱�, STEERING �� ���� ����ݴ�� * (-1)
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

                    case decision_stateflow_IN_RSPA_R:  //steering �� 3�� ����
                        U8RSPAState=Backward;
                        U8Ref_vel=DInputVR;

                        move_distance(700);

                        //�������
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

                        if (CameraSwitchRequest == 2)  // �Ĺ�ī�޶� ���� ���������� WP �޾�����
                        {
                            U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_R; //����
                            }
                        }
                        break;

                    case decision_stateflow_IN_RSPA_LANE_R:  //�̶��� ����-STANELY
                        U8RSPAState=Backward_Assist;
                        U8Ref_vel=DInputVR;
                        U8Parkingfail=0;

                        //�������
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

                        if (U8IsStopline == 1 && IsWPTrackingFinish == 1) //���� �Ϸ�
                        {
                            U8Ref_vel = 0;
                            if (U8Curr_vel==0){

                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_P;
                            }
                        }
                        else if (IsWPTrackingFinish == 1 && U8IsStopline == 0){ //���� WP�� �� �����ߴµ�, ���� �߶Ի߶��Ҷ�
                            U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                U8Parkingfail=1;
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_D;
                            }
                        }

                        /*else if (U8IsOb_R == 0 ){ //��δ�� �����̴ٰ� �Ĺ� �浹�Ǵ� ������ �����ϰ� ��� ����
                            U8Ref_vel= 0;
                            if (U8Curr_vel==0){
                                U8Parkingfail=1;
                                decision_stateflow_DW.is_RSPA_Mode = decision_stateflow_IN_RSPA_LANE_D;
                            }
                        }*///�̷���Ȳ �������ϱ����!!
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
