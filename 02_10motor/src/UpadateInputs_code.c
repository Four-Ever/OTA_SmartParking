#include "UpdateInputs.h"
#include "rtwtypes.h"

float stanelyAngle=0.0;
float pre_stanelyAngle=0.0;
float RefRPM=0;
int steeringInputL=-40;
int steeringInputR=40;
int conersteering = -50;
//int IsPrk_LR; //1�̸� ������ �� ����ĭ 2�� ������
/* ��Ⱦ���� reference input */
void update_VCU_inputs_c(void) {   //����»�� �������� input����/�Լ� �ְ�, Ⱦ INPUT �� �־�� ��.
    RefRPM= ((float)U8Ref_vel)*(60*gear_ratio*1000) / circumference;
    if(decision_stateflow_DW.is_c3_decision_stateflow == decision_stateflow_IN_SAFE_RCA ){

        switch (U8RCAState) {
            case Emergency:
                vehicle_status.steering_angle = 0;
                vehicle_status.ref_rpm = RefRPM;
                break;
            case Decel:
                vehicle_status.steering_angle = 0;  //
                vehicle_status.ref_rpm = RefRPM;
                break;
            default:
                vehicle_status.steering_angle = 0;
                vehicle_status.ref_rpm = 0;
                break;
        }
    }

    //���� ��û
    if (decision_stateflow_DW.is_c3_decision_stateflow == decision_stateflow_IN_FIND_CAR){
        vehicle_status.steering_angle = 0;
        vehicle_status.ref_rpm = RefRPM;
    }

    if(decision_stateflow_DW.is_c3_decision_stateflow == decision_stateflow_IN_SAFE_FCA){
        switch (U8FCAState) {
            case Emergency:
                vehicle_status.steering_angle = 0;
                vehicle_status.ref_rpm = RefRPM;
                break;
            case Decel:
                vehicle_status.steering_angle = 0;  //
                vehicle_status.ref_rpm = RefRPM;
                break;
            default:
                vehicle_status.steering_angle = 0;
                vehicle_status.ref_rpm = 0;
                break;
        }
    }


    if(decision_stateflow_DW.is_c3_decision_stateflow == decision_stateflow_IN_DRIVER_Mode){

       vehicle_status.transmission = U8IsTrButton;

        switch (U8DriverState) {
            case Parking:
                vehicle_status.steering_angle = 0;
                vehicle_status.ref_rpm = 0;
                break;
            case Driving:
                //vehicle_status.ref_rpm = 0;
                vehicle_status.steering_angle = D_steering;  //
                vehicle_status.ref_rpm = D_RefRPM;
                break;
            case Reversing:
                //vehicle_status.ref_rpm = 0;
                vehicle_status.steering_angle = D_steering;  //
                vehicle_status.ref_rpm = -D_RefRPM;
                break;
            default:
                vehicle_status.steering_angle = 0;
                vehicle_status.ref_rpm = 0;
                break;
        }
    }
    else if(decision_stateflow_DW.is_c3_decision_stateflow == decision_stateflow_IN_RSPA_Mode){
        switch (U8RSPAState) {
            case Parking_Complete:  //���� �Ϸ�
                vehicle_status.steering_angle = 0;

                vehicle_status.ref_rpm = 0.0f;
                break;
            case Forward:  //�ϵ��ڵ� ����

                    if(IsPrk_LR != RIGHT){
                        vehicle_status.steering_angle = 10;
                        vehicle_status.ref_rpm = 1500;

                    }

                    else if(md_flag==-1){
                        vehicle_status.steering_angle = 35;
                        vehicle_status.ref_rpm = 2300;
                    }
                    else if(md_flag == 0){
                        vehicle_status.steering_angle = -35;
                        vehicle_status.ref_rpm = 2300;
                    }
                    else if (md_flag==4){
                        vehicle_status.steering_angle = -35;
                        vehicle_status.ref_rpm = 2300;
                    }
                    else if (md_flag==1 || md_flag==3 || md_flag==5 || md_flag==7 || md_flag==10){

                        vehicle_status.steering_angle = 0;
                        vehicle_status.ref_rpm = 0;
                    }


                break;
            case Forward_Assist:  //������ ����, �̶��� �׳� ������ ���Ⱚ �ֱ�, Ʃ�׾ȵǸ� �������
                if (IsPrk_LR==LEFT){
                    vehicle_status.steering_angle = 10;  // ��: ���Ⱚ
                }
                else if (IsPrk_LR==RIGHT){
                    vehicle_status.steering_angle = -10;
                }
                vehicle_status.ref_rpm = RefRPM;
                break;
            case Backward:

               //�������� �� ����ĭ
                    if(md_flag==2){
                        vehicle_status.steering_angle = 40;
                        vehicle_status.ref_rpm = -1000;
                    }
                    else if (md_flag==6){
                        vehicle_status.steering_angle = 40;
                        vehicle_status.ref_rpm = -1000;
                    }
                    else if (md_flag==8){
                        vehicle_status.steering_angle = 40;
                        vehicle_status.ref_rpm = -1000;
                    }
                    else if (md_flag==9){
                        vehicle_status.steering_angle = 10;
                        vehicle_status.ref_rpm = -1000;

                    }
                    else if (md_flag==1 || md_flag==3 || md_flag==5 || md_flag==7 || md_flag==10){

                        vehicle_status.steering_angle = 0;
                        vehicle_status.ref_rpm = 0;

                    }


//                if(steeringInputR <= 19 || steeringInputL >= -19){
//                    CameraSwitchRequest=2;
//
//
//                }
                break;
            case Searching:  //�����ν� �������� Ž��
                stanelyAngle=gitstanley();
                //vehicle_status.steering_angle = (sint8)stanelyAngle;  //
                vehicle_status.steering_angle = 5;  //
                vehicle_status.ref_rpm = RefRPM;
                if (U8IsConerline==1){
                    vehicle_status.steering_angle=conersteering;
                    if (move_distance(100) == REACHED_TARGET_DIS){
                        U8IsConerline=0;

                    }
                }

                break;
            case Backward_Assist:  //�����ν� ���� RA
                stanelyAngle=gitstanley();
                vehicle_status.steering_angle = (sint8)stanelyAngle;  //
                vehicle_status.ref_rpm = -RefRPM;
                break;

            case InitRSPAState:
                //�ʱ����
                break;
        }
    }

    else if(decision_stateflow_DW.is_c3_decision_stateflow == decision_stateflow_IN_CONERING){
        switch (U8ConerState) {
            case InitConering:
                break;
            case Conering_Forward:
                if(conering_dir_flag == 0)
                {
                    vehicle_status.steering_angle = -35;
                    vehicle_status.ref_rpm = 2300;
                }
                else if(conering_dir_flag == 2)
                {
                    vehicle_status.steering_angle = -35;
                    vehicle_status.ref_rpm = 2300;
                }
                else if(conering_dir_flag == 3)
                {
                    vehicle_status.steering_angle = 0;
                    vehicle_status.ref_rpm = 1500;
                }
                break;
            case Conering_Backward:
                if(conering_dir_flag == 1)
                {
                    vehicle_status.steering_angle = 40;
                    vehicle_status.ref_rpm = -1000;
                }
                break;
            case Conering_Finished:
                break;
        }
    }

    // input
    setServoAngle(vehicle_status.steering_angle);
    RPM_CMD1=vehicle_status.ref_rpm;
}
