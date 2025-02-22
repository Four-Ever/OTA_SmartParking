#include "UpdateInputs.h"
#include "rtwtypes.h"

float stanelyAngle=0.0;
float pre_stanelyAngle=0.0;
float RefRPM=0;
int steeringInputL=-40;
int steeringInputR=40;
int conersteering = -50;
//int IsPrk_LR; //1이면 왼쪽이 빈 주차칸 2면 오른쪽
/* 종횡제어 reference input */
void update_VCU_inputs_c(void) {   //종욱쨩의 수동조작 input변수/함수 넣고, 횡 INPUT 도 넣어야 함.
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

    //출차 요청
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
            case Parking_Complete:  //주차 완료
                vehicle_status.steering_angle = 0;

                vehicle_status.ref_rpm = 0.0f;
                break;
            case Forward:  //하드코딩 전진

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
            case Forward_Assist:  //주차중 전진, 이때는 그냥 임의의 조향값 넣기, 튜닝안되면 차선기반
                if (IsPrk_LR==LEFT){
                    vehicle_status.steering_angle = 10;  // 예: 조향값
                }
                else if (IsPrk_LR==RIGHT){
                    vehicle_status.steering_angle = -10;
                }
                vehicle_status.ref_rpm = RefRPM;
                break;
            case Backward:

               //오른쪽이 빈 주차칸
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
            case Searching:  //차선인식 주차공간 탐색
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
            case Backward_Assist:  //차선인식 후진 RA
                stanelyAngle=gitstanley();
                vehicle_status.steering_angle = (sint8)stanelyAngle;  //
                vehicle_status.ref_rpm = -RefRPM;
                break;

            case InitRSPAState:
                //초기상태
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
