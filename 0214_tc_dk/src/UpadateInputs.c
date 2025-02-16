#include "UpdateInputs.h"
#include "rtwtypes.h"

float stanelyAngle=0.0;
float pre_stanelyAngle=0.0;
float RefRPM=0;
//int IsPrk_LR; //1이면 왼쪽이 빈 주차칸 2면 오른쪽
/* 종횡제어 reference input */
void update_VCU_inputs(void) {   //종욱쨩의 수동조작 input변수/함수 넣고, 횡 INPUT 도 넣어야 함.
    RefRPM= ((float)U8Ref_vel)*(60*gear_ratio) / circumference;
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

        //vehicle_status.transmission = D_trans;
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
                vehicle_status.ref_rpm = D_RefRPM;
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
                if (IsPrk_LR==LEFT) {// 왼쪽이 빈 주차칸
                    vehicle_status.steering_angle = 30;
                    vehicle_status.ref_rpm = RefRPM;
                }
                else if (IsPrk_LR==RIGHT) {  //오른쪽이 빈 주차칸
                    vehicle_status.steering_angle = -30;
                    vehicle_status.ref_rpm = RefRPM;
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
            case Reversing:
                if (IsPrk_LR==LEFT) {// 왼쪽이 빈 주차칸
                    vehicle_status.steering_angle = -40;
                    vehicle_status.ref_rpm = RefRPM;
                }
                else if (IsPrk_LR==RIGHT) {  //오른쪽이 빈 주차칸
                    vehicle_status.steering_angle = 40;
                    vehicle_status.ref_rpm = RefRPM;
                }
                break;
            case Searching:  //차선인식 주차공간 탐색
                stanelyAngle=gitstanley();
                vehicle_status.steering_angle = (sint8)stanelyAngle;  //
                vehicle_status.ref_rpm = RefRPM;
                break;
            case Backward_Assist:  //차선인식 후진 RA
                stanelyAngle=gitstanley();
                vehicle_status.steering_angle = (sint8)stanelyAngle;  //
                vehicle_status.ref_rpm = RefRPM;
                break;

            case InitRSPAState:
                //초기상태
                break;
        }
    }

    //모터 input
    setServoAngle(vehicle_status.steering_angle);
    RPM_CMD1=vehicle_status.ref_rpm;
}
