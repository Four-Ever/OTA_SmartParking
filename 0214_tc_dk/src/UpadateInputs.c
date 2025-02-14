#include "UpdateInputs.h"
#include "gitstanley.h"
#include "rtwtypes.h"

float stanelyAngle=0.0;
float pre_stanelyAngle=0.0;

 int IsPrk_LR;

/* 종횡제어 reference input */
void update_VCU_inputs(void) {   //종욱쨩의 수동조작 input변수/함수 넣고, 횡 INPUT 도 넣어야 함.
    //stanelyAngle=gitstanely();
    stanelyAngle=0;
    if(decision_stateflow_DW.is_c3_decision_stateflow == decision_stateflow_IN_SAFE_RCA ){
        switch (U8RCAState) {
            case 'E':
                vehicle_status.servo_angle = 0;
                vehicle_status.target_rpm = U8Ref_vel;
                break;
            case 'D':
                vehicle_status.servo_angle = 0;  //
                vehicle_status.target_rpm = U8Ref_vel;
                break;
            default:
                vehicle_status.servo_angle = 0;
                vehicle_status.target_rpm = 0;
                break;
        }
    }

    if(decision_stateflow_DW.is_c3_decision_stateflow == decision_stateflow_IN_SAFE_FCA){
        switch (U8FCAState) {
            case 'E':
                vehicle_status.servo_angle = 0;
                vehicle_status.target_rpm = U8Ref_vel;
                break;
            case 'D':
                vehicle_status.servo_angle = 0;  //
                vehicle_status.target_rpm = U8Ref_vel;
                break;
            default:
                vehicle_status.servo_angle = 0;
                vehicle_status.target_rpm = 0;
                break;
        }
    }




    if(decision_stateflow_DW.is_c3_decision_stateflow == decision_stateflow_IN_DRIVER_Mode){
        switch (U8DriverState) {
            case 'P':
                vehicle_status.servo_angle = 0;
                vehicle_status.target_rpm = 0;
                break;
            case 'D':
                vehicle_status.target_rpm = 0;
                vehicle_status.servo_angle = D_steering;  //
                vehicle_status.target_rpm = D_RefRPM;
                break;
            case 'R':
                vehicle_status.target_rpm = 0;
                vehicle_status.servo_angle = D_steering;  //
                vehicle_status.target_rpm = D_RefRPM;
                break;
            default:
                vehicle_status.servo_angle = 0;
                vehicle_status.target_rpm = 0;
                break;
        }
    }
    else if(decision_stateflow_DW.is_c3_decision_stateflow == decision_stateflow_IN_RSPA_Mode){
        switch (U8RSPAState) {
            case 'P':  //주차 완료
                vehicle_status.servo_angle = 0.0;
                vehicle_status.target_rpm = 0.0;
                break;
            case 'D':  //하드코딩 전진
                if (IsPrk_LR==1) {// 왼쪽이 빈 주차칸
                    vehicle_status.servo_angle = 30.0;
                    vehicle_status.target_rpm = DInputVD;
                }
                else if (IsPrk_LR==2) {  //오른쪽이 빈 주차칸
                    vehicle_status.servo_angle = -30.0;
                    vehicle_status.target_rpm = DInputVD;
                }

                break;
            case 'J':  //주차중 전진
                if (IsPrk_LR==1){
                    vehicle_status.servo_angle = 10;  // 예: 조향값
                }
                else if (IsPrk_LR==2){
                    vehicle_status.servo_angle = -10;
                }
                vehicle_status.target_rpm = DInputVD;
                break;
            case 'R':
                if (IsPrk_LR==1) {// 왼쪽이 빈 주차칸
                    vehicle_status.servo_angle = -40.0;
                    vehicle_status.target_rpm = DInputVD;
                }
                else if (IsPrk_LR==2) {  //오른쪽이 빈 주차칸
                    vehicle_status.servo_angle = 40.0;
                    vehicle_status.target_rpm = DInputVD;
                }
                break;
            case 'S':  //차선인식 주차공간 탐색
                vehicle_status.servo_angle = stanelyAngle;  // 예: 조향값
                vehicle_status.target_rpm = DInputVD;
                break;
            case 'K':  //차선인식 후진 RA
                vehicle_status.servo_angle = stanelyAngle;  // 예: 조향 반전
                vehicle_status.target_rpm = DInputVR;
                break;
            default:
                vehicle_status.servo_angle = stanelyAngle;
                vehicle_status.target_rpm = DInputVD;
                break;
        }
    }
    pre_stanelyAngle=stanelyAngle;

    //모터 input
    setServoAngle(vehicle_status.servo_angle);
    (vehicle_status.target_rpm);

    //dc input 주면 됨

}
