#include "UpdateInputs.h"
#include "rtwtypes.h"

float stanelyAngle=0.0;
float pre_stanelyAngle=0.0;
float RefRPM=0;
int steeringInputL=-40;
int steeringInputR=40;
int conersteering = -50;

//int IsPrk_LR;
/*reference input */

void update_VCU_inputs_c(void) {
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

    if (decision_stateflow_DW.is_c3_decision_stateflow == decision_stateflow_IN_FIND_CAR){
        vehicle_status.steering_angle = 10;
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
            case Parking_Complete:
                vehicle_status.steering_angle = 0;

                vehicle_status.ref_rpm = 0.0f;
                break;
            case Forward:

                    if(IsPrk_LR != RIGHT){
                        vehicle_status.steering_angle = 10;
                        vehicle_status.ref_rpm = 1000;

                    }

                    else if(md_flag==-1){
                        vehicle_status.steering_angle = 35;
                        vehicle_status.ref_rpm = 1500; //2000
                    }
                    else if(md_flag == 0){
                        vehicle_status.steering_angle = -35;
                        vehicle_status.ref_rpm = 1500; //2000
                    }
                    else if (md_flag==4){
                        vehicle_status.steering_angle = -35;
                        vehicle_status.ref_rpm = 1500;
                    }
                    else if (md_flag==8){
                            vehicle_status.steering_angle = -35;
                            vehicle_status.ref_rpm = 1500;
                        }
                    else if (md_flag==9){
                        vehicle_status.steering_angle = 40;
                        vehicle_status.ref_rpm = 1800;
                    }
                    else if (md_flag==1 || md_flag==3 || md_flag==5 || md_flag==7 || md_flag==10){

                        vehicle_status.steering_angle = 0;
                        vehicle_status.ref_rpm = 0;
                    }


                break;
            case Forward_Assist:
                if (IsPrk_LR==LEFT){
                    vehicle_status.steering_angle = 10;
                }
                else if (IsPrk_LR==RIGHT){
                    vehicle_status.steering_angle = -10;
                }
                vehicle_status.ref_rpm = RefRPM;
                break;
            case Backward:

                    if(md_flag==2){
                        vehicle_status.steering_angle = 40;
                        vehicle_status.ref_rpm = -1000;
                    }
                    else if (md_flag==6){
                        vehicle_status.steering_angle = 40;
                        vehicle_status.ref_rpm = -1000;
                    }
//                    else if (md_flag==8){
//                        vehicle_status.steering_angle = 25;
//                        vehicle_status.ref_rpm = -1000;
//                    }

                    else if (md_flag==10){
                        vehicle_status.steering_angle = 10;
                        vehicle_status.ref_rpm = -1000;

                    }
                    else if (md_flag==1 || md_flag==3 || md_flag==5 || md_flag==7 || md_flag==11){

                        vehicle_status.steering_angle = 0;
                        vehicle_status.ref_rpm = 0;

                    }


//                if(steeringInputR <= 19 || steeringInputL >= -19){
//                    CameraSwitchRequest=2;
//
//
//                }
                break;
            case Searching:
                stanelyAngle=gitstanley();
                vehicle_status.steering_angle = (sint8)stanelyAngle;  //
                //vehicle_status.steering_angle = (sint8)stanelyAngle;  //
                vehicle_status.ref_rpm = RefRPM;
                break;
            case Backward_Assist:
                stanelyAngle=gitstanleytest();
                vehicle_status.steering_angle = (sint8)stanelyAngle;  //
                vehicle_status.ref_rpm = -RefRPM;
                break;

            case InitRSPAState:
                break;
        }
    }

    else if(decision_stateflow_DW.is_c3_decision_stateflow == decision_stateflow_IN_CONERING){
        switch (U8ConerState) {
            case InitConering:
                break;
            case Conering_Forward:
                if(conering_dir_flag == -2)
                {
                    vehicle_status.steering_angle = 0;
                    vehicle_status.ref_rpm = 1200;

                }
                else if(conering_dir_flag == -1)
                {
                    vehicle_status.steering_angle = 35;
                    vehicle_status.ref_rpm = 1800;

                }
                else if(conering_dir_flag == 0)
                {
                    vehicle_status.steering_angle = -35;
                    vehicle_status.ref_rpm = 1800;
                }
                else if(conering_dir_flag == 2)
                {
                    vehicle_status.steering_angle = -30;
                    vehicle_status.ref_rpm = 1800;
                }
                else if(conering_dir_flag == 4)
                {
                    vehicle_status.steering_angle = -35;
                    vehicle_status.ref_rpm = 1800;
                }

                else if(conering_dir_flag == 6)
                {
                    vehicle_status.steering_angle = 20;
                   vehicle_status.ref_rpm = 1800;
                }
                else if(conering_dir_flag  == 7)
                {
                    vehicle_status.steering_angle = -35;
                   vehicle_status.ref_rpm = 1800;
                }
                else if(conering_dir_flag  == 9)
                {
                    vehicle_status.steering_angle = -35;
                   vehicle_status.ref_rpm = 1800;
                }
                else if(conering_dir_flag  == 11)
                {
                    vehicle_status.steering_angle = -35;
                   vehicle_status.ref_rpm = 1800;
                }

                break;
            case Conering_Backward:
                if(conering_dir_flag == 1)
                {
                    vehicle_status.steering_angle = 30;
                    vehicle_status.ref_rpm = -1500;
                }
                else if(conering_dir_flag == 3)
                {
                    vehicle_status.steering_angle = 25;
                    vehicle_status.ref_rpm = -1500;
                }
                else if(conering_dir_flag == 5)
                {
                    vehicle_status.steering_angle = 35;
                    vehicle_status.ref_rpm = -1500;
                }
                else if(conering_dir_flag == 8)
                {
                    vehicle_status.steering_angle = 35;
                    vehicle_status.ref_rpm = -1500;
                }
                else if(conering_dir_flag == 10)
                {
                    vehicle_status.steering_angle = 30;
                    vehicle_status.ref_rpm = -1500;
                }
                break;
            case Conering_Finished:
                vehicle_status.steering_angle = 8;
                vehicle_status.ref_rpm = 550;
                break;
        }
    }

    // input
    setServoAngle(vehicle_status.steering_angle);
    RPM_CMD1=vehicle_status.ref_rpm;
}
