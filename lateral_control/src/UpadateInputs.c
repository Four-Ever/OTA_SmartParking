#include "UpdateInputs.h"
#include "rtwtypes.h"

float LatControlInput=0.0;
float LonControlInput=0.0;

/* 종횡제어 reference input */
void update_VCU_inputs(void) {   //종욱쨩의 수동조작 input변수/함수 넣.
    if(decision_stateflow_DW.is_c3_decision_stateflow == decision_stateflow_IN_DRIVER_Mode){
        switch (U8DriverState) {
            case 'P':
                LatControlInput = 0;
                LonControlInput = 0;
                break;
            case 'D':
                LatControlInput = 0;  //
                LonControlInput = DInputVD;
                break;
            case 'R':
                LatControlInput = 0;  //
                LonControlInput = DInputVR;
                break;
            default:
                LatControlInput = 0;
                LonControlInput = 0;
                break;
        }
    }

    else if(decision_stateflow_DW.is_c3_decision_stateflow == decision_stateflow_IN_RSPA_Mode){
        switch (U8RSPAState) {
            case 'P':
                LatControlInput = 0.0;
                LonControlInput = 0.0;
                break;
            case 'D':
                LatControlInput = 1;  // 예: 조향값
                LonControlInput = DInputVD;
                break;
            case 'R':
                LatControlInput = -1;  // 예: 조향 반전
                LonControlInput = DInputVR;
                break;
            default:
                LatControlInput = 0;
                LonControlInput = 0;
                break;
        }
    }

    //서보input하는 함수 주면 될듯
    //dc input 주면 됨

}
