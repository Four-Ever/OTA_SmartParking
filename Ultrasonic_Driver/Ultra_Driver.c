/*******************************************************************************
 * @file    Ultra_Driver.h
 * @brief   Ultra Sonic distance
 * @version 1.0
 * @date    2025-02-07
 ******************************************************************************/

#include "Ultra_Driver.h"

void initUltrasonic(IfxPort_Pin TRIG_PIN, IfxPort_Pin ECHO_PIN)
{
    IfxPort_setPinModeOutput(TRIG_PIN.port, TRIG_PIN.pinIndex, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeInput(ECHO_PIN.port, ECHO_PIN.pinIndex, IfxPort_InputMode_noPullDevice);
}
float getDistance(IfxPort_Pin TRIG_PIN, IfxPort_Pin ECHO_PIN)
{
    uint32 startTime=0;
    uint32 endTime=0;
    uint32 timeout =0;
    uint32 pulseDuration=0;
    float distance=0;

    IfxPort_setPinLow(TRIG_PIN.port, TRIG_PIN.pinIndex);  //초기화
    IfxPort_setPinHigh(TRIG_PIN.port, TRIG_PIN.pinIndex); //TRIG 신호 발생
    delay(10);
    IfxPort_setPinLow(TRIG_PIN.port, TRIG_PIN.pinIndex); //10u초간 TRIG 신호 발생
    startTime =  MODULE_STM0.TIM0.U;
    timeout = startTime + 500000;//100cm
    while(IfxPort_getPinState(ECHO_PIN.port, ECHO_PIN.pinIndex)==FALSE)//high 되면 측정 시작
        {
            if(MODULE_STM0.TIM0.U>timeout)
                {
                    return -1;
                }
        }
    startTime =  MODULE_STM0.TIM0.U;
    timeout = startTime + 500000;//100cm
    while (IfxPort_getPinState(ECHO_PIN.port, ECHO_PIN.pinIndex) == TRUE)//low 되면 감지 끝
    {
        if(MODULE_STM0.TIM0.U>timeout)
        {
            return -2;
        }
    }
    endTime =  MODULE_STM0.TIM0.U;
    pulseDuration= endTime - startTime;
    distance = (pulseDuration * 0.0343) /100/ 2;
    return distance;
}
void delay(uint32 us){
    uint32 startTime = MODULE_STM0.TIM0.U;
    while((MODULE_STM0.TIM0.U - startTime) < (us * (IfxStm_getFrequency(&MODULE_STM0) / 1000000)));
}
