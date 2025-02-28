/*******************************************************************************
 * @file    Ultra_Driver.h
 * @brief   Ultra Sonic distance
 * @version 1.0
 * @date    2025-02-07
 ******************************************************************************/

#include "Ultra_Driver.h"
const IfxPort_Pin R_TRIG_PIN = {&MODULE_P14, 0};  //22
const IfxPort_Pin R_ECHO_PIN = {&MODULE_P14, 1};  //23
const IfxPort_Pin L_TRIG_PIN = {&MODULE_P15, 6};  //24
const IfxPort_Pin L_ECHO_PIN = {&MODULE_P00, 0};  //25
float Ultra_Distance[NUM_ULTRA] = {0};
#define ULTRA_FILTER_COUNT 5  //이동평균필터 크기 5
void initUltrasonic (void)
{
    IfxPort_setPinModeOutput(R_TRIG_PIN.port, R_TRIG_PIN.pinIndex, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeInput(R_ECHO_PIN.port, R_ECHO_PIN.pinIndex, IfxPort_InputMode_noPullDevice);
    IfxPort_setPinModeOutput(L_TRIG_PIN.port, L_TRIG_PIN.pinIndex, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeInput(L_ECHO_PIN.port, L_ECHO_PIN.pinIndex, IfxPort_InputMode_noPullDevice);
}
float getDistance (int RLstauts)
{
    IfxPort_Pin TRIG_PIN, ECHO_PIN;
    if (RLstauts == R_ULTRA)
    {
        TRIG_PIN = R_TRIG_PIN;
        ECHO_PIN = R_ECHO_PIN;
    }
    else
    {
        TRIG_PIN = L_TRIG_PIN;
        ECHO_PIN = L_ECHO_PIN;
    }

    uint32 startTime = 0;
    uint32 endTime = 0;
    uint32 timeout = 0;
    uint32 pulseDuration = 0;
    float distance = 0;

    IfxPort_setPinLow(TRIG_PIN.port, TRIG_PIN.pinIndex);  //초기화
    IfxPort_setPinHigh(TRIG_PIN.port, TRIG_PIN.pinIndex); //TRIG 신호 발생
    delay(10);
    IfxPort_setPinLow(TRIG_PIN.port, TRIG_PIN.pinIndex); //10u초간 TRIG 신호 발생
    startTime = MODULE_STM0.TIM0.U;
    timeout = startTime + 500000; //100cm
    while (IfxPort_getPinState(ECHO_PIN.port, ECHO_PIN.pinIndex) == FALSE) //high 되면 측정 시작
    {
        if (MODULE_STM0.TIM0.U > timeout)
        {
            return 100;
        }
    }
    startTime = MODULE_STM0.TIM0.U;
    timeout = startTime + 500000; //100cm
    while (IfxPort_getPinState(ECHO_PIN.port, ECHO_PIN.pinIndex) == TRUE) //low 되면 감지 끝
    {
        if (MODULE_STM0.TIM0.U > timeout)
        {
            return 100;
        }
    }
    endTime = MODULE_STM0.TIM0.U;
    pulseDuration = endTime - startTime;
    distance = (pulseDuration * 0.0343) / 100 / 2;
    if (RLstauts == R_ULTRA)
        return R_getFilteredDistance(distance);
    else
        return L_getFilteredDistance(distance);
}
void delay (uint32 us)
{
    uint32 startTime = MODULE_STM0.TIM0.U;
    while ((MODULE_STM0.TIM0.U - startTime) < (us * (IfxStm_getFrequency(&MODULE_STM0) / 1000000)))
        ;
}

float R_getFilteredDistance (float Ultra_Value)
{
    static float R_buffer[ULTRA_FILTER_COUNT] = {0};
    static int R_index = 0;

    R_buffer[R_index] = Ultra_Value;
    R_index = (R_index + 1) % ULTRA_FILTER_COUNT;

    float sum = 0, weightSum = 0;
    uint8 weight = ULTRA_FILTER_COUNT;
    for (int i = 0; i < ULTRA_FILTER_COUNT; i++)
    {
        int idx = (R_index - 1 - i + ULTRA_FILTER_COUNT) % ULTRA_FILTER_COUNT; // 최신 값에 더 큰 가중치
        sum += R_buffer[idx] * weight;
        weightSum += weight;
        weight--;
    }

    return sum / weightSum;
}

float L_getFilteredDistance (float Ultra_Value)
{
    static float L_buffer[ULTRA_FILTER_COUNT] = {0};
    static int L_index = 0;

    L_buffer[L_index] = Ultra_Value;
    L_index = (L_index + 1) % ULTRA_FILTER_COUNT;

    float sum = 0, weightSum = 0;
    uint8 weight = ULTRA_FILTER_COUNT;
    for (int i = 0; i < ULTRA_FILTER_COUNT; i++)
    {
        int idx = (L_index - 1 - i + ULTRA_FILTER_COUNT) % ULTRA_FILTER_COUNT; // 최신 값에 더 큰 가중치
        sum += L_buffer[idx] * weight;
        weightSum += weight;
        weight--;
    }

    return sum / weightSum;
}
