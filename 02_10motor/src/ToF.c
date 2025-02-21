#include "ToF.h"
#include "ASCLIN_UART.h"
#include <string.h>



static const unsigned char TOF_length = 16;
static unsigned int rxBuf0Idx = 0;
static unsigned int rxBuf1Idx = 0;
static unsigned char gBuf0_tof[16] = {0};
static unsigned char gBuf1_tof[16] = {0};
int Distance[NUM_TOF] ={0};

static void disableUartRxInterrupt() {
    SRC_ASCLIN0RX.B.SRE = 0;  // ASCLIN0 RX 인터럽트 비활성화
}

static void enableUartRxInterrupt() {
    SRC_ASCLIN0RX.B.SRE = 1;  // ASCLIN0 RX 인터럽트 활성화
}

void Init_ToF(void)
{
    init_ASC0LIN_UART();
    init_ASC1LIN_UART();
}

/* Interrupt Service Routine for RX */
IFX_INTERRUPT(IsrUart0RxHandler_tof, 0, INTPRIO_ASCLIN0_RX);
void IsrUart0RxHandler_tof(void)
{
    IfxAsclin_Asc_isrReceive(&g_asc0Handle);
    static unsigned char rxBuf[16] = {0};

    uint8 c = in_uart0();

    rxBuf[rxBuf0Idx] = c;
    ++rxBuf0Idx;

    /* 버퍼가 꽉 차면, buf_tof에 복사 */
    if (rxBuf0Idx == TOF_length)
    {
        memcpy(gBuf0_tof, rxBuf, TOF_length);
        rxBuf0Idx = 0;
    }
}

IFX_INTERRUPT(IsrUart1RxHandler_tof, 0, INTPRIO_ASCLIN1_RX);
void IsrUart1RxHandler_tof(void)
{
    IfxAsclin_Asc_isrReceive(&g_asc1Handle);
    static unsigned char rxBuf[16] = {0};

    uint8 c = in_uart1();

    rxBuf[rxBuf1Idx] = c;
    ++rxBuf1Idx;

    /* 버퍼가 꽉 차면, buf_tof에 복사 */
    if (rxBuf1Idx == TOF_length)
    {
        memcpy(gBuf1_tof, rxBuf, TOF_length);
        rxBuf1Idx = 0;
    }
}

/* 수신 데이터가 정상이면 1, 그렇지 않으면 0 반환 */
static int verifyCheckSum(unsigned char data[])
{
    unsigned char checksum = 0;
    for (int i = 0; i < TOF_length - 1; i++)
    {
        checksum += data[i];
    }
    if (data[0] == 0x57 && data[1] == 0x0 && data[2] == 0xFF)
    {
        return checksum == data[TOF_length - 1];
    }
    else
    {
        return 0;
    }
}

/* 유효 거리인 경우 1 반환, 그렇지 않으면 0 반환 */
static int checkTofStrength(unsigned char data[])
{
    int TOF_distance = data[8] | (data[9] << 8) | (data[10] << 16);
    int TOF_signal_strength = data[12] | (data[13] << 8);
    /* when distance over 2m - out of range */
    if (TOF_signal_strength != 0 && TOF_distance != 0xFFFFF6u)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/* Return Distance(mm) */

void ToF_get_All_Distance(){
    int TOF_distance = 0;
    unsigned char buf_ToF[TOF_length];

    /* copy buf_tof into tmp */
//    disableUartRxInterrupt();
    memcpy(buf_ToF, gBuf0_tof, TOF_length);
//    enableUartRxInterrupt();

    /* for debugging */
    //     for (int i = 0; i < 16; i++) {
    //         my_printf("%.2X ", buf_ToF[i]);
    //     }

    if (!verifyCheckSum(buf_ToF))
    {
        return;
    }
    if (!checkTofStrength(buf_ToF))
    {
        return;
    }

    TOF_distance = buf_ToF[8] | (buf_ToF[9] << 8) | (buf_ToF[10] << 16);


    if( buf_ToF[3] == TOF1)
    {
        Distance[TOF1] = (TOF_distance + OFFSET) <0 ? 0: (TOF_distance + OFFSET);
        Distance[TOF1]=B_getFilteredDistance(Distance[TOF1]);
    }
    else if( buf_ToF[3] == TOF0)
    {
        Distance[TOF0] = (TOF_distance + OFFSET) <0 ? 0:(TOF_distance + OFFSET);
        Distance[TOF0]=F_getFilteredDistance(Distance[TOF0]);
    }

//    disableUartRxInterrupt();
    memcpy(buf_ToF, gBuf1_tof, TOF_length);
//    enableUartRxInterrupt();

    if (!verifyCheckSum(buf_ToF))
    {
        return;
    }
    if (!checkTofStrength(buf_ToF))
    {
        return;
    }

    TOF_distance = buf_ToF[8] | (buf_ToF[9] << 8) | (buf_ToF[10] << 16);

    if( buf_ToF[3] == TOF1)
    {
        Distance[TOF1] = (TOF_distance + OFFSET) <0 ? 0: (TOF_distance + OFFSET);
        Distance[TOF1]=B_getFilteredDistance(Distance[TOF1]);
    }
    else if( buf_ToF[3] == TOF0)
    {
        Distance[TOF0] = (TOF_distance + OFFSET) <0 ? 0:(TOF_distance + OFFSET);
        Distance[TOF0]=F_getFilteredDistance(Distance[TOF0]);
    }

    return;
}

int getTofDistance()
{
    int TOF_distance = 0;
    unsigned char buf_ToF[TOF_length];

    /* copy buf_tof into tmp */
    memcpy(buf_ToF, gBuf0_tof, TOF_length);

    /* for debugging */
    //     for (int i = 0; i < 16; i++) {
    //         my_printf("%.2X ", buf_ToF[i]);
    //     }

    if (!verifyCheckSum(buf_ToF))
    {
        return -1;
    }
    if (!checkTofStrength(buf_ToF))
    {
        return -2;
    }

    TOF_distance = buf_ToF[8] | (buf_ToF[9] << 8) | (buf_ToF[10] << 16);

    return TOF_distance;
}
int F_getFilteredDistance(int TOF_Value)
{
    static int F_buffer[TOF_FILTER_COUNT] = {0};
    static int F_index = 0;

    F_buffer[F_index] = TOF_Value;
    F_index = (F_index + 1) % TOF_FILTER_COUNT;

    int sum = 0, weightSum = 0;
    uint8 weight=TOF_FILTER_COUNT;
    for (int i = 0; i < TOF_FILTER_COUNT; i++) {
        int idx = (F_index - 1 - i + TOF_FILTER_COUNT) % TOF_FILTER_COUNT; // 최신 값에 더 큰 가중치
        sum += F_buffer[idx] * weight;
        weightSum += weight;
        weight--;
    }

    return (int)(sum / weightSum);
}

int B_getFilteredDistance(int TOF_Value)
{
    static int B_buffer[TOF_FILTER_COUNT] = {0};
    static int B_index = 0;

    B_buffer[B_index] = TOF_Value;
    B_index = (B_index + 1) % TOF_FILTER_COUNT;

    int sum = 0, weightSum = 0;
    uint8 weight=TOF_FILTER_COUNT;
    for (int i = 0; i < TOF_FILTER_COUNT; i++) {
        int idx = (B_index - 1 - i + TOF_FILTER_COUNT) % TOF_FILTER_COUNT; // 최신 값에 더 큰 가중치
        sum += B_buffer[idx] * weight;
        weightSum += weight;
        weight--;
    }

    return (int)(sum / weightSum);
}
