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
    SRC_ASCLIN0RX.B.SRE = 0;  // ASCLIN0 RX �씤�꽣�읇�듃 鍮꾪솢�꽦�솕
}

static void enableUartRxInterrupt() {
    SRC_ASCLIN0RX.B.SRE = 1;  // ASCLIN0 RX �씤�꽣�읇�듃 �솢�꽦�솕
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

    /* 踰꾪띁媛� 苑� 李⑤㈃, buf_tof�뿉 蹂듭궗 */
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

    /* 踰꾪띁媛� 苑� 李⑤㈃, buf_tof�뿉 蹂듭궗 */
    if (rxBuf1Idx == TOF_length)
    {
        memcpy(gBuf1_tof, rxBuf, TOF_length);
        rxBuf1Idx = 0;
    }
}

/* �닔�떊 �뜲�씠�꽣媛� �젙�긽�씠硫� 1, 洹몃젃吏� �븡�쑝硫� 0 諛섑솚 */
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

/* �쑀�슚 嫄곕━�씤 寃쎌슦 1 諛섑솚, 洹몃젃吏� �븡�쑝硫� 0 諛섑솚 */
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


    if( buf_ToF[3] == TOF0)
    {
        Distance[TOF0] = TOF_distance + OFFSET <0 ? 0:TOF_distance + OFFSET;
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
        Distance[TOF1] = TOF_distance + OFFSET <0 ? 0:TOF_distance + OFFSET;
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

