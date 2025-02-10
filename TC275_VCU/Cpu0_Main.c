/************************************************************
 * Copyright (c) 2023, Infineon Technologies AG
 * Encoder reading using GPT12 Incremental Interface Mode
 ************************************************************/

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxGpt12_IncrEnc.h"
#include "IfxCpu.h"
//#include "Cpu/Irq/IfxCpu_Irq.h"
//#include "IfxCpu_Irq.h"

#include "IfxScuWdt.h"
#include "IfxPort.h"
#include "IfxStm.h"
#include "Ifx_Types.h"

#include "Ifx_DateTime.h"
#include "SysSe/Bsp/Bsp.h"

#include "IfxGtm_reg.h"
#include "GTM_ATOM_PWM.h"

#include "STM_Interrupt.h"

//#include "OurCan.h"

#include "Driver_Stm.h"
#include "ASCLIN_Shell_UART.h"
#include "servo.h"
#include "Logger.h"

//#include "Ifx_IntPrioDef.h"
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
#define WAIT_TIME   10              /* Number of milliseconds to wait between each duty cycle change                */

// Interrupt priority definitions
#define ISR_PRIORITY_INCRENC_ZERO 6

// 엔코더 설정
#define PULSES_PER_REV 12     // 한 채널당 펄스 수

// 핀 정의
#define PWMA_PIN &MODULE_P02,1   // PWM 핀 (P2.1)
#define BRAKEA_PIN &MODULE_P02,7 // 브레이크 핀 (P2.7)
#define DIRA_PIN &MODULE_P10,1   // 방향 제어 핀 (P10.1)
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
typedef struct
{
    uint32 u32nuCnt1ms;
    uint32 u32nuCnt10ms;
    uint32 u32nuCnt100ms;
    uint32 u32nuCnt1000ms;
    uint32 u32nuCnt5000ms;
} Taskcnt;
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
IfxCpu_syncEvent g_cpuSyncEvent = 0;

IfxGpt12_IncrEnc_Config gpt12Config;
IfxGpt12_IncrEnc gpt12;

const uint8 CPR = (PULSES_PER_REV * 4);  // 4체배시 한바퀴 펄스 수

volatile uint8 motor_speed = 0;    // 0~100
volatile boolean motor_dir = 0;    // 0:정방향, 1:역방향
volatile boolean motor_enable = 0;  // 0:제동, 1:해제

volatile float32 Kp_s=0,Ki_s=0,Kd_s=0;
volatile float32 RPM_CMD1=0;

Taskcnt stTestCnt;
char cEnc_count[16];
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
void convert_sint32_to_char(char* des, sint32* src);

void AppScheduling(void);
void AppTask1ms(void);
void AppTask10ms(void);
void AppTask100ms(void);
void AppTask1000ms(void);
void AppTask5000ms(void);
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/

// Interrupt handler for zero crossing
//IFX_INTERRUPT(ISR_IncrIncZero, 0, ISR_PRIORITY_INCRENC_ZERO)
//{
//    IfxGpt12_IncrEnc_onZeroIrq(&gpt12Config);
//}

void initIncrEnc(void)
{
    // Initialize global clocks
    IfxGpt12_enableModule(&MODULE_GPT120);
    IfxGpt12_setGpt1BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt1BlockPrescaler_8);
    IfxGpt12_setGpt2BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt2BlockPrescaler_4);

    // Create module config
//    IfxGpt12_IncrEnc_Config gpt12Config;
    IfxGpt12_IncrEnc_initConfig(&gpt12Config, &MODULE_GPT120);

    // Configure encoder parameters
    gpt12Config.base.offset               = 100;                    // Initial position offset
    gpt12Config.base.reversed             = FALSE;               // Count direction not reversed
    gpt12Config.base.resolution           = PULSES_PER_REV;                // Encoder resolution
    gpt12Config.base.periodPerRotation    = 1;                   // Number of periods per rotation
    gpt12Config.base.resolutionFactor     = IfxStdIf_Pos_ResolutionFactor_fourFold;  // Quadrature mode
    gpt12Config.base.updatePeriod         = 0.001;              // 1ms update period
    gpt12Config.base.speedModeThreshold   = 100;                // Threshold for speed calculation mode
    gpt12Config.base.minSpeed             = 10;                 // Minimum speed in rpm
    gpt12Config.base.maxSpeed             = 5000;                // Maximum speed in rpm

    // Configure pins
    //gpt12Config.pinA = &IfxGpt120_T2INA_P00_7_IN;     // Encoder A signal -> T3IN
    //gpt12Config.pinB = &IfxGpt120_T2EUDA_P00_8_IN;    // Encoder B signal -> T3EUD
    gpt12Config.pinB = &IfxGpt120_T2INA_P00_7_IN;     // Encoder A signal -> T3IN
    gpt12Config.pinA = &IfxGpt120_T2EUDA_P00_8_IN;    // Encoder B signal -> T3EUD

    gpt12Config.pinZ = NULL;                          // No Z signal used
    gpt12Config.pinMode = IfxPort_InputMode_pullDown;   // Use internal pullup

    // Configure interrupts
    gpt12Config.zeroIsrPriority = ISR_PRIORITY_INCRENC_ZERO;
    gpt12Config.zeroIsrProvider = IfxSrc_Tos_cpu0;

    // Enable speed filter
    gpt12Config.base.speedFilterEnabled = TRUE;
    gpt12Config.base.speedFilerCutOffFrequency = gpt12Config.base.maxSpeed / 2 * IFX_PI * 2;

    // Initialize module
    IfxGpt12_IncrEnc_init(&gpt12, &gpt12Config);

}


// 모터 제어 함수
void setMotorControl(uint8 direction, uint8 enable)
{
    // 브레이크 설정
    if (enable == 0)
    {
        IfxPort_setPinState(BRAKEA_PIN, IfxPort_State_high); // 브레이크 활성화
        // PWM 출력 중지
        GTM_TOM0_TGC0_GLB_CTRL.B.UPEN_CTRL1 = 0;
        return;
    }
    else
    {
        IfxPort_setPinState(BRAKEA_PIN, IfxPort_State_low); // 브레이크 비활성화
        GTM_TOM0_TGC0_GLB_CTRL.B.UPEN_CTRL1 = 2;
    }

    // 방향 설정
    if (direction == 0)
    {
        IfxPort_setPinState(DIRA_PIN, IfxPort_State_low); // 정방향
    }
    else
    {
        IfxPort_setPinState(DIRA_PIN, IfxPort_State_high); // 역방향
    }


}

// 핀 초기화
void initPins(void)
{
    // 방향 핀 초기화
    IfxPort_setPinMode(DIRA_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinState(DIRA_PIN, IfxPort_State_low);

    // 브레이크 핀 초기화
    IfxPort_setPinMode(BRAKEA_PIN, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinState(BRAKEA_PIN, IfxPort_State_low);
}



float32 speed;
sint32 rawPosition;
IfxStdIf_Pos_Dir direction;

float g_angle;

void Encoder_update(void)
{
    IfxGpt12_IncrEnc_update(&gpt12);
    speed = IfxGpt12_IncrEnc_getSpeed(&gpt12);
    rawPosition = IfxGpt12_IncrEnc_getRawPosition(&gpt12);
    direction = IfxGpt12_IncrEnc_getDirection(&gpt12);
}


void convert_sint32_to_char(char* des, sint32* src)
{
    sint32 value = *src;

    // 첫 문자에 부호 입력
    if (value < 0) {
        des[0] = '-';
        value = -value;  // 절대값으로 변환
    } else {
        des[0] = '+';
    }

    // 숫자를 문자열로 변환
    sint32 temp = value;
    int digit_count = 1;  // 최소 1자리

    // 자릿수 계산
    while (temp >= 10) {
        temp /= 10;
        digit_count++;
    }

    // 숫자를 문자로 변환하여 뒤에서부터 입력
    int idx = digit_count;
    temp = value;

    // 숫자가 0인 경우 처리
    if (value == 0) {
        des[1] = '0';
        des[2] = '\0';
        return;
    }

    // 각 자릿수를 문자로 변환
    while (temp > 0) {
        des[idx] = '0' + (temp % 10);
        temp /= 10;
        idx--;
    }

    // 문자열 종료
    des[digit_count + 1] = '\0';
}


int core0_main(void)
{
    // Initialize system
    IfxCpu_enableInterrupts();

    /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
     * Enable the watchdogs and service them periodically if it is required
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    /* Wait for CPU sync event */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    /* Initialize a time variable */
//    Ifx_TickTime ticksFor10ms = IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME);

    // Install interrupt handlers
//    IfxCpu_Irq_installInterruptHandler(&ISR_IncrIncZero, ISR_PRIORITY_INCRENC_ZERO);

    // Initialize peripherals
    initIncrEnc();
//    initLED();
//    initGpt12Timer();
    initGtmATomPwm();
    initPins();
    initPeripherals(); /* Initialize the STM module */

    initShellInterface();
    Driver_Stm_Init();

    //initUartLogger();

    //initServo(); // D30

    //setServoAngle(20.0f);

    //motor_speed = 15;    // 0~100
    motor_dir = 0;    // 0:정방향, 1:역방향
    motor_enable = 1;  // 0:제동, 1:해제

    //uint8 testData;
    waitTime(300000000); // 3초
    //RPM_CMD1=0;
    //waitTime(1000000); // 0.01초
    // Main loop

    while(1)
    {
        //runShellInterface();
        AppScheduling();

        /*
        //can 메세지 받는 곳


        // 엔진 메세지
        if (db_msg.CTRL_Engine_Msg.B.Flag == 1)
        {
            db_msg.CTRL_Engine_Msg.B.Flag = 0;

        }

        // 수동조작 메세지
        if (db_msg.CTRL_Move_Msg.B.Flag == 1)
        {
            db_msg.CTRL_Move_Msg.B.Flag = 0;
        }

        //accel 신호 오면(10ms) :
        //target vel + 1 씩

        //brake 신호 오면(10ms) :
        //target vel - 5 씩

        //steer angle 신호 오면(10ms) :
        //서보모터 angle로 변환
        setServoAngle(steer angle);
       */

// test
//        for (g_angle = -50.0f; g_angle <= 50.0f; g_angle += 0.2f)
//        {
//            setServoAngle(g_angle);
//            waitTime(1000000); // 50ms
//        }
//
//        for (g_angle = 50.0f; g_angle >= -50.0f; g_angle -= 0.2f)
//        {
//            setServoAngle(g_angle);
//            waitTime(1000000);
//        }
// test
    }

    return 0;
}


void AppTask1ms(void)
{
    stTestCnt.u32nuCnt1ms++;
}


void AppTask10ms(void)
{
    stTestCnt.u32nuCnt10ms++;
}


void AppTask100ms(void)
{
    stTestCnt.u32nuCnt100ms++;
    //setServoAngle(20.0f);
    print_enc(&s32_motor_speed_rpm);

    Kp_s = p_gain;
    Ki_s = i_gain;
    Kd_s = d_gain;
    RPM_CMD1 = pwm_speed;
}


void AppTask1000ms(void)
{
    stTestCnt.u32nuCnt1000ms++;
}

void AppTask5000ms(void)
{
    stTestCnt.u32nuCnt5000ms++;
}


void AppScheduling(void)
{

    if (stSchedulingInfo.u8nuScheduling1msFlag == 1u)
    {
        stSchedulingInfo.u8nuScheduling1msFlag = 0u;

        AppTask1ms();

        if (stSchedulingInfo.u8nuScheduling10msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling10msFlag = 0u;
            AppTask10ms();
        }

        if (stSchedulingInfo.u8nuScheduling100msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling100msFlag = 0u;
            AppTask100ms();
        }
        if (stSchedulingInfo.u8nuScheduling1000msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling1000msFlag = 0u;
            AppTask1000ms();
        }
        if (stSchedulingInfo.u8nuScheduling5000msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling5000msFlag = 0u;
            AppTask5000ms();
        }
    }
}
/*********************************************************************************************************************/
