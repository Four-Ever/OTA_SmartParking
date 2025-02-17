/************************************************************
 * Copyright (c) 2023, Infineon Technologies AG
 * Encoder reading using GPT12 Incremental Interface Mode
 ************************************************************/

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
#include "Ifx_Types.h"
#include "IfxCpu.h"
//#include "Cpu/Irq/IfxCpu_Irq.h"
//#include "IfxCpu_Irq.h"

#include "IfxScuWdt.h"
#include "IfxPort.h"
#include "IfxStm.h"
#include "Ifx_Types.h"

#include "Ifx_DateTime.h"
#include "SysSe/Bsp/Bsp.h"

#include "STM_Interrupt.h"

#include "OurCan.h"

#include "Driver_Stm.h"
#include "ASCLIN_Shell_UART.h"
#include "servo.h"
#include "EncMotor.h"

//#include "Ifx_IntPrioDef.h"
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
//#define motor_Test //엔코더 거리 확인
//#define putty_Test //putty uart 메세지 확인
//#define tuning_Test //시뮬링크에서 pid 계수 조정 및 스코프 확인
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

typedef enum
{
    Parking = 0,
    Driving = 1,
    Reverse = 2
}Transmission;

typedef enum
{
    engine_off  = 0,
    engine_on   = 1
}EngineState;

typedef enum
{
    system_drive_mode = 0,
    user_drive_mode = 1
}Mode;

typedef enum
{
    serching    = 0,
    positioning = 1, // 주차 공간 인식 후 전진하는 상태
    reversing   = 2, // 후진 주차 중
    parked      = 3 // 주차 완료
}ParkingStatus;


typedef struct
{
    Mode user_mode;
    float32 target_rpm;
    float32 servo_angle;

    uint8 velocity;
    sint8 steering_angle;
    Transmission transmission;
    ParkingStatus parking_status;
    EngineState engine_state;
}VehicleStatus;
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
IfxCpu_syncEvent g_cpuSyncEvent = 0;

VehicleStatus vehicle_status = {system_drive_mode, 0.0f, 0.0f, 0, 0,
                                    Parking, parked, engine_off};
Taskcnt stTestCnt;
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
void make_can_message(void);
sint16 round_to_integer(float32 num);
void update_vehicle_velocity(void);
void update_message_vehicle_status(VCU_Vehicle_Status_Msg* dest, const VehicleStatus* src);
void update_message_parking_status(VCU_Parking_Status_Msg* dest, const VehicleStatus* src);
void update_message_engine_status(VCU_Vehicle_Engine_Status_Msg* dest, const VehicleStatus* src);

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


float g_angle;

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

    initGtmATomPwm();
    initPins();
    initPeripherals(); /* Initialize the STM module */

    initCan();
    initCanDB();
    initShellInterface();
    Driver_Stm_Init();

    initServo(); // D6

    motor_dir = 0;    // 0:정방향, 1:역방향
    motor_enable = 0;  // 0:제동, 1:해제

#ifdef motor_Test
    motor_enable = 1;  // 0:제동, 1:해제

    Kp_s = 1.55f;//1.75f;
    Ki_s = 2.65f;//0.198f;
    Kd_s = 0.001f;

    vehicle_status.engine_state = engine_on;

    waitTime(300000000); // 3초
    init_move_distance_control(1000.0f, 500.0f); // 1000mm, 1000rpm
#endif

    while(1)
    {
        AppScheduling();

        //can 메세지 받는 곳
#if !defined(motor_Test) && !defined(tuning_Test) && !defined(putty_Test) // 주행 코드
        // 엔진 메세지
        if (db_flag.CGW_Engine_Flag == 1)
        {
            db_flag.CGW_Engine_Flag = 0;
            vehicle_status.engine_state = db_msg.CGW_Engine.control_engine;

            if (vehicle_status.engine_state == engine_on)
            {
                motor_enable = 1;
            }
            else if (vehicle_status.engine_state == engine_off)
            {
                motor_enable = 0;
            }
        }

        // 엔진이 켜져 있을 때,
        if (vehicle_status.engine_state == engine_on)
        {
            // 수동조작 메세지
            if (db_flag.CGW_Move_Flag == 1)
            {
                db_flag.CGW_Move_Flag = 0;

                // 수동 조작 모드로 전환
                vehicle_status.user_mode = user_drive_mode;
                if (db_msg.CGW_Move.control_accel == 1) // accel
                {
                    vehicle_status.target_rpm += 100.0f; // period 100ms 기준 1초 동안 누르면, 1000rpm 목표

                    if (vehicle_status.target_rpm >= 3000.0f)
                    {
                        vehicle_status.target_rpm = 3000.0f;
                    }
                }
                if (db_msg.CGW_Move.control_brake == 1)
                {
                    vehicle_status.target_rpm -= 200.0f;

                    if (vehicle_status.target_rpm < 0.0f)
                    {
                        vehicle_status.target_rpm = 0.0f;
                    }
                }
                vehicle_status.servo_angle = db_msg.CGW_Move.control_steering_angle;

                if (vehicle_status.transmission != db_msg.CGW_Move.control_transmission)
                {
                    vehicle_status.target_rpm = 0.0f;
                    vehicle_status.transmission = db_msg.CGW_Move.control_transmission;
                }
            }

            // 자동 주차 요청

            // OTA 업데이트 확인

            // 차량 찾기 요청

            // 출차 요청
        }
#endif
    }

    return 0;
}


void make_can_message(void)
{
    update_vehicle_velocity();

    update_message_vehicle_status(&db_msg.VCU_Vehicle_Status, &vehicle_status);
    output_message(&db_msg.VCU_Vehicle_Status, VCU_Vehicle_Status_ID);

    update_message_parking_status(&db_msg.VCU_Parking_Status, &vehicle_status);
    output_message(&db_msg.VCU_Parking_Status, VCU_Parking_Status_ID);

    update_message_engine_status(&db_msg.VCU_Vehicle_Engine_Status, &vehicle_status);
    output_message(&db_msg.VCU_Vehicle_Engine_Status, VCU_Vehicle_Engine_Status_ID);
}


void update_message_vehicle_status(VCU_Vehicle_Status_Msg* dest, const VehicleStatus* src)
{
    dest->vehicle_velocity = vehicle_status.velocity;
    dest->vehicle_steering_angle = (sint8)vehicle_status.servo_angle;
    dest->vehicle_transmission = vehicle_status.transmission;
}


void update_message_parking_status(VCU_Parking_Status_Msg* dest, const VehicleStatus* src)
{
    dest->parking_status = vehicle_status.parking_status;
}


void update_message_engine_status(VCU_Vehicle_Engine_Status_Msg* dest, const VehicleStatus* src)
{
    dest->vehicle_engine_status = vehicle_status.engine_state;
}


sint16 round_to_integer(float32 num)
{
    sint16 res;
    if (num >= 0)
    {
        res = (sint16)(num + 0.5f);
    }
    else
    {
        res = (sint16)(num - 0.5f);
    }
    return res;
}

void update_vehicle_velocity(void)
{
    float32 value = (s32_motor_speed_rpm * circumference) / (60 * gear_ratio);
    sint16 s16_velocity;
    uint16 u16_velocity;

    value /= 10; // 7bit로 보내주기 위함

    s16_velocity = round_to_integer(value);
    u16_velocity = s16_velocity >= 0  ? s16_velocity : (s16_velocity * -1);
    u16_velocity = u16_velocity > 100 ? 100 : u16_velocity;

    vehicle_status.velocity = (uint8)u16_velocity; // (실제 속도 / 10) [mm/0.1s]
}


void AppTask1ms(void)
{
    stTestCnt.u32nuCnt1ms++;
}


void AppTask10ms(void)
{
    stTestCnt.u32nuCnt10ms++;

#ifdef motor_Test
    setServoAngle(-50.0f);
    print_dis(&s32_DisSum);
#endif

#ifdef tuning_Test
    Kp_s = p_gain;
    Ki_s = i_gain;
    Kd_s = d_gain;
    RPM_CMD1 = rpm_speed;
    setServoAngle(servo_angle);

    print_enc(&s32_motor_speed_rpm);
#endif

#ifdef putty_Test
    Kp_s = 1.55f;
    Ki_s = 2.65f;
    Kd_s = 0.001f;
    RPM_CMD1 = 1500.0f;

    myprintf("rpm : %d\r\n", s32_motor_speed_rpm);
#endif

#if !defined(motor_Test) && !defined(tuning_Test) && !defined(putty_Test) // 주행 코드
    make_can_message();
#endif
}


void AppTask100ms(void)
{
    stTestCnt.u32nuCnt100ms++;

#if !defined(motor_Test) && !defined(tuning_Test) && !defined(putty_Test) // 주행 코드
    if (vehicle_status.engine_state == engine_on)
    {
        if (vehicle_status.user_mode == user_drive_mode)
        {
            if (vehicle_status.transmission == Driving)
            {
                RPM_CMD1 = vehicle_status.target_rpm;
            }
            else if (vehicle_status.transmission == Reverse)
            {
                RPM_CMD1 = vehicle_status.target_rpm * -1;
            }
            setServoAngle(vehicle_status.servo_angle);
        }
    }
#endif
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
