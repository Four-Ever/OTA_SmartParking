/************************************************************
 * Copyright (c) 2023, Infineon Technologies AG
 * Encoder reading using GPT12 Incremental Interface Mode
 ************************************************************/

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
#include "Ifx_Types.h"
#include "IfxCpu.h"
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
#include "decision_stateflow.h"
#include "UpdateInputs.h"
#include "Homo_Coordinate.h"


//ToF 헤더
#include "ToF.h"

//#include "Ifx_IntPrioDef.h"
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
//#define motor_Test //�뿏肄붾뜑 嫄곕━ �솗�씤
//#define putty_Test //putty uart 硫붿꽭吏� �솗�씤
//#define tuning_Test //�떆裕щ쭅�겕�뿉�꽌 pid 怨꾩닔 議곗젙 諛� �뒪肄뷀봽 �솗�씤
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
    uint32 u32nuCnt50ms;
    uint32 u32nuCnt100ms;
    uint32 u32nuCnt1000ms;
    uint32 u32nuCnt5000ms;
} Taskcnt;


/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
IfxCpu_syncEvent g_cpuSyncEvent = 0;
Taskcnt stTestCnt;
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
void make_can_message(void);
void update_message_vehicle_status(VCU_Vehicle_Status_Msg* dest, const VehicleStatus* src);
void update_message_parking_status(VCU_Parking_Status_Msg* dest, const VehicleStatus* src);
void update_message_engine_status(VCU_Vehicle_Engine_Status_Msg* dest, const VehicleStatus* src);

void AppScheduling(void);
void AppTask1ms(void);
void AppTask10ms(void);
void AppTask50ms(void);
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
IMU now_status = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
Euler now_euler = { 0, 0, 0 };
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame
float stanelytheta = 0.0f;
int stopstatus = 0;

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
    initIMU();
    initGPIO();


    //ToF user manual !
    Init_ToF(); // init

    ToF_get_All_Distance(); // put this code to task code ( work for synchronize recent distance data )
    Distance[TOF0]; // uart0 ToF Data
    Distance[TOF1]; // uart2 ToF Data

    // connect -> check the excel PINMAP Sheet

    // motor_dir = 0;    // 0:�젙諛⑺뼢, 1:�뿭諛⑺뼢
    // motor_enable = 0;  // 0:�젣�룞, 1:�빐�젣

#ifdef motor_Test
    // motor_enable = 1;  // 0:�젣�룞, 1:�빐�젣

    Kp_s = 1.55f;//1.75f;
    Ki_s = 2.65f;//0.198f;
    Kd_s = 0.001f;

    vehicle_status.engine_state = engine_on;

    waitTime(300000000); // 3珥�
    init_move_distance_control(1000.0f, 500.0f); // 1000mm, 1000rpm
#endif

    while(1)
    {
        AppScheduling();
        stopstatus=Touch();
        //can 硫붿꽭吏� 諛쏅뒗 怨�
#if !defined(motor_Test) && !defined(tuning_Test) && !defined(putty_Test) // 二쇳뻾 肄붾뱶
        // �뿏吏� 硫붿꽭吏�
        if (db_flag.CGW_Engine_Flag == 1)
        {
            db_flag.CGW_Engine_Flag = 0;
            //if (db_msg.CGW_Engine.control_engine==1){
            //    vehicle_status.engine_state = db_msg.CGW_Engine.control_engine;
            //}
            if (U8DriverState == Parking || U8RSPAState==Parking_Complete || U8DriverState == InitDriverState ){
                vehicle_status.engine_state = db_msg.CGW_Engine.control_engine;
            }

        }

        // �뿏吏꾩씠 耳쒖졇 �엳�쓣 �븣,
        if (vehicle_status.engine_state == ENGINE_ON)
        {
            // �닔�룞議곗옉 硫붿꽭吏�
            if (db_flag.CGW_Move_Flag == 1)
            {
                db_flag.CGW_Move_Flag = 0;

                //D_trans = db_msg.CGW_Move.control_transmission;
                U8IsTrButton = db_msg.CGW_Move.control_transmission;
                //vehicle_status.transmission = db_msg.CGW_Move.control_transmission;
                
                D_steering = db_msg.CGW_Move.control_steering_angle;
                //vehicle_status.steering_angle = db_msg.CGW_Move.control_steering_angle;

                // �닔�룞 議곗옉 紐⑤뱶濡� �쟾�솚
                //vehicle_status.user_mode = USER_DRIVE_MODE;
                if (db_msg.CGW_Move.control_accel == 1) // accel
                {
                    D_RefRPM += 100.0f;

                    if (D_RefRPM >= 3000.0f)
                    {
                        D_RefRPM = 3000.0f;
                    }
                }
                if (db_msg.CGW_Move.control_brake == 1)  //decel
                {
                    D_RefRPM -= 200.0f;

                    if (D_RefRPM < 0.0f)
                    {
                        D_RefRPM = 0.0f;
                    }
                }
                D_Ref_vel = (double)((D_RefRPM * circumference) / (60 * gear_ratio));
            }

            // �옄�룞 二쇱감 �슂泥�

            if (db_flag.CGW_Auto_Parking_Request_Flag==1)
            {
                db_flag.CGW_Auto_Parking_Request_Flag=0;

                IsRSPAButton = 1;

            }
            //wp �닔�떊�솗�씤
            if (db_flag.CCU_Cordi_data1_Flag == 1 && db_flag.CCU_Cordi_data2_Flag == 1) {
                // 紐⑤뱺 �뜲�씠�꽣媛� �뱾�뼱�삩 寃쎌슦
                db_flag.CCU_Cordi_data1_Flag=0;
                db_flag.CCU_Cordi_data2_Flag=0;

                // 移대찓�씪 �뵿�� 醫뚰몴 ���옣
                //if (db_msg.CCU_Cordi_data2.trust_value > 0.7){  //�떊猶곕룄 �넂�쓣寃쎌슦留� 媛믪쓣 �궗�슜�븿.
                    // �씠�쟾 �뜲�씠�꽣 珥덇린�솕
                    InitCampoints();
                    cam_points[0][0] = db_msg.CCU_Cordi_data1.cordi_data_x1;
                    cam_points[0][1] = db_msg.CCU_Cordi_data1.cordi_data_y1;
                    cam_points[1][0] = db_msg.CCU_Cordi_data1.cordi_data_x2;
                    cam_points[1][1] = db_msg.CCU_Cordi_data1.cordi_data_y2;

                    cam_points[2][0] = db_msg.CCU_Cordi_data2.cordi_data_x3;
                    cam_points[2][1] = db_msg.CCU_Cordi_data2.cordi_data_y3;
                    cam_points[3][0] = db_msg.CCU_Cordi_data2.cordi_data_x4;
                    cam_points[3][1] = db_msg.CCU_Cordi_data2.cordi_data_y4;

                    int camera_mode = db_msg.CCU_Cordi_data2.using_camera;

                    data_ready_flag = 1;
                //}
                if (U8RSPAState == Searching || U8RSPAState == Backward_Assist) {  //李⑥꽑湲곕컲 二쇳뻾 紐⑤뱶�뒗 �몢媛쒕컰�뿉 �뾾�쓬
                    if (data_ready_flag == 1){
                        InitWorldpoints();

                        if(First_Set==1){   //留� 泥섏쓬�뿉 wp 諛쏆쓣�븣
                            initStanley();
                            transform_points(H, cam_points, world_points); //醫뚰몴怨� 蹂��솚
                            if (transform_finished==1){
                                updateWaypoints(world_points);  //stanely �쓽 臾숉몴 waypoint 蹂�寃�
                            }
                            First_Set=0;
                            if (U8RSPAState==Searching){
                                lanecheck_request=1;
                            }
                        }
                        else if (IsWPTrackingFinish==1){ // 洹� �씠�썑�뿉�뒗 �씠�쟾 wp 紐⑤몢 異붿쥌�뻽�쓣�븣 �깉濡� 媛깆떊�븿.
                            initStanley();
                            transform_points(H, cam_points, world_points);
                            if (transform_finished==1){
                                updateWaypoints(world_points);
                            }
                        }
                        U8IsWp_R=camera_mode;
                    }
                }
            }


            //�젙吏��꽑 媛곷룄
            /*if (db_flag.CCU_RightAngle_detect_flag == 1)
            {
                db_flag.CCU_RightAngle_detect_flag = 0;
                U8IsStopline = db_flag.CCU_RightAngle_detect.right_angle_lane_detected;
            }*/
        }

        //�뿏吏꾩씠 爰쇱졇 �엳�쓣 �븣,
        else if (vehicle_status.engine_state == ENGINE_OFF)
        {
            if (db_flag.CGW_Off_Request_Flag==1)
            {
                db_flag.CGW_Off_Request_Flag=0;

                // 李⑤웾 李얘린 �슂泥�
                if (db_msg.CGW_Off_Request.alert_request==1)
                {
                    //二쇱감�븳 李⑤웾�씠 �엳�쑝硫� LED �샊�� 遺��� �굪�슜�굪�슜
                    if (U8PrkFinished==1)
                    {

                    }
                }

                // 異쒖감 �슂泥�
                if (db_msg.CGW_Off_Request.auto_exit_request==1)
                {
                    vehicle_status.engine_state = ENGINE_ON;
                    // �떆�뒪�뀥 議곗옉 紐⑤뱶濡� �쟾�솚
                    //vehicle_status.user_mode = SYSTEM_DRIVE_MODE;

                    //二쇱감�븳 李⑤웾�씠 �엳�쓣 �븣 異쒖감 �슂泥�// �뵳 �븳踰덈쭔 二쇱감-異쒖감
                    if (U8PrkFinished==1)
                    {
                        ExitCAR_request=1;
                    }
                }
            }
        }
#endif
    }

    return 0;
}


void make_can_message(void)
{
    update_message_vehicle_status(&db_msg.VCU_Vehicle_Status, &vehicle_status);
    output_message(&db_msg.VCU_Vehicle_Status, VCU_Vehicle_Status_ID);

    update_message_parking_status(&db_msg.VCU_Parking_Status, &vehicle_status);
    output_message(&db_msg.VCU_Parking_Status, VCU_Parking_Status_ID);

    update_message_engine_status(&db_msg.VCU_Vehicle_Engine_Status, &vehicle_status);
    output_message(&db_msg.VCU_Vehicle_Engine_Status, VCU_Vehicle_Engine_Status_ID);

    if (CameraSwitchRequest != 0) // 1 : �쟾諛�, 2 : �썑諛�
    {
        db_msg.VCU_Camera.camera_num = CameraSwitchRequest;
        output_message(&db_msg.VCU_Camera, VCU_Camera_ID);
        CameraSwitchRequest = 0;
    }
    if (lanecheck_request != 0) {
        //�젽�뒯�뿉�꽌 二쇱감�젙吏��꽑�솗�씤 �슂泥�
    }
}


void update_message_vehicle_status(VCU_Vehicle_Status_Msg* dest, const VehicleStatus* src)
{
    dest->vehicle_velocity = vehicle_status.u8_velocity;
    dest->vehicle_steering_angle = vehicle_status.steering_angle;
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
}

void AppTask50ms(void){
    stTestCnt.u32nuCnt50ms++;
    decision_stateflow_step();  //二쇳뻾紐⑤뱶�뿉 �뵲瑜� 醫낇슒 input 寃곗젙
}

void AppTask100ms(void)
{
    stTestCnt.u32nuCnt100ms++;
    update_VCU_inputs();  //紐⑦꽣�뿉 �젣�뼱input �쓣 �꽔�뼱以�
    now_status = imuRead();
    stanelytheta = now_euler.yaw;
    if (stopstatus == 1)
    {
        q0 = 1;
        q1 = 0;
        q2 = 0;
        q3 = 0;
        now_euler.yaw = 0;
    }
    now_euler = MadgwickAHRSupdateIMU(now_status);
    print_encimu(&now_status, &now_euler);

#if (!defined(motor_Test) && !defined(tuning_Test) && !defined(putty_Test)) // 二쇳뻾 肄붾뱶
    if (vehicle_status.engine_state == ENGINE_ON)
    {

        //�떆�룞�씠 耳쒖졇�엳�쓣 �븣, can message 異쒕젰
        make_can_message();
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
        if (stSchedulingInfo.u8nuScheduling50msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling50msFlag = 0u;
            AppTask50ms();
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
