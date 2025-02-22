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
#include "Obstacle_Detection.h"
#include "LED_Buzzer.h"

//#include "Ifx_IntPrioDef.h"
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
//#define motor_Test //
//#define putty_Test //
//#define tuning_Test //
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
    uint32 u32nuCnt500ms;
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
void make_can_message (void);
void update_message_vehicle_status (VCU_Vehicle_Status_Msg *dest, const VehicleStatus *src);
void update_message_parking_status (VCU_Parking_Status_Msg *dest, const VehicleStatus *src);
void update_message_engine_status (VCU_Vehicle_Engine_Status_Msg *dest, const VehicleStatus *src);
void initIMU_error (void);

void AppScheduling (void);
void AppTask1ms (void);
void AppTask10ms (void);
void AppTask50ms (void);
void AppTask100ms (void);
void AppTask500ms (void);
void AppTask1000ms (void);
void AppTask5000ms (void);
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/

// Interrupt handler for zero crossing
//IFX_INTERRUPT(ISR_IncrIncZero, 0, ISR_PRIORITY_INCRENC_ZERO)
//{
//    IfxGpt12_IncrEnc_onZeroIrq(&gpt12Config);
//}

float g_angle;
IMU now_status = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
Euler now_euler = {0, 0, 0};
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame
float stanelytheta = 0.0f;
int stopstatus = 0;
//int md_flag=0;


int core0_main (void)
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
    //initGPIO();

    //ToF user manual !
    Init_ToF(); // init
    initUltrasonic();

    init_LED_Buzzer();

    alarm_request = 1;
    initStanley();

    initIMU();
    waitTime(100000000);
    //U8Driver=ModeOn;

    IsRSPAButton = 1;
#ifdef motor_Test
    // motor_enable = 1;
    Kp_s = 1.55f;//1.75f;
    Ki_s = 2.65f;//0.198f;
    Kd_s = 0.001f;

    vehicle_status.engine_state = engine_on;

    waitTime(300000000); //
    init_move_distance_control(1000.0f, 500.0f); // 1000mm, 1000rpm
#endif



    while (1)
    {


        AppScheduling();
        //stopstatus=Touch();
        //can msg 占쎈땾占쎈뻿
#if !defined(motor_Test) && !defined(tuning_Test) && !defined(putty_Test) // 占쎌뜏占쎌뒩占쎈떁嶺뚮쵓�삕 �뜝�럡��占쎄껀占쎈폇�뤃占�
        // 占쎈퓦筌욑옙 on/off
        if (db_flag.CGW_Engine_Flag == 1)
        {
            db_flag.CGW_Engine_Flag = 0;

            //if (U8DriverState == Parking || U8RSPAState==Parking_Complete || U8DriverState == InitDriverState ){
                vehicle_status.engine_state = db_msg.CGW_Engine.control_engine;
           // }

        }
        //占쎈퓦筌욑옙 ON
//        if (vehicle_status.engine_state == ENGINE_ON)
        {
            //driver mode
            if (db_flag.CGW_Move_Flag == 1)
            {
                db_flag.CGW_Move_Flag = 0;
                U8IsTrButton = db_msg.CGW_Move.control_transmission;
                D_steering = db_msg.CGW_Move.control_steering_angle;

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

            //RSPA mode
            if (db_flag.CGW_Auto_Parking_Request_Flag==1)
            {
                db_flag.CGW_Auto_Parking_Request_Flag=0;

                IsRSPAButton = 1;
            }

            //waypoint msg
            if (db_flag.CCU_Cordi_data1_Flag == 1 && db_flag.CCU_Cordi_data2_Flag == 1) {
                //嶺뚮ㅄ維�獄�占� wp 嶺뚮∥�뾼�땻占썹춯�쉻�삕占쎈ご�뜝占� �뜝�럥�빢�뜝�럥六욕뜝�럥六ε뜝�럩諭� �뜝�럥瑜�
                db_flag.CCU_Cordi_data1_Flag=0;
                db_flag.CCU_Cordi_data2_Flag=0;

                //
                //if (db_msg.CCU_Cordi_data2.trust_value > 0.7){
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
                if (U8RSPAState == Searching || U8RSPAState == Backward_Assist) {  //lane detection mode
                    if (data_ready_flag == 1){
                        InitWorldpoints();

                        if(First_Set==1){    //
                            initStanley();
                            initIMU_error();
                            transform_points(H, cam_points, world_points);
                            if (transform_finished==1){
                                updateWaypoints(world_points);
                            }
                            First_Set=0;
                            if (U8RSPAState==Searching){
                                lanecheck_request=1;
                            }
                        }
                        else if (IsWPTrackingFinish==1){    //
                            initStanley();
                            initIMU_error();
                            transform_points(H, cam_points, world_points);
                            if (transform_finished==1){
                                updateWaypoints(world_points);
                            }
                        }
                        U8IsWp_R=camera_mode;
                    }
                }
            }


            //lane detecion_Coner
            if (db_flag.CCU_RightAngle_detect_Flag == 1)
            {
                db_flag.CCU_RightAngle_detect_Flag = 0;
                U8IsConerline = db_msg.CCU_RightAngle_detect.right_angle_lane_detected;
            }
        }

        //engine off
//        else if (vehicle_status.engine_state == ENGINE_OFF)
//        {
//            if (db_flag.CGW_Off_Request_Flag==1)
//            {
//                db_flag.CGW_Off_Request_Flag=0;
//
//                //
//                if (db_msg.CGW_Off_Request.alert_request==1)
//                {
//                    //find my car_LED/Sound
//                    if (U8PrkFinished==1)
//                    {
//
//                    }
//                }
//
//                //�빊�뮇媛�
//                if (db_msg.CGW_Off_Request.auto_exit_request==1)
//                {
//                    vehicle_status.engine_state = ENGINE_ON;
//
//                    if (U8PrkFinished==1)
//                    {
//                        ExitCAR_request=1;
//                    }
//                }
//            }
//        }
#endif
    }

    return 0;
}

void make_can_message (void)
{
    update_message_vehicle_status(&db_msg.VCU_Vehicle_Status, &vehicle_status);
    output_message(&db_msg.VCU_Vehicle_Status, VCU_Vehicle_Status_ID);

    //update_message_parking_status(&db_msg.VCU_Parking_Status, &vehicle_status);
    //output_message(&db_msg.VCU_Parking_Status, VCU_Parking_Status_ID);

    update_message_engine_status(&db_msg.VCU_Vehicle_Engine_Status, &vehicle_status);
    output_message(&db_msg.VCU_Vehicle_Engine_Status, VCU_Vehicle_Engine_Status_ID);

    if (CameraSwitchRequest != 0) // 1 : front camera, 2 : rear carmera
    {
        db_msg.VCU_Camera.camera_num = CameraSwitchRequest;
        output_message(&db_msg.VCU_Camera, VCU_Camera_ID);
        CameraSwitchRequest = 0;
    }
    if (lanecheck_request != 0)
    {
        db_msg.VCU_ParkingLane_Request.Lane_Request = lanecheck_request;
        output_message(&db_msg.VCU_ParkingLane_Request, VCU_ParkingLane_Request_ID);
        lanecheck_request = 0;
    }
    if (U8RSPAState != InitRSPAState)
    {
        db_msg.VCU_Parking_Status.parking_status = ToController_Prkstate;
        output_message(&db_msg.VCU_Parking_Status, VCU_Parking_Status_ID);
    }

    db_msg.VCU_Exiting_Status.exiting_status = ToController_Exitstate;
    output_message(&db_msg.VCU_Exiting_Status, VCU_Exiting_Status_ID);

}

void update_message_vehicle_status (VCU_Vehicle_Status_Msg *dest, const VehicleStatus *src)
{
    dest->vehicle_velocity = src->u8_velocity;
    dest->vehicle_steering_angle = src->steering_angle;
    dest->vehicle_transmission = src->transmission;
}

//void update_message_parking_status(VCU_Parking_Status_Msg* dest, const VehicleStatus* src)
//{
//    dest->parking_status = src->parking_status;
//}

void update_message_engine_status (VCU_Vehicle_Engine_Status_Msg *dest, const VehicleStatus *src)
{
    dest->vehicle_engine_status = src->engine_state;
}

void initIMU_error (void)
{

    {
        q0 = 1;
        q1 = 0;
        q2 = 0;
        q3 = 0;
        now_euler.yaw = 0;
    }
}


void AppTask1ms (void)
{
    stTestCnt.u32nuCnt1ms++;
}

void AppTask10ms (void)
{
    stTestCnt.u32nuCnt10ms++;
    now_status = imuRead();
    stanelytheta = now_euler.yaw;
    /*if (stopstatus == 1)
     {
     q0 = 1;
     q1 = 0;
     q2 = 0;
     q3 = 0;
     now_euler.yaw = 0;
     }*/
    now_euler = MadgwickAHRSupdateIMU(now_status);


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

void AppTask50ms (void)
{
    stTestCnt.u32nuCnt50ms++;
    Obstacle_get_All_Distance();
    decision_stateflow_step_c();

}

void AppTask100ms (void)
{
    stTestCnt.u32nuCnt100ms++;
    //parking_coded_input();

    update_VCU_inputs_c();
//    setServoAngle(gitstanley());
//    RPM_CMD1 = stanleytref_vel;
//    now_status.accel_x = x;
//    now_status.accel_y = y;
//    now_status.accel_z = (float)current_wp_idx;
//    now_status.gyro_x = waypointsT[current_wp_idx][0];
//    now_status.gyro_y = waypointsT[current_wp_idx][1];
//
//    print_encimu(&now_status, &now_euler);
//
//

//#if (!defined(motor_Test) && !defined(tuning_Test) && !defined(putty_Test)) //
    if (vehicle_status.engine_state == ENGINE_ON)
    {
        //send message
        make_can_message();
    }
//#endif
}

void AppTask500ms (void)
{
    stTestCnt.u32nuCnt500ms++;

    FindCar_Plz();
    cnt_alarm++;
}

void AppTask1000ms (void)
{
    stTestCnt.u32nuCnt1000ms++;
}

void AppTask5000ms (void)
{
    stTestCnt.u32nuCnt5000ms++;
}

void AppScheduling (void)
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

        if (stSchedulingInfo.u8nuScheduling500msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling500msFlag = 0u;
            AppTask500ms();
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
