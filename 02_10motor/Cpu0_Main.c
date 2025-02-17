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

//#include "Ifx_IntPrioDef.h"
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
//#define motor_Test //���ڴ� �Ÿ� Ȯ��
//#define putty_Test //putty uart �޼��� Ȯ��
//#define tuning_Test //�ùĸ�ũ���� pid ��� ���� �� ������ Ȯ��
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
IfxI2c_I2c_Status first = 4;
IfxI2c_I2c_Status second = 4;
IfxI2c_I2c_Status third = 4;
IfxI2c_I2c_Status fourth = 4;
IfxI2c_I2c_Status test1 = 4;
IfxI2c_I2c_Status test2 = 4;
IfxI2c_I2c_Status test3 = 4;
IfxI2c_I2c_Status test4 = 4;
IfxI2c_I2c_Status test5 = 4;
IfxI2c_I2c_Status test6 = 4;
IfxI2c_I2c_Status test7 = 4;
IfxI2c_I2c_Status test8 = 4;
//////////////////////////////////
IMU now_status = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
Euler now_euler = { 0, 0, 0 };
uint8 in = 0;
uint8 now13 = 0;
// ak status check reg
uint8 status1_val = 0;
uint8 status2_val = 5;
float asa_x = 0;
float asa_y = 0;
float asa_z = 0;
int i = 0;
float scale_x = 0.0f;
float scale_y = 0.0f;
float scale_z = 0.0f;
float x_offset = 0.0f;
float y_offset = 0.0f;
float z_offset = 0.0f;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame
int nowcnt = 0;
float nowtheta = 0.0f;
float stanelytheta = 0.0f;
void Touch(void);
void initGPIO(void);
IfxPort_State TouchState = 0;
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
    now13 = 1;

    in = 5;
    initGPIO();
    // motor_dir = 0;    // 0:������, 1:������
    // motor_enable = 0;  // 0:����, 1:����

#ifdef motor_Test
    // motor_enable = 1;  // 0:����, 1:����

    Kp_s = 1.55f;//1.75f;
    Ki_s = 2.65f;//0.198f;
    Kd_s = 0.001f;

    vehicle_status.engine_state = engine_on;

    waitTime(300000000); // 3��
    init_move_distance_control(1000.0f, 500.0f); // 1000mm, 1000rpm
#endif

    while(1)
    {
        AppScheduling();

        //can �޼��� �޴� ��
#if !defined(motor_Test) && !defined(tuning_Test) && !defined(putty_Test) // ���� �ڵ�
        // ���� �޼���
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

        // ������ ���� ���� ��,
        if (vehicle_status.engine_state == ENGINE_ON)
        {
            // �������� �޼���
            if (db_flag.CGW_Move_Flag == 1)
            {
                db_flag.CGW_Move_Flag = 0;

                //D_trans = db_msg.CGW_Move.control_transmission;
                U8IsTrButton = db_msg.CGW_Move.control_transmission;
                //vehicle_status.transmission = db_msg.CGW_Move.control_transmission;
                
                D_steering = db_msg.CGW_Move.control_steering_angle;
                //vehicle_status.steering_angle = db_msg.CGW_Move.control_steering_angle;

                // ���� ���� ���� ��ȯ
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

            // �ڵ� ���� ��û

            if (db_flag.CGW_Auto_Parking_Request_Flag==1)
            {
                db_flag.CGW_Auto_Parking_Request_Flag=0;

                IsRSPAButton = 1;

            }
            //wp ����Ȯ��
            if (db_flag.CCU_Cordi_data1_Flag == 1 && db_flag.CCU_Cordi_data2_Flag == 1) {
                // ��� �����Ͱ� ���� ���
                db_flag.CCU_Cordi_data1_Flag=0;
                db_flag.CCU_Cordi_data2_Flag=0;

                // ī�޶� �ȼ� ��ǥ ����
                //if (db_msg.CCU_Cordi_data2.trust_value > 0.7){  //�ŷڵ� ������츸 ���� �����.
                    // ���� ������ �ʱ�ȭ
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
                if (U8RSPAState == Searching || U8RSPAState == Backward_Assist) {  //������� ���� ���� �ΰ��ۿ� ����
                    if (data_ready_flag == 1){
                        InitWorldpoints();

                        if(First_Set==1){   //�� ó���� wp ������
                            initStanley();
                            transform_points(H, cam_points, world_points); //��ǥ�� ��ȯ
                            if (transform_finished==1){
                                updateWaypoints(world_points);  //stanely �� ��ǥ waypoint ����
                            }
                            First_Set=0;
                            if (U8RSPAState==Searching){
                                lanecheck_request=1;
                            }
                        }
                        else if (IsWPTrackingFinish==1){ // �� ���Ŀ��� ���� wp ��� ���������� ���� ������.
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


            //������ ����
            /*if (db_flag.CCU_RightAngle_detect_flag == 1)
            {
                db_flag.CCU_RightAngle_detect_flag = 0;
                U8IsStopline = db_flag.CCU_RightAngle_detect.right_angle_lane_detected;
            }*/
        }

        //������ ���� ���� ��,
        else if (vehicle_status.engine_state == ENGINE_OFF)
        {          
            if (db_flag.CGW_Off_Request_Flag==1)
            {
                db_flag.CGW_Off_Request_Flag=0;

                // ���� ã�� ��û
                if (db_msg.CGW_Off_Request.alert_request==1)
                {
                    //������ ������ ������ LED Ȥ�� ���� �߿�߿�
                    if (U8PrkFinished==1)
                    {

                    }
                }

                // ���� ��û
                if (db_msg.CGW_Off_Request.auto_exit_request==1)
                {
                    vehicle_status.engine_state = ENGINE_ON;
                    // �ý��� ���� ���� ��ȯ
                    //vehicle_status.user_mode = SYSTEM_DRIVE_MODE;

                    //������ ������ ���� �� ���� ��û// �� �ѹ��� ����-����
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

    if (CameraSwitchRequest != 0) // 1 : ����, 2 : �Ĺ�
    {
        db_msg.VCU_Camera.camera_num = CameraSwitchRequest;
        output_message(&db_msg.VCU_Camera, VCU_Camera_ID);
        CameraSwitchRequest = 0;
    }
    if (lanecheck_request != 0) {
        //�������� ����������Ȯ�� ��û
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
    decision_stateflow_step();  //�����忡 ���� ��Ⱦ input ����
}

void AppTask100ms(void)
{
    stTestCnt.u32nuCnt100ms++;
    update_VCU_inputs();  //���Ϳ� ����input �� �־���
    i++;
    now_status = imuRead();
    stanelytheta = nowtheta + now_euler.yaw;
    if (stopstatus == 1)
    {
        q0 = 1;
        q1 = 0;
        q2 = 0;
        q3 = 0;
        nowtheta += now_euler.yaw;
        now_euler.yaw = 0;
    }
    now_euler = MadgwickAHRSupdateIMU(now_status);
    print_encimu(&now_status, &now_euler);

#if (!defined(motor_Test) && !defined(tuning_Test) && !defined(putty_Test)) // ���� �ڵ�
    if (vehicle_status.engine_state == ENGINE_ON)
    {

        //�õ��� �������� ��, can message ���
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
void initGPIO(void)
{
    IfxPort_setPinMode(&MODULE_P14, 0, IfxPort_Mode_inputPullUp);  //input?�로 ?�정
}

void Touch(void)
{
    TouchState = IfxPort_getPinState(&MODULE_P14, 0);
    if (TouchState == 1)
        stopstatus = 1;
    else
        stopstatus = 0;
}
/*********************************************************************************************************************/
