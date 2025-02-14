/**********************************************************************************************************************
 * \file OurCan.h
 * \copyright Copyright (C) Infineon Technologies AG 2019
 *
 * Use of this file is subject to the terms of use agreed between (i) you or the company in which ordinary course of
 * business you are acting and (ii) Infineon Technologies AG or its licensees. If and as long as no such terms of use
 * are agreed, use of this file is subject to following:
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization obtaining a copy of the software and
 * accompanying documentation covered by this license (the "Software") to use, reproduce, display, distribute, execute,
 * and transmit the Software, and to prepare derivative works of the Software, and to permit third-parties to whom the
 * Software is furnished to do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including the above license grant, this restriction
 * and the following disclaimer, must be included in all copies of the Software, in whole or in part, and all
 * derivative works of the Software, unless such copies or derivative works are solely in the form of
 * machine-executable object code generated by a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *********************************************************************************************************************/

#ifndef OURCAN_H_
#define OURCAN_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
#include "OurCan_message.h"
#include "IfxMultican_Can.h"
#include "IfxPort_PinMap.h"
#include "IfxPort.h"
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/

// CAN 노드 및 메시지 핸들러 선언
#define CAN_TX_MESSAGE_ID 0x100
#define CAN_RX_MESSAGE_ID 0x123

// CAN RXTX 정의
#define TC275_CAN0 IfxMultican_SrcId_0
#define CAN0_RX IfxMultican_RXD0B_P20_7_IN
#define CAN0_TX IfxMultican_TXD0_P20_8_OUT

#define EIGHTBYTE_F 0xFFFFFFFFFFFFFFFF
#define FOURBYTE_F 0xFFFFFFFF

/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/

/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
typedef struct
{
        VCU_Vehicle_Status_Msg VCU_Vehicle_Status;
        VCU_Parking_Status_Msg VCU_Parking_Status;
        VCU_Vehicle_Engine_Status_Msg VCU_Vehicle_Engine_Status;
        VCU_Camera_Msg  VCU_Camera;
        SCU_Obstacle_Detection_Msg SCU_Obstacle_Detection;
        CGW_OTA_File_Size_Msg CGW_OTA_File_Size;
        CGW_OTA_File_Data_Msg CGW_OTA_File_Data;
        CGW_OTA_Control_Msg CGW_OTA_Control;
        CCU_Cordi_data1_Msg CCU_Cordi_data1;
        CCU_Cordi_data2_Msg CCU_Cordi_data2;
        CGW_Engine_Msg CGW_Engine;
        CGW_Move_Msg CGW_Move;
        CGW_Auto_Parking_Request_Msg CGW_Auto_Parking_Request;
        CGW_Off_Request_Msg CGW_Off_Request;
        CTRL_Engine_Msg CTRL_Engine;
        CTRL_Move_Msg CTRL_Move;
        CTRL_Auto_Parking_Request_Msg CTRL_Auto_Parking_Request;
        CTRL_Off_Request_Msg CTRL_Off_Request;

} DBMessages;

extern DBMessages db_msg;

typedef struct
{
        uint8 VCU_Vehicle_Status_Flag : 1;
        uint8 VCU_Parking_Status_Flag : 1;
        uint8 VCU_Vehicle_Engine_Status_Flag : 1;
        uint8  VCU_Camera_Flag : 1;
        uint8 SCU_Obstacle_Detection_Flag : 1;
        uint8 CGW_OTA_File_Size_Flag : 1;
        uint8 CGW_OTA_File_Data_Flag : 1;
        uint8 CGW_OTA_Control_Flag : 1;
        uint8 CCU_Cordi_data1_Flag : 1;
        uint8 CCU_Cordi_data2_Flag : 1;
        uint8 CGW_Engine_Flag : 1;
        uint8 CGW_Move_Flag : 1;
        uint8 CGW_Auto_Parking_Request_Flag : 1;
        uint8 CGW_Off_Request_Flag : 1;
        uint8 CTRL_Engine_Flag : 1;
        uint8 CTRL_Move_Flag : 1;
        uint8 CTRL_Auto_Parking_Request_Flag : 1;
        uint8 CTRL_Off_Request_Flag : 1;

} DBFlag;

extern DBFlag db_flag;

// Transmission 열거형 (대문자로 변경)
typedef enum
{
    PARKING = 0,
    DRIVING = 1,
    REVERSE = 2
} Transmission;

// EngineState 열거형 (대문자로 변경)
typedef enum
{
    ENGINE_OFF = 0,
    ENGINE_ON = 1
} EngineState;

// Mode 열거형 (대문자로 변경)
typedef enum
{
    SYSTEM_DRIVE_MODE = 0,
    USER_DRIVE_MODE = 1
} Mode;

// ParkingStatus 열거형 (대문자로 변경)
typedef enum
{
    SERCHING = 0,
    POSITIONING = 1, // 주차 공간 인식 후 전진하는 상태
    REVERSING = 2,   // 후진 주차 중
    PARKED = 3       // 주차 완료
} ParkingStatus;


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

extern VehicleStatus vehicle_status;


/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
void initCanDB(void);
void output_message(void *msg, uint32 msgID);
void output_message_test(int cnt);
void initCan(void);
/*********************************************************************************************************************/

#endif /* OURCAN_H_ */
