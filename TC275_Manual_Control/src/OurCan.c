/**********************************************************************************************************************
 * \file OurCan.c
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

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
#include "OurCan.h"
#include "Platform_Types.h"
#include <string.h>
#include "ASCLIN_Shell_UART.h"
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/

/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/

//    user_mode, target_rpm , target_vel, cur_rpm, cur_vel,
//    uint8 velocity, sint8 steering_angle, transmission, parking_status, engine_state

VehicleStatus vehicle_status = {SYSTEM_DRIVE_MODE, 0.0f, 0.0f, 0, 0.0f,
                                    0, 0, PARKING, PARKED, ENGINE_OFF};
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
IfxMultican_Can can;
IfxMultican_Can_Node canNode;
IfxMultican_Can_MsgObj canMsgObjTx;
IfxMultican_Can_MsgObj canMsgObjRx;

DBMessages db_msg;
DBFlag db_flag;

/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
IFX_INTERRUPT(RX_Int0Handler, 0, 101);
// void RX_Int0Handler (void){}
void RX_Int0Handler(void)
{
    IfxCpu_enableInterrupts();

    IfxMultican_Message readmsg;
    //    while (!IfxMultican_Can_MsgObj_isRxPending(&canMsgObjRx)){}// 수신 대기
    if (IfxMultican_Can_MsgObj_readMessage(&canMsgObjRx, &readmsg) == IfxMultican_Status_newData)
    {
        switch (readmsg.id)
        {
            case CGW_OTA_File_Size_ID:
            {
                uint32 serialized = 0;
                memcpy(&serialized,&readmsg.data[0],CGW_OTA_File_Size_Size);
                Deserialize_CGW_OTA_File_Size_Msg(&serialized,&db_msg.CGW_OTA_File_Size);
                db_flag.CGW_OTA_File_Size_Flag = 1;
                break;
            }
            case CGW_OTA_File_Data_ID:
            {
                uint64 serialized = 0;
                memcpy(&serialized,&readmsg.data[0],CGW_OTA_File_Data_Size);
                //memcpy((uint32*)&serialized+1,&readmsg.data[1],CGW_OTA_File_Data_Size-4);
                Deserialize_CGW_OTA_File_Data_Msg(&serialized,&db_msg.CGW_OTA_File_Data);
                db_flag.CGW_OTA_File_Data_Flag = 1;
                break;
            }
            case CGW_OTA_Control_ID:
            {
                uint64 serialized = 0;
                memcpy(&serialized,&readmsg.data[0],CGW_OTA_Control_Size);
                //memcpy((uint32*)&serialized+1,&readmsg.data[1],CGW_OTA_Control_Size-4);
                Deserialize_CGW_OTA_Control_Msg(&serialized,&db_msg.CGW_OTA_Control);
                db_flag.CGW_OTA_Control_Flag = 1;
                break;
            }
            case CCU_Cordi_data1_ID:
            {
                uint64 serialized = 0;
                memcpy(&serialized,&readmsg.data[0],CCU_Cordi_data1_Size);
                //memcpy((uint32*)&serialized+1,&readmsg.data[1],CCU_Cordi_data1_Size-4);
                Deserialize_CCU_Cordi_data1_Msg(&serialized,&db_msg.CCU_Cordi_data1);
                db_flag.CCU_Cordi_data1_Flag = 1;
                break;
            }
            case CCU_Cordi_data2_ID:
            {
                uint64 serialized = 0;
                memcpy(&serialized,&readmsg.data[0],CCU_Cordi_data2_Size);
                //memcpy((uint32*)&serialized+1,&readmsg.data[1],CCU_Cordi_data2_Size-4);
                Deserialize_CCU_Cordi_data2_Msg(&serialized,&db_msg.CCU_Cordi_data2);
                db_flag.CCU_Cordi_data2_Flag = 1;
                break;
            }
            case CGW_Engine_ID:
            {
                uint8 serialized = 0;
                memcpy(&serialized,&readmsg.data[0],CGW_Engine_Size);
                Deserialize_CGW_Engine_Msg(&serialized,&db_msg.CGW_Engine);
                db_flag.CGW_Engine_Flag = 1;
                break;
            }
            case CGW_Move_ID:
            {
                uint16 serialized = 0;
                memcpy(&serialized,&readmsg.data[0],CGW_Move_Size);
                Deserialize_CGW_Move_Msg(&serialized,&db_msg.CGW_Move);
                db_flag.CGW_Move_Flag = 1;

//                int a = db_msg.CGW_Move.control_accel;
//                int b = db_msg.CGW_Move.control_brake;
//                int c = db_msg.CGW_Move.control_steering_angle;
//                int d = db_msg.CGW_Move.control_transmission;
//                myprintf("%d %d %d %d\n\r",a,b,c,d);

                break;
            }
            case CGW_Auto_Parking_Request_ID:
            {
                uint8 serialized = 0;
                memcpy(&serialized,&readmsg.data[0],CGW_Auto_Parking_Request_Size);
                Deserialize_CGW_Auto_Parking_Request_Msg(&serialized,&db_msg.CGW_Auto_Parking_Request);
                db_flag.CGW_Auto_Parking_Request_Flag = 1;
                break;
            }
            case CGW_Off_Request_ID:
            {
                uint8 serialized = 0;
                memcpy(&serialized,&readmsg.data[0],CGW_Off_Request_Size);
                Deserialize_CGW_Off_Request_Msg(&serialized,&db_msg.CGW_Off_Request);
                db_flag.CGW_Off_Request_Flag = 1;
                break;
            }

            case CTRL_Engine_ID:
            {
                uint8 serialized = 0;
                memcpy(&serialized,&readmsg.data[0],CTRL_Engine_Size);
                Deserialize_CTRL_Engine_Msg(&serialized,&db_msg.CTRL_Engine);
                db_flag.CTRL_Engine_Flag = 1;
                break;
            }
            case CTRL_Move_ID:
            {
                uint16 serialized = 0;
                memcpy(&serialized,&readmsg.data[0],CTRL_Move_Size);
                Deserialize_CTRL_Move_Msg(&serialized,&db_msg.CTRL_Move);
                db_flag.CTRL_Move_Flag = 1;
                break;
            }
            case CTRL_Auto_Parking_Request_ID:
            {
                uint8 serialized = 0;
                memcpy(&serialized,&readmsg.data[0],CTRL_Auto_Parking_Request_Size);
                Deserialize_CTRL_Auto_Parking_Request_Msg(&serialized,&db_msg.CTRL_Auto_Parking_Request);
                db_flag.CTRL_Auto_Parking_Request_Flag = 1;
                break;
            }
            case CTRL_Off_Request_ID:
            {
                uint8 serialized = 0;
                memcpy(&serialized,&readmsg.data[0],CTRL_Off_Request_Size);
                Deserialize_CTRL_Off_Request_Msg(&serialized,&db_msg.CTRL_Off_Request);
                db_flag.CTRL_Off_Request_Flag = 1;
                break;
            }


        default:
            break;
        }
    }
}
void initCanDB(void)
{
    memset(&db_msg,0,sizeof(DBMessages));
    memset(&db_flag,0,sizeof(DBFlag));

}



void output_message(void *msg, uint32 msgID)
{
    IfxMultican_Message tx_msg;
    uint32 send_data[2] = {0};

    switch (msgID)
    {
        case VCU_Vehicle_Status_ID:
        {
            uint16 serialized = 0;
            VCU_Vehicle_Status_Msg* out_msg = (VCU_Vehicle_Status_Msg*)msg;
            Serialize_VCU_Vehicle_Status_Msg(&serialized, out_msg);
            memcpy(&send_data[0],&serialized,2);
            IfxMultican_Message_init(&tx_msg, VCU_Vehicle_Status_ID, send_data[0], send_data[1], VCU_Vehicle_Status_Size);
            break;
        }
        case VCU_Parking_Status_ID:
        {
            uint8 serialized = 0;
            VCU_Parking_Status_Msg* out_msg = (VCU_Parking_Status_Msg*)msg;
            Serialize_VCU_Parking_Status_Msg(&serialized, out_msg);
            memcpy(&send_data[0],&serialized,1);
            IfxMultican_Message_init(&tx_msg, VCU_Parking_Status_ID, send_data[0], send_data[1], VCU_Parking_Status_Size);
            break;
        }
        case VCU_Vehicle_Engine_Status_ID:
        {
            uint8 serialized = 0;
            VCU_Vehicle_Engine_Status_Msg* out_msg = (VCU_Vehicle_Engine_Status_Msg*)msg;
            Serialize_VCU_Vehicle_Engine_Status_Msg(&serialized, out_msg);
            memcpy(&send_data[0],&serialized,1);
            IfxMultican_Message_init(&tx_msg, VCU_Vehicle_Engine_Status_ID, send_data[0], send_data[1], VCU_Vehicle_Engine_Status_Size);
            break;
        }
        case VCU_Camera_ID:
        {
            uint8 serialized = 0;
            VCU_Camera_Msg* out_msg = (VCU_Camera_Msg*)msg;
            Serialize_VCU_Camera_Msg(&serialized, out_msg);
            memcpy(&send_data[0],&serialized,1);
            IfxMultican_Message_init(&tx_msg, VCU_Camera_ID, send_data[0], send_data[1], VCU_Camera_Size);
            break;
        }
        case SCU_Obstacle_Detection_ID:
        {
            uint8 serialized = 0;
            SCU_Obstacle_Detection_Msg* out_msg = (SCU_Obstacle_Detection_Msg*)msg;
            Serialize_SCU_Obstacle_Detection_Msg(&serialized, out_msg);
            memcpy(&send_data[0],&serialized,1);
            IfxMultican_Message_init(&tx_msg, SCU_Obstacle_Detection_ID, send_data[0], send_data[1], SCU_Obstacle_Detection_Size);
            break;
        }
        case CCU_Cordi_data1_ID:
        {
            uint64 serialized = 0;
            CCU_Cordi_data1_Msg* out_msg = (CCU_Cordi_data1_Msg*)msg;
            Serialize_CCU_Cordi_data1_Msg(&serialized, out_msg);
            //memcpy(&send_data[0],&serialized,4);
            //memcpy(&send_data[1],((uint32*)&serialized)+1,2);
            memcpy(send_data, &serialized, 6); // 하위 6바이트 복사
            IfxMultican_Message_init(&tx_msg, CCU_Cordi_data1_ID, send_data[0], send_data[1], CCU_Cordi_data1_Size);
            break;
        }
        default:
            break;
        }

        while (IfxMultican_Can_MsgObj_sendMessage(&canMsgObjTx, &tx_msg) == IfxMultican_Status_notSentBusy)
        {
        }
}

void initCan(void)
{
    // CAN 모듈 초기화
    IfxMultican_Can_Config canConfig;
    IfxMultican_Can_initModuleConfig(&canConfig, &MODULE_CAN);

    //     CAN0 인터럽트 활성화
    canConfig.nodePointer[TC275_CAN0].priority = 101;
    canConfig.nodePointer[TC275_CAN0].typeOfService = IfxSrc_Tos_cpu0;

    IfxMultican_Can_initModule(&can, &canConfig);

    // CAN 노드 초기화
    IfxMultican_Can_NodeConfig canNodeConfig;
    IfxMultican_Can_Node_initConfig(&canNodeConfig, &can);
    canNodeConfig.nodeId = IfxMultican_NodeId_0;
    canNodeConfig.rxPin = &CAN0_RX;
    canNodeConfig.rxPinMode = IfxPort_InputMode_pullUp;
    canNodeConfig.txPin = &CAN0_TX;
    canNodeConfig.txPinMode = IfxPort_OutputMode_pushPull;
    IfxMultican_Can_Node_init(&canNode, &canNodeConfig);

    // Tx 메시지 객체 초기화
    IfxMultican_Can_MsgObjConfig canMsgObjConfig;
    IfxMultican_Can_MsgObj_initConfig(&canMsgObjConfig, &canNode);
    canMsgObjConfig.msgObjId = 0; // 메시지 객체 ID
    canMsgObjConfig.messageId = CAN_TX_MESSAGE_ID;
    canMsgObjConfig.frame = IfxMultican_Frame_transmit;
    canMsgObjConfig.control.extendedFrame = FALSE;
    IfxMultican_Can_MsgObj_init(&canMsgObjTx, &canMsgObjConfig);

    // Rx 메시지 객체 초기화
    canMsgObjConfig.msgObjId = 1; // 메시지 객체 ID
    canMsgObjConfig.messageId = CAN_RX_MESSAGE_ID;
    canMsgObjConfig.acceptanceMask = 0x0; // 비교 안함, 전부 수신
    canMsgObjConfig.frame = IfxMultican_Frame_receive;
    canMsgObjConfig.control.extendedFrame = FALSE;

    // 인터럽트 활성화
    canMsgObjConfig.rxInterrupt.enabled = TRUE;
    canMsgObjConfig.rxInterrupt.srcId = TC275_CAN0;

    IfxMultican_Can_MsgObj_init(&canMsgObjRx, &canMsgObjConfig);
}

/*********************************************************************************************************************/
