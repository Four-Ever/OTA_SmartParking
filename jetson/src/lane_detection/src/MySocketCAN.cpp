#include "lane_detection/MySocketCAN.h"
#include "lane_detection/Protocol.h"
#include "lane_detection/CentralGateway.h"
#include "lane_detection/MyWebSocketSession.h"
#include "lane_detection/vision_node.hpp"
#include <iostream>
#include <thread>
#include <cstdint>
#include <atomic>

MySocketCAN::MySocketCAN(boost::asio::io_context& io_context, std::shared_ptr<CentralGateway> owner)
    : SocketCAN(io_context), owner_(owner) {
}

bool MySocketCAN::Init(const std::string& interface_name) {
    if (!SocketCAN::Init(interface_name)) {
        return false;
    }
    std::cout << "MySocketCAN 초기화 " << interface_name << std::endl;
    return true;
}

void MySocketCAN::on_receive_can(const can_frame& frame) {
    //std::cout << "Thread " << std::this_thread::get_id() 
              //<< " - MyCANSocket received frame - ID: 0x" 
              //<< std::hex << frame.can_id << std::endl;
    int32_t id = frame.can_id;
    std::shared_ptr<IMessage> msg;

    switch (id)
    {
    case VCU_Vehicle_Status_ID:
    {
        msg = std::make_shared<CGW_Vehicle_Status_Msg>();                   
        memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
        break;
    }
    case VCU_Parking_Status_ID:
    {
        msg = std::make_shared<CGW_Parking_Status_Msg>();
        memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
        break;
    }
    case VCU_Vehicle_Engine_Status_ID:
    {
        msg = std::make_shared<VCU_Vehicle_Engine_Status_Msg>();
        memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
        break;
    }   
    case VCU_Camera_ID:
    {
        msg = std::make_shared<VCU_Camera_Msg>();
        memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());

        int camera_num = std::static_pointer_cast<VCU_Camera_Msg>(msg)->data_.data.camera_num;
        if (vision_node) {
            if(camera_num == 0)
            {
                vision_node->set_parameter(rclcpp::Parameter("mode", "driving"));
            }
            else if(camera_num == 1)
            {
                vision_node->set_parameter(rclcpp::Parameter("mode", "parking"));
            }
        }
        return;
    }         
    case SCU_Obstacle_Detection_ID:
    {
        msg = std::make_shared<SCU_Obstacle_Detection_Msg>();
        memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
        break;
    }  
    // case CGW_OTA_File_Size_ID:
    // {
    //     msg = std::make_shared<CGW_OTA_File_Size_Msg>();
    //     memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
    //     break;
    // }  
    // case CGW_OTA_File_Data_ID:
    // {
    //     msg = std::make_shared<CGW_OTA_File_Data_Msg>();
    //     memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
    //     break;
    // } 
    // case CGW_OTA_Control_ID:
    // {
    //     msg = std::make_shared<CGW_OTA_Control_Msg>();
    //     memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
    //     break;
    // } 
    // case CGW_OTA_Update_Request_ID:
    // {
    //     msg = std::make_shared<CGW_OTA_Update_Request_Msg>();
    //     memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
    //     break;
    // } 
    // case CGW_OTA_Update_State_ID:
    // {
    //     msg = std::make_shared<CGW_OTA_Update_State_Msg>();
    //     memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
    //     break;
    // }
    // case CGW_Vehicle_Status_ID:
    // {
    //     msg = std::make_shared<CGW_Vehicle_Status_Msg>();
    //     memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
    //     break;
    // }
    // case CCU_Cordi_data1_ID:
    // {
    //     msg = std::make_shared<CCU_Cordi_data1_Msg>();
    //     memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
    //     break;
    // }   
    // case CCU_Cordi_data2_ID:
    // {
    //     msg = std::make_shared<CCU_Cordi_data2_Msg>();
    //     memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
    //     break;
    // } 
    // case CTRL_Engine_ID:
    // {
    //     msg = std::make_shared<CTRL_Engine_Msg>();
    //     memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
    //     break;
    // } 
    // case CTRL_Move_ID:
    // {
    //     msg = std::make_shared<CTRL_Move_Msg>();
    //     memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
    //     break;
    // } 
    // case CTRL_Auto_Parking_Request_ID:
    // {
    //     msg = std::make_shared<CTRL_Auto_Parking_Request_Msg>();
    //     memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
    //     break;
    // } 
    // case CTRL_OTA_Update_Confirm_ID:
    // {
    //     msg = std::make_shared<CTRL_OTA_Update_Confirm_Msg>();
    //     memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
    //     break;
    // }
    // case CTRL_Off_Request_ID:
    // {
    //     msg = std::make_shared<CTRL_Off_Request_Msg>();
    //     memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
    //     break;
    // }         
    default:
        break;       
    }
    
    // auto owner = owner_.lock();
    // if (owner) {
    //     auto session = owner->ws_server_->sessions_.back();
    //     session->send_message(msg);
    // }

    //CGW->ws_server_->send_message(msg);
    
    auto ws_session = std::atomic_load(&(CGW->ws_server_->current_session_));
    ws_session->send_message(msg);

    //ws_queue_.Push(msg);
    // if (frame.can_id == 0x123) {
    //     std::cout << "Special frame received!" << std::endl;
    //     std::cout << "Data: ";
    //     for (int i = 0; i < frame.can_dlc; i++) {
    //         std::cout << std::hex << (int)frame.data[i] << " ";
    //     }
    //     std::cout << std::endl;
    // }
}

void MySocketCAN::on_send_can(const boost::system::error_code& error, std::size_t bytes_transferred) {
    if (!error) {
        // std::cout << "Thread " << std::this_thread::get_id() 
        //          << " - MyCANSocket successfully sent " << bytes_transferred << " bytes" << std::endl;
    } 
    else {
        std::cerr << "Thread " << std::this_thread::get_id() 
                 << " - MyCANSocket send error: " << error.message() << std::endl;
    }
}