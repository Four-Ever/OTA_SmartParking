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
    auto ws_session = CGW->ws_server_->current_session_;
    if (!ws_session) {
        return;    // 연결된 세션이 없으면 리턴
    }

    int32_t id = frame.can_id;
    std::shared_ptr<IMessage> msg;

    switch (id)
    {
    case VCU_Vehicle_Status_ID:
    {
        msg = std::make_shared<CGW_Vehicle_Status_Msg>();                   
        memcpy(msg->SerializeCan(), frame.data, msg->GetSizeCan());
        break;
    }
    case VCU_Parking_Status_ID:
    {
        msg = std::make_shared<CGW_Parking_Status_Msg>();
        memcpy(msg->SerializeCan(), frame.data, msg->GetSizeCan());
        break;
    }
    case VCU_Vehicle_Engine_Status_ID:
    {
        msg = std::make_shared<VCU_Vehicle_Engine_Status_Msg>();
        memcpy(msg->SerializeCan(), frame.data, msg->GetSizeCan());
        break;
    }
    case VCU_ParkingLane_Request_ID:
    {
        msg = std::make_shared<VCU_ParkingLane_Request_Msg>();
        memcpy(msg->SerializeCan(), frame.data, msg->GetSizeCan());
        int Lane_Request = std::static_pointer_cast<VCU_ParkingLane_Request_Msg>(msg)->data_.data.Lane_Request;
        if(vision_node)
        {
            if(Lane_Request == 0) // 요청 안함.
            {
                vision_node->lane_request_ = false;   
                vision_node->set_parameter(rclcpp::Parameter("lane_request", false));
            }
            else if(Lane_Request == 1) // 요청 함.
            {
               vision_node->lane_request_= true;
                vision_node->set_parameter(rclcpp::Parameter("lane_request", true));
            }
        }

        break;
    }
    case VCU_Exiting_Status_ID:
    {
        msg = std::make_shared<CGW_Exiting_Status_Msg>();
        memcpy(msg->SerializeCan(), frame.data, msg->GetSizeCan());
        break;
    }   
    case VCU_Camera_ID:
    {
        msg = std::make_shared<VCU_Camera_Msg>();
        memcpy(msg->SerializeCan(),frame.data,msg->GetSizeCan());
    
        int camera_num = std::static_pointer_cast<VCU_Camera_Msg>(msg)->data_.data.camera_num;
        if (vision_node)
        {
            if(camera_num == 0)
            {
                vision_node->mode_= Mode::OFF;
                vision_node->set_parameter(rclcpp::Parameter("mode", "offstate"));
            }
            if(camera_num == 1)
            {
                vision_node->mode_= Mode::DRIVING;
                vision_node->set_parameter(rclcpp::Parameter("mode", "driving"));
            }
            else if(camera_num == 2)
            {
                vision_node->mode_= Mode::PARKING;
                vision_node->set_parameter(rclcpp::Parameter("mode", "parking"));
            }
        }
        return;
    }          
    case SCU_Obstacle_Detection_ID:
    {
        msg = std::make_shared<SCU_Obstacle_Detection_Msg>();
        memcpy(msg->SerializeCan(), frame.data, msg->GetSizeCan());
        break;
    }  
    default:
        return;       
    }
    
    ws_session = CGW->ws_server_->current_session_;
    if (ws_session) {
        // 세션의 strand를 통해 메시지 전송
        auto ws_strand = ws_session->get_strand();  // strand getter 필요
        boost::asio::post(ws_strand, 
            [ws_session, msg]() {
                ws_session->send_message(msg);
            });
    }
}

void MySocketCAN::on_send_can(const boost::system::error_code& error, std::size_t bytes_transferred) {
    if (error) {
        std::cerr << "Thread " << std::this_thread::get_id() 
                 << " - MyCANSocket send error: " << error.message() << std::endl;
    }
}




    
