#include "lane_detection/MyWebSocketSession.h"
#include "lane_detection/Protocol.h"
#include "lane_detection/CentralGateway.h"
#include "lane_detection/MySocketCAN.h"
#include <iostream>
#include <thread>

MyWebSocketSession::MyWebSocketSession(boost::asio::ip::tcp::socket&& socket, std::shared_ptr<CentralGateway> owner)
    : WebSocketSession(std::move(socket)), owner_(owner) {
}

void MyWebSocketSession::on_read(boost::beast::error_code ec, std::size_t bytes_transferred) {
    if (ec) {
        if (ec != boost::beast::websocket::error::closed) {
            std::cerr << "Read failed: " << ec.message() << std::endl;
        }
        std::cout<<"closed"<<std::endl;
        return;
    }
    //std::cout<<"received"<<std::endl;
    // 수신된 메시지 처리
    //std::string msg = boost::beast::buffers_to_string(buffer_.data());
    //process_message(msg);
    int id = *reinterpret_cast<uint8_t*>(buffer_.data().data());
    std::shared_ptr<IMessage> msg;

    switch (id)
    {
    // case VCU_Vehicle_Status_ID:
    // {
    //     msg = std::make_shared<VCU_Vehicle_Status_Msg>();
    //     memcpy(msg->SerializeHttp(),buffer_.data().data(),msg->GetSizeHttp());
    //     break;
    // }
    // case VCU_Parking_Status_ID:
    // {
    //     msg = std::make_shared<VCU_Parking_Status_Msg>();
    //     memcpy(msg->SerializeHttp(),buffer_.data().data(),msg->GetSizeHttp());
    //     break;
    // }
    // case VCU_Vehicle_Engine_Status_ID:
    // {
    //     msg = std::make_shared<VCU_Vehicle_Engine_Status_Msg>();
    //     memcpy(msg->SerializeHttp(),buffer_.data().data(),msg->GetSizeHttp());
    //     break;
    // }
    // case VCU_Camera_ID:
    // {
    //     msg = std::make_shared<VCU_Camera_Msg>();
    //     memcpy(msg->SerializeHttp(),buffer_.data().data(),msg->GetSizeHttp());
    //     break;
    // }
    // case SCU_Obstacle_Detection_ID:
    // {
    //     msg = std::make_shared<SCU_Obstacle_Detection_Msg>();
    //     memcpy(msg->SerializeHttp(),buffer_.data().data(),msg->GetSizeHttp());
    //     break;
    // }
    // case CGW_OTA_File_Size_ID:
    // {
    //     msg = std::make_shared<CGW_OTA_File_Size_Msg>();
    //     memcpy(msg->SerializeHttp(),buffer_.data().data(),msg->GetSizeHttp());
    //     break;
    // }
    // case CGW_OTA_File_Data_ID:
    // {
    //     msg = std::make_shared<CGW_OTA_File_Data_Msg>();
    //     memcpy(msg->SerializeHttp(),buffer_.data().data(),msg->GetSizeHttp());
    //     break;
    // }
    // case CGW_OTA_Control_ID:
    // {
    //     msg = std::make_shared<CGW_OTA_Control_Msg>();
    //     memcpy(msg->SerializeHttp(),buffer_.data().data(),msg->GetSizeHttp());
    //     break;
    // }
    // case CGW_OTA_Update_Request_ID:
    // {
    //     msg = std::make_shared<CGW_OTA_Update_Request_Msg>();
    //     memcpy(msg->SerializeHttp(),buffer_.data().data(),msg->GetSizeHttp());
    //     break;
    // }
    // case CGW_OTA_Update_State_ID:
    // {
    //     msg = std::make_shared<CGW_OTA_Update_State_Msg>();
    //     memcpy(msg->SerializeHttp(),buffer_.data().data(),msg->GetSizeHttp());
    //     break;
    // }
    // case CGW_Vehicle_Status_ID:
    // {
    //     msg = std::make_shared<CGW_Vehicle_Status_Msg>();
    //     memcpy(msg->SerializeHttp(),buffer_.data().data(),msg->GetSizeHttp());
    //     break;
    // }
    // case CCU_Cordi_data1_ID:
    // {
    //     msg = std::make_shared<CCU_Cordi_data1_Msg>();
    //     memcpy(msg->SerializeHttp(),buffer_.data().data(),msg->GetSizeHttp());
    //     break;
    // }
    // case CCU_Cordi_data2_ID:
    // {
    //     msg = std::make_shared<CCU_Cordi_data2_Msg>();
    //     memcpy(msg->SerializeHttp(),buffer_.data().data(),msg->GetSizeHttp());
    //     break;
    // }
    case CTRL_Engine_ID:
    {
        msg = std::make_shared<CGW_Engine_Msg>();
        memcpy(msg->SerializeCan(),(uint8_t*)(buffer_.data().data())+1,msg->GetSizeCan());
        break;
    }
    case CTRL_Move_ID:
    {
        msg = std::make_shared<CGW_Move_Msg>();
        memcpy(msg->SerializeCan(),(uint8_t*)(buffer_.data().data())+1,msg->GetSizeCan());
        auto tempMsg = std::static_pointer_cast<CTRL_Move_Msg>(msg);
        //std::cout<<"size "<<tempMsg->GetSizeCan()<<std::endl;
        // std::cout<<"c_a "<<static_cast<int>(tempMsg->data_.data.control_accel)<<std::endl;
        // std::cout<<"c_b "<<static_cast<int>(tempMsg->data_.data.control_brake)<<std::endl;
        // std::cout<<"c_s_a "<<static_cast<int>(tempMsg->data_.data.control_steering_angle)<<std::endl;
        // std::cout<<"c_t "<<static_cast<int>(tempMsg->data_.data.control_transmission)<<std::endl;
        
        break;
    }
    case CTRL_Auto_Parking_Request_ID:
    {
        msg = std::make_shared<CGW_Auto_Parking_Request_Msg>();
        memcpy(msg->SerializeCan(),(uint8_t*)(buffer_.data().data())+1,msg->GetSizeCan());
        break;
    }
    // case CTRL_OTA_Update_Confirm_ID:
    // {
    //     msg = std::make_shared<CTRL_OTA_Update_Confirm_Msg>();
    //     memcpy(msg->SerializeHttp(),buffer_.data().data(),msg->GetSizeHttp());
    //     break;
    // }
    case CTRL_Off_Request_ID:
    {
        msg = std::make_shared<CGW_Off_Request_Msg>();
        memcpy(msg->SerializeCan(),(uint8_t*)(buffer_.data().data())+1,msg->GetSizeCan());
        break;
    }
    default:
    {
        buffer_.consume(buffer_.size());
        do_read();
        return;
    }
    }
    // if (msg->GetSizeHttp() != buffer.data().size())
    //     continue;   
    //can_queue_.Push(msg);
   // {
        // auto owner = owner_.lock();
        // if (owner) {
        //         owner->can_socket_->async_send(msg);           
        // }
   // }
    CGW->can_socket_->async_send(msg);

    buffer_.consume(buffer_.size());
    do_read();
}

// void MyWebSocketSession::process_message(const std::string& message) {
//     // std::cout << "Thread " << std::this_thread::get_id() 
//     //           << " - MyWebSocketSession received: " << message << std::endl;
    
//     // 메시지 에코
//     //std::string response = "Server received: " + message;


//     send_message(response);
// }

MyWebSocketServer::MyWebSocketServer(boost::asio::io_context& ioc, unsigned short port,std::shared_ptr<CentralGateway> owner)
    : WebSocketServer(ioc, port), owner_(owner) {       
}

std::shared_ptr<WebSocketSession> MyWebSocketServer::create_session(boost::asio::ip::tcp::socket socket) {
    
    return std::make_shared<MyWebSocketSession>(std::move(socket), owner_.lock());
}