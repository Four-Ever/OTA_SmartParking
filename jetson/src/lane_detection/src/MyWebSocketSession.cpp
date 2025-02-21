#include "lane_detection/MyWebSocketSession.h"
#include "lane_detection/Protocol.h"
#include "lane_detection/CentralGateway.h"
#include "lane_detection/MySocketCAN.h"
#include "lane_detection/OTADownloader.h"
#include <iostream>
#include <thread>
#include <chrono>

MyWebSocketSession::MyWebSocketSession(boost::asio::ip::tcp::socket&& socket, std::shared_ptr<CentralGateway> owner)
    : WebSocketSession(std::move(socket)), owner_(owner) {
}

void MyWebSocketSession::on_read(boost::beast::error_code ec, std::size_t bytes_transferred) {
    if (ec) {
        if (ec != boost::beast::websocket::error::closed) {
            std::cerr << "Read failed: " << ec.message() << std::endl;
        }
        std::cout << "WebSocket connection closed" << std::endl;
        return;
    }

    int id = *reinterpret_cast<uint8_t*>(buffer_.data().data());
    std::shared_ptr<IMessage> msg;

    switch (id) {
    case CTRL_Engine_ID:
    {
        msg = std::make_shared<CGW_Engine_Msg>();
        memcpy(msg->SerializeCan(), (uint8_t*)(buffer_.data().data())+1, msg->GetSizeCan());
        CGW->can_socket_->async_send(msg);

        std::shared_ptr<CGW_Engine_Msg> cmsg = std::static_pointer_cast<CGW_Engine_Msg>(msg);

        if (cmsg->data_.data.control_engine == 0) {
            std::shared_ptr<IMessage> wmsg = std::make_shared<CGW_OTA_Update_Request_Msg>();
            auto request = std::static_pointer_cast<CGW_OTA_Update_Request_Msg>(wmsg);
            request->SetOtaUpdateRequest(1);
            //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            std::cout<<"OTA 요청 전송"<<std::endl;
            auto ws_session = CGW->ws_server_->current_session_;
            if (ws_session) {
                // 세션의 strand를 통해 메시지 전송
                auto ws_strand = ws_session->get_strand();  // strand getter 필요
                boost::asio::post(ws_strand, 
                    [ws_session, wmsg]() {
                        ws_session->send_message(wmsg);
                    });
            }

            //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            if (ws_session) {
                // 세션의 strand를 통해 메시지 전송
                auto ws_strand = ws_session->get_strand();  // strand getter 필요
                boost::asio::post(ws_strand, 
                    [ws_session, wmsg]() {
                        ws_session->send_message(wmsg);
                    });
            }
        }


        break;
    }
    case CTRL_Move_ID:
    {
        msg = std::make_shared<CGW_Move_Msg>();
        memcpy(msg->SerializeCan(), (uint8_t*)(buffer_.data().data())+1, msg->GetSizeCan());
        break;
    }
    case CTRL_Auto_Parking_Request_ID:
    {
        msg = std::make_shared<CGW_Auto_Parking_Request_Msg>();
        memcpy(msg->SerializeCan(), (uint8_t*)(buffer_.data().data())+1, msg->GetSizeCan());
        break;
    }
    case CTRL_Off_Request_ID:
    {
        msg = std::make_shared<CGW_Off_Request_Msg>();
        memcpy(msg->SerializeCan(), (uint8_t*)(buffer_.data().data())+1, msg->GetSizeCan());
        //CGW->can_socket_->async_send(msg);

        // //CGW_OTA_Update_Request

        // std::shared_ptr<IMessage> wmsg = std::make_shared<CGW_OTA_Update_Request_Msg>();
        // auto request = std::static_pointer_cast<CGW_OTA_Update_Request_Msg>(wmsg);
        // request->SetOtaUpdateRequest(1);
        // std::cout<<"OTA 요청 전송"<<std::endl;
        // auto ws_session = CGW->ws_server_->current_session_;
        // if (ws_session) {
        //     // 세션의 strand를 통해 메시지 전송
        //     auto ws_strand = ws_session->get_strand();  // strand getter 필요
        //     boost::asio::post(ws_strand, 
        //         [ws_session, wmsg]() {
        //             ws_session->send_message(wmsg);
        //         });
        // }

        break;
    }
    case CTRL_OTA_Update_Confirm_ID:
    {
        std::cout<<"confirm"<<std::endl;
        msg = std::make_shared<CTRL_OTA_Update_Confirm_Msg>();
        memcpy(msg->SerializeHttp(), (uint8_t*)(buffer_.data().data()), msg->GetSizeHttp());
        std::shared_ptr<CTRL_OTA_Update_Confirm_Msg> wmsg = std::static_pointer_cast<CTRL_OTA_Update_Confirm_Msg>(msg);
        bool ans = wmsg->data_.data.ota_confirm;
        std::cout<<"ans"<<ans<<std::endl;
        if (!ans) 
            break;
        //OTA 전송시작
        std::cout<<"펌웨어 전송 시작작"<<std::endl;
        if (!SendFirmware())
            std::cout<<"펌웨어 전송 실패"<<std::endl;

        break;
    }
    default:
    {
        buffer_.consume(buffer_.size());
        do_read();
        return;
    }
    }
    if (id != CTRL_OTA_Update_Confirm_ID && id != CTRL_Engine_ID) {
        CGW->can_socket_->async_send(msg);
    }
    buffer_.consume(buffer_.size());
    do_read();
}

MyWebSocketServer::MyWebSocketServer(boost::asio::io_context& ioc, unsigned short port, std::shared_ptr<CentralGateway> owner)
    : WebSocketServer(ioc, port), owner_(owner) {
}

std::shared_ptr<WebSocketSession> MyWebSocketServer::create_session(boost::asio::ip::tcp::socket socket) {
    return std::make_shared<MyWebSocketSession>(std::move(socket), owner_.lock());
}