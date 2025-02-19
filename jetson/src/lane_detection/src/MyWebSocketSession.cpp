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
        break;
    }
    default:
    {
        buffer_.consume(buffer_.size());
        do_read();
        return;
    }
    }

    CGW->can_socket_->async_send(msg);

    buffer_.consume(buffer_.size());
    do_read();
}

MyWebSocketServer::MyWebSocketServer(boost::asio::io_context& ioc, unsigned short port, std::shared_ptr<CentralGateway> owner)
    : WebSocketServer(ioc, port), owner_(owner) {
}

std::shared_ptr<WebSocketSession> MyWebSocketServer::create_session(boost::asio::ip::tcp::socket socket) {
    return std::make_shared<MyWebSocketSession>(std::move(socket), owner_.lock());
}