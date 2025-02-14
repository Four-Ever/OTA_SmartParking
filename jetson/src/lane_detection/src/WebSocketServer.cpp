#include "lane_detection/WebSocketServer.h"
#include "lane_detection/Protocol.h"
#include <iostream>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

WebSocketSession::WebSocketSession(tcp::socket&& socket)
    : ws_(std::move(socket)) {
}

void WebSocketSession::start() {
    ws_.async_accept(
        beast::bind_front_handler(
            &WebSocketSession::on_accept,
            shared_from_this()));
}

void WebSocketSession::on_accept(beast::error_code ec) {
    if (ec) {
        std::cerr << "Accept failed: " << ec.message() << std::endl;
        return;
    }
    do_read();
}

void WebSocketSession::on_read(beast::error_code ec, std::size_t bytes_transferred) {
    if (ec) {
        if (ec != websocket::error::closed) {
            std::cerr << "Read failed: " << ec.message() << std::endl;
        }
        return;
    }

    std::string received_msg = beast::buffers_to_string(buffer_.data());
    std::cout << "Received message: " << received_msg << std::endl;
    buffer_.consume(buffer_.size());

    do_read();
}

void WebSocketSession::on_write(beast::error_code ec, std::size_t bytes_transferred) {
    if (ec) {
        std::cerr << "Write failed: " << ec.message() << std::endl;
        return;
    }
}

void WebSocketSession::do_read() {
    ws_.async_read(
        buffer_,
        beast::bind_front_handler(
            &WebSocketSession::on_read,
            shared_from_this()));
}

void WebSocketSession::send_message(const std::shared_ptr<IMessage>& message) {
    ws_.async_write(
        net::buffer(message->SerializeHttp(),message->GetSizeHttp()),
        beast::bind_front_handler(
            &WebSocketSession::on_write,
            shared_from_this()));
}

WebSocketServer::WebSocketServer(net::io_context& ioc, unsigned short port)
    : ioc_(ioc),
      acceptor_(ioc, {net::ip::make_address("0.0.0.0"), port}) {
}

bool WebSocketServer::Init() {
    try {
        do_accept();
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "WebSocket server init error: " << e.what() << std::endl;
        return false;
    }
}

// void WebSocketServer::broadcast_message(const std::string& message) {
//     std::lock_guard<std::mutex> lock(sessions_mutex_);
//     auto it = sessions_.begin();
//     while (it != sessions_.end()) {
//         try {
//             (*it)->send_message(message);
//             ++it;
//         }
//         catch (const std::exception& e) {
//             std::cerr << "Error broadcasting to session: " << e.what() << std::endl;
//             it = sessions_.erase(it);
//         }
//     }
// }

std::shared_ptr<WebSocketSession> WebSocketServer::create_session(tcp::socket socket) {
    return std::make_shared<WebSocketSession>(std::move(socket));
}

void WebSocketServer::do_accept() {
    acceptor_.async_accept(
        net::make_strand(ioc_),
        [this](beast::error_code ec, tcp::socket socket) {
            if (ec) {
                std::cerr << "Accept error: " << ec.message() << std::endl;
            } else {
                std::cout << "New connection from: " 
                         << socket.remote_endpoint().address().to_string() 
                         << ":" << socket.remote_endpoint().port() << std::endl;
                
                auto session = create_session(std::move(socket));
                // {
                //     std::lock_guard<std::mutex> lock(sessions_mutex_);
                //     sessions_.push_back(session);
                //     std::cout << "Total sessions: " << sessions_.size() << std::endl;
                // }
                std::atomic_store(&current_session_, session);
                session->start();
            }
            do_accept();
        });
}