#include "lane_detection/WebSocketServer.h"
#include "lane_detection/Protocol.h"
#include <iostream>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

WebSocketSession::WebSocketSession(tcp::socket&& socket)
    : ws_(std::move(socket)), strand_(ws_.get_executor()) {
}

void WebSocketSession::start() {
    auto self = shared_from_this();
    boost::asio::post(strand_, [self]() {
        self->ws_.async_accept(
            boost::beast::bind_front_handler(
                &WebSocketSession::on_accept,
                self));
    });
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

void WebSocketSession::on_write(boost::beast::error_code ec, std::size_t bytes_transferred) {
    std::cout << "Write completed. Result: " << (ec ? ec.message() : "Success") << std::endl;
    if (ec) {
        std::cerr << "Write failed: " << ec.message() << std::endl;
        return;
    }
}

void WebSocketSession::do_read() {
    auto self = shared_from_this();
    boost::asio::post(strand_, [self]() {
        self->ws_.async_read(
            self->buffer_,
            boost::beast::bind_front_handler(
                &WebSocketSession::on_read,
                self));
    });
}

void WebSocketSession::do_write() {
    if (is_writing_) return;

    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (write_queue_.empty()) return;

    is_writing_ = true;
    auto msg = write_queue_.front();
    write_queue_.pop();

    auto self = shared_from_this();
    ws_.async_write(
        boost::asio::buffer(msg->SerializeHttp(), msg->GetSizeHttp()),
        [this, self](boost::system::error_code ec, std::size_t) {
            is_writing_ = false;
            if (!ec) {
                do_write();  // 다음 메시지 처리
            }
        });
}

void WebSocketSession::send_message(const std::shared_ptr<IMessage> message) {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        write_queue_.push(message);
    }
    do_write();
}

bool WebSocketSession::send(const std::shared_ptr<class IMessage> message) {
    try {
        //std::string msg_str = message->Serialize();
        ws_.write(boost::asio::buffer(message->SerializeHttp(), message->GetSizeHttp()));
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "WebSocket send error: " << e.what() << std::endl;
        return false;
    }
}

WebSocketServer::WebSocketServer(net::io_context& ioc, unsigned short port)
    : ioc_(ioc),
      acceptor_(ioc, {net::ip::make_address("0.0.0.0"), port}),
      current_session_(nullptr) {
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
                
                current_session_ = create_session(std::move(socket));
                current_session_->start();
            }
            do_accept();
        });
}