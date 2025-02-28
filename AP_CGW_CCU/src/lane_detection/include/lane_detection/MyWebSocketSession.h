#pragma once

#include "WebSocketServer.h"
#include <iostream>

class MyWebSocketSession : public WebSocketSession {
public:
    MyWebSocketSession(boost::asio::ip::tcp::socket&& socket, std::shared_ptr<class CentralGateway> owner);
    ~MyWebSocketSession() { 
        std::cout << "~MyWebSocketSession" << std::endl;
    }

protected:
    void on_read(boost::beast::error_code ec, std::size_t bytes_transferred) override;

private:
    std::weak_ptr<class CentralGateway> owner_;
};

class MyWebSocketServer : public WebSocketServer {
public:
    MyWebSocketServer(boost::asio::io_context& ioc, unsigned short port, std::shared_ptr<class CentralGateway> owner);
    
protected:
    std::shared_ptr<WebSocketSession> create_session(boost::asio::ip::tcp::socket socket) override;

private:
    std::weak_ptr<class CentralGateway> owner_;
};