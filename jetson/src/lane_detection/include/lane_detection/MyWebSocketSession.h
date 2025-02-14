#pragma once

#include "WebSocketServer.h"



class MyWebSocketSession : public WebSocketSession {
public:
    MyWebSocketSession(boost::asio::ip::tcp::socket&& socket, std::shared_ptr<class CentralGateway> owner);
protected:
    void on_read(boost::beast::error_code ec, std::size_t bytes_transferred) override;

private:
    //void process_message(const std::string& message);
public:
    std::weak_ptr<class CentralGateway> owner_;
};


class MyWebSocketServer : public WebSocketServer, public std::enable_shared_from_this<WebSocketServer> {
public:
    MyWebSocketServer(boost::asio::io_context& ioc, unsigned short port, std::shared_ptr<class CentralGateway> owner);
    
protected:
    std::shared_ptr<WebSocketSession> create_session(boost::asio::ip::tcp::socket socket) override;
public:
    std::weak_ptr<class CentralGateway> owner_;
    
};


