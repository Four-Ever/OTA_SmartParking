#pragma once
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <mutex>
#include <queue>

// WebSocket 세션 기본 클래스
class WebSocketSession : public std::enable_shared_from_this<WebSocketSession> {
public:
    WebSocketSession(boost::asio::ip::tcp::socket&& socket);
    virtual ~WebSocketSession() = default;

    virtual void start();
    void send_message(const std::shared_ptr<class IMessage> message);
    bool send(const std::shared_ptr<class IMessage> message);
    boost::asio::strand<boost::asio::executor>& get_strand() { 
        return strand_; 
    }

protected:
    virtual void on_accept(boost::beast::error_code ec);
    virtual void on_read(boost::beast::error_code ec, std::size_t bytes_transferred);
    virtual void on_write(boost::beast::error_code ec, std::size_t bytes_transferred);
    void do_read();
    void do_write();

protected:
    boost::beast::websocket::stream<boost::asio::ip::tcp::socket> ws_;
    boost::beast::flat_buffer buffer_;

private:
    boost::asio::strand<boost::asio::executor> strand_; 
    std::queue<std::shared_ptr<IMessage>> write_queue_;
    std::mutex queue_mutex_;
    std::atomic<bool> is_writing_{false};
};

class WebSocketServer {
public:
    WebSocketServer(boost::asio::io_context& ioc, unsigned short port);
    virtual ~WebSocketServer() = default;

    bool Init();

protected:
    virtual std::shared_ptr<WebSocketSession> create_session(boost::asio::ip::tcp::socket socket);

private:
    void do_accept();

public:
    boost::asio::io_context& ioc_;
    boost::asio::ip::tcp::acceptor acceptor_;
    std::shared_ptr<WebSocketSession> current_session_;
};