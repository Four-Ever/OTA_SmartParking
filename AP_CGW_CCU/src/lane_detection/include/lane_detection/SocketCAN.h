#pragma once

#include <boost/asio.hpp>
#include <linux/can.h>

class SocketCAN {
public:
    SocketCAN(boost::asio::io_context& io_context);
    virtual ~SocketCAN() = default;

    virtual bool Init(const std::string& interface_name);
    void async_send(std::shared_ptr<class IMessage>& message);
    void start_receive();
    bool send(std::shared_ptr<class IMessage> message);

protected:
    virtual void on_receive_can(const can_frame& frame) = 0;
    virtual void on_send_can(const boost::system::error_code& error, std::size_t bytes_transferred) = 0;

private:
    void async_receive();

    boost::asio::generic::raw_protocol::socket socket_;
    boost::asio::io_context& io_context_;
};

