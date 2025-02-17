#pragma once

#include "SocketCAN.h"

class MySocketCAN : public SocketCAN {
public:
    MySocketCAN(boost::asio::io_context& io_context, std::shared_ptr<class CentralGateway> owner);

public:
    bool Init(const std::string& interface_name) override;
    void on_receive_can(const can_frame& frame) override;
    void on_send_can(const boost::system::error_code& error, std::size_t bytes_transferred) override;

private:
    std::weak_ptr<class CentralGateway> owner_;
};