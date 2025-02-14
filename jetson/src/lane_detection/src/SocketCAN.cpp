#include "lane_detection/SocketCAN.h"
#include "lane_detection/Protocol.h"
#include <iostream>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

using namespace boost::asio;

SocketCAN::SocketCAN(io_context& io_context)
    : socket_(io_context), io_context_(io_context) {
}

bool SocketCAN::Init(const std::string& interface_name) {
    try {
        // CAN 소켓 생성
        int can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket < 0) {
            std::cerr << "Failed to create CAN socket" << std::endl;
            return false;
        }

        // 인터페이스 인덱스 가져오기
        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, interface_name.c_str());
        if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
            std::cerr << "Failed to get interface index for " << interface_name << std::endl;
            close(can_socket);
            return false;
        }

        // 소켓 바인딩
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "Failed to bind CAN socket to " << interface_name << std::endl;
            close(can_socket);
            return false;
        }

        // boost::asio::socket에 네이티브 소켓 할당
        socket_.assign(boost::asio::generic::raw_protocol(AF_CAN, CAN_RAW), can_socket);
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Exception in Init: " << e.what() << std::endl;
        return false;
    }
}

void SocketCAN::async_send(std::shared_ptr<IMessage>& message) {
    struct can_frame frame = { };
    frame.can_id = message->GetMsgId();
    frame.can_dlc = message->GetSizeCan();
    memcpy(frame.data, message->SerializeCan(),message->GetSizeCan());
    post(socket_.get_executor(), [this, frame]() {
        socket_.async_send(
            boost::asio::buffer(&frame, sizeof(frame)),
            [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
                on_send_can(error, bytes_transferred);
            });
    });
}

void SocketCAN::start_receive() {
    async_receive();
}

void SocketCAN::async_receive() {
    auto frame = std::make_shared<can_frame>();
    post(socket_.get_executor(), [this, frame]() {
        socket_.async_receive(
            boost::asio::buffer(frame.get(), sizeof(can_frame)),
            [this, frame](const boost::system::error_code& error, std::size_t bytes_transferred) {
                if (!error && bytes_transferred == sizeof(can_frame)) {
                    on_receive_can(*frame);
                    async_receive();
                }
            });
    });
}