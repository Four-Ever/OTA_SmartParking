#pragma once

#include "MySocketCAN.h"
#include "MyWebSocketSession.h"
#include "LockBasedQueue.h"
#include "Protocol.h"
#include <boost/asio.hpp>
#include <thread>
#include <vector>

extern std::shared_ptr<CentralGateway> CGW;

class CentralGateway : public std::enable_shared_from_this<CentralGateway> {
public:
    CentralGateway(size_t thread_pool_size = 4);
    ~CentralGateway();

    bool Init();
    void Start();
    void Stop();

public:
    LockBasedQueue<std::shared_ptr<class IMessage>> ccu_queue_;
    std::shared_ptr<MySocketCAN> can_socket_;
    std::shared_ptr<MyWebSocketServer> ws_server_;    

private:
    void run_io_context();

    boost::asio::io_context io_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    
    
    std::vector<std::thread> threads_;
    size_t thread_pool_size_;
    bool is_running_;
};

