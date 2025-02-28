#include "lane_detection/CentralGateway.h"
#include <iostream>
#include <sstream>

std::shared_ptr<CentralGateway> CGW = std::make_shared<CentralGateway>(4);

CentralGateway::CentralGateway(size_t thread_pool_size)
    : work_guard_(io_context_.get_executor())
    , thread_pool_size_(thread_pool_size)
    , is_running_(false) {
       
}

CentralGateway::~CentralGateway() {
    Stop();
}

bool CentralGateway::Init() {
    can_socket_ = std::make_shared<MySocketCAN>(io_context_,shared_from_this());
    ws_server_ = std::make_shared<MyWebSocketServer>(io_context_, 8080, shared_from_this());

    // CAN 소켓 초기화
    if (!can_socket_->Init("can0")) {
        std::cerr << "CAN socket 초기화 실패패" << std::endl;
        return false;
    }

    // WebSocket 서버 초기화
    if (!ws_server_->Init()) {
        std::cerr << "WebSocket server 초기화 실패패" << std::endl;
        return false;
    }

    // 스레드 풀 시작
    threads_.reserve(thread_pool_size_);
    for (size_t i = 0; i < thread_pool_size_; ++i) {
        threads_.emplace_back([this]() { run_io_context(); });
    }

    std::cout << "게이트웨이:" << std::endl;
    std::cout << "- 웹소켓 포트 8080" << std::endl;
    std::cout << "- 소켓CAN can0" << std::endl;
    std::cout << "- 워커 스레드 개수수: " << thread_pool_size_ << std::endl;
    return true;
}

void CentralGateway::Start() {
    is_running_ = true;
    
    // CAN 수신 시작
    can_socket_->start_receive();

}

void CentralGateway::Stop() {
    if (!is_running_) return;
    
    is_running_ = false;
    work_guard_.reset();

    for (auto& thread : threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    threads_.clear();

    io_context_.stop();
    std::cout << "게이트웨이 종료료" << std::endl;
}


void CentralGateway::run_io_context() {
    try {
        std::cout << "Thread " << std::this_thread::get_id() << " started" << std::endl;
        io_context_.run();
        std::cout << "Thread " << std::this_thread::get_id() << " stopped" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Thread exception: " << e.what() << std::endl;
    }
}