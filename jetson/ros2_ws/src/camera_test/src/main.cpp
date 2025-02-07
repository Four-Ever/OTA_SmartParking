#include "camera_test/camera_test_node.hpp"

// src/main.cpp
int main(int argc, char* argv[]) {
    // ROS2 초기화
    rclcpp::init(argc, argv);
    
    // 노드 생성 및 실행
    auto node = std::make_shared<CameraTestNode>();
    
    // 스핀
    rclcpp::spin(node);
    
    // 종료
    rclcpp::shutdown();
    return 0;
}