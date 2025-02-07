// include/camera_test/camera_test_node.hpp
#ifndef CAMERA_TEST_NODE_HPP_
#define CAMERA_TEST_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraTestNode : public rclcpp::Node {
public:
    // 생성자
    CameraTestNode();
    // 소멸자
    ~CameraTestNode();

private:
    // 타이머 콜백 함수
    void timer_callback();
    
    // 카메라 초기화 함수
    bool initialize_cameras();

    // ROS2 퍼블리셔
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rear_pub_;
    
    // 타이머
    rclcpp::TimerBase::SharedPtr timer_;

    // OpenCV 비디오 캡처 객체
    cv::VideoCapture front_cam_;
    cv::VideoCapture rear_cam_;

    // 카메라 설정값
    const int FRONT_CAM_ID = 0;  // 전방 카메라 ID
    const int REAR_CAM_ID = 1;   // 후방 카메라 ID
    const int FRAME_WIDTH = 640;  // 프레임 너비
    const int FRAME_HEIGHT = 480; // 프레임 높이
};

#endif // CAMERA_TEST_NODE_HPP_
