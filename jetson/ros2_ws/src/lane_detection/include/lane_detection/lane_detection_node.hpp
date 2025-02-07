#ifndef LANE_DETECTION_NODE_HPP_
#define LANE_DETECTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class LaneDetectionNode : public rclcpp::Node {
public:
    LaneDetectionNode();
    ~LaneDetectionNode();

private:
    void timer_callback();
    void process_frame(const cv::Mat& input_frame, cv::Mat& output_frame);
    std::vector<cv::Vec4i> detect_lanes(const cv::Mat& frame);
    bool initialize_camera();

    // ROS2 퍼블리셔
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_debug_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rear_debug_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // OpenCV 관련
    cv::VideoCapture front_cam_;
    cv::VideoCapture rear_cam_;

    // 카메라 설정 값
    static constexpr int FRONT_CAMERA_ID = 0;
    static constexpr int REAR_CAMERA_ID = 1;
    const int FRAME_WIDTH = 640; // 프레임 너비
    const int FRAME_HEIGHT = 480; // 프레임 높이
    // 1280 x 720 가능
    const double ROI_HEIGHT_RATIO = 0.6;  // ROI 영역 높이 비율

    // 차선 검출 파라미터
    const int CANNY_LOW = 50;
    const int CANNY_HIGH = 150;
    const int HOUGH_THRESHOLD = 30;
    const int MIN_LINE_LENGTH = 30;
    const int MAX_LINE_GAP = 60;
};

#endif // LANE_DETECTION_NODE_HPP_