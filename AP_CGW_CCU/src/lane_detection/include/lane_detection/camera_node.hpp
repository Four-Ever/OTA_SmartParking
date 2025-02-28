#ifndef LANE_DETECTION_CAMERA_NODE_HPP_
#define LANE_DETECTION_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "DevOption.hpp"

class CameraNode : public rclcpp::Node 
{
public:
    explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~CameraNode();

private:
    void publishImages();  // 카메라 이미지 발행 함수
    void loadCalibrationParams();  // 카메라 캘리브레이션 함수
    // 카메라 설정 변수
    int front_cam_id_;    
    int rear_cam_id_;     
    int img_width_;       
    int img_height_;      
    cv::VideoCapture front_cap_;  
    cv::VideoCapture rear_cap_;   
    
    // ROS2 퍼블리셔 및 타이머
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rear_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

   // 캘리브레이션 관련 변수
   bool use_calibration_{true};
   cv::Mat front_camera_matrix_;
   cv::Mat front_dist_coeffs_;
   cv::Mat rear_camera_matrix_;
   cv::Mat rear_dist_coeffs_;   
};

extern std::shared_ptr<CameraNode> camera_node;

#endif  // LANE_DETECTION_CAMERA_NODE_HPP_