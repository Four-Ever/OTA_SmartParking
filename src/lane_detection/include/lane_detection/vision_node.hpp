#ifndef LANE_DETECTION_VISION_NODE_HPP_
#define LANE_DETECTION_VISION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_service.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

enum class Mode {
   DRIVING,    // 주행 모드
   PARKING     // 주차 모드
};
enum class UsingCamera {
   Front,    // 전면
   Rear,     // 후면
};

class VisionNode : public rclcpp::Node 
{
public:
   explicit VisionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
   // 이미지 처리 콜백 함수
   void processFrontImage(const sensor_msgs::msg::Image::SharedPtr msg);
   void processRearImage(const sensor_msgs::msg::Image::SharedPtr msg);

   // 차선 검출 함수
   std::vector<cv::Point2i> detectDrivingLanes(const cv::Mat& img);
   std::vector<cv::Point2i> detectFrontParkingLanes(const cv::Mat& img);
   std::vector<cv::Point2i> detectRearParkingLanes(const cv::Mat& img);
   cv::Point2i findNearestPoint(const std::vector<cv::Point2i>& points, int target_y);
   // 웨이포인트 관련 함수
   void publishWaypoints(const std::vector<cv::Point2i>& points, UsingCamera is_front);
   void drawDebugImage(cv::Mat& img, const std::vector<cv::Point2i>& points, UsingCamera is_front);


   // 변수들
   Mode mode_{Mode::DRIVING}; 

   cv::Mat front_M_;         
   cv::Mat rear_M_;          

   // ROS2 통신
   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr front_cam_sub_;
   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rear_cam_sub_;
   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_debug_pub_;
   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rear_debug_pub_;
   
   rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr front_waypoint_pub_;
   rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr rear_waypoint_pub_;

   // 디버그용 이미지 퍼블리셔 추가
   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_warped_pub_;   // IPM 결과
   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_edges_pub_;    // Canny 결과
   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_lines_pub_;    // Hough 결과

   // Param Handler
   rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

   // 슬라이딩 윈도우
   std::vector<cv::Point2i> prev_left_points_;
   std::vector<cv::Point2i> prev_right_points_;
   bool first_frame_ = true;
   float prev_confidence_ = 1.0;

   bool isValidLaneWidth(const cv::Point2i& left, const cv::Point2i& right) {
       int width = abs(right.x - left.x);
       return (width > 150 && width < 400);  // 적절한 차선 폭 범위
   }

   void smoothPoints(std::vector<cv::Point2i>& current_points, 
                    const std::vector<cv::Point2i>& prev_points);
};
#endif  // LANE_DETECTION_VISION_NODE_HPP_