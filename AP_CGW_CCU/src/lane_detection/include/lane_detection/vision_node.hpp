#ifndef LANE_DETECTION_VISION_NODE_HPP_
#define LANE_DETECTION_VISION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_service.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "DevOption.hpp"

#ifdef WITH_CUDA
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif
enum class Mode {
   OFF,
   DRIVING,    // 주행 모드
   PARKING     // 주차 모드
};
enum class UsingCamera {
   OFF,
   Front,    // 전면
   Rear,     // 후면
};
enum class DebugDrawOption {
   NONE = 0x00,                  // 디버그 그리기 없음
   ROI_BOX = 0x01,               // ROI 영역 표시
   SLIDING_WINDOWS_BOX = 0x02,   // 윈도우 영역 표시
   MID_POINT_OF_LANE = 0x04,           // 도로 중심 포인트 표시
   REGRESSION_CURVE = 0x08,      // 회귀 곡선 그리기
   WAY_POINT = 0x10,             // 곡선에서의 way point표시
   ALL = 0xFF                    // 모든 디버그 정보 표시
};
struct RegressionResult {
   std::vector<cv::Point2i> waypoints;
   cv::Mat coefficients;
};
struct ResultVisionProcess{
   std::vector<cv::Point2i> waypoints;
   float window_confidence;
};

class VisionNode : public rclcpp::Node 
{
public:
   explicit VisionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
      // 파라미터 세팅 관련
   void setParameter(const std::string &name, int value)
   {
      if (rclcpp::ok())
      {
         this->set_parameter(rclcpp::Parameter(name, value));
      }
   }

public:
   // 이미지 처리 콜백 함수
   void processFrontImage(const sensor_msgs::msg::Image::SharedPtr msg);
   void processRearImage(const sensor_msgs::msg::Image::SharedPtr msg);

   // 차선 검출 함수
   ResultVisionProcess detectDrivingLanes(const cv::Mat& img);
   std::vector<cv::Point2i> detectFrontParkingLanes(const cv::Mat& img);
   ResultVisionProcess detectRearParkingLanes(const cv::Mat& img);
   cv::Point2i findNearestPoint(const std::vector<cv::Point2i>& points, int target_y); // 근처의 점 찾기
   // 웨이포인트 관련 함수
   void smoothPoints(std::vector<cv::Point2i> &current_points,
      const std::vector<cv::Point2i> &prev_points,
      const std::vector<float> &confidences);
   bool isValidLaneWidth(const cv::Point2i& left, const cv::Point2i& right, int lane_width); // 라인 검출
   RegressionResult regressionWaypoints(const std::vector<cv::Point2i>& waypoints, 
      const std::vector<float>& confidences,
      int num_points);  // 원하는 웨이포인트 개수

   cv::Scalar getColorByConfidence(float confidence); // 신뢰도에 따른 색상 검출
   void publishWaypoints(const std::vector<cv::Point2i>& points, UsingCamera is_front); // 웨이포인트 발행
   void drawDebugImage(cv::Mat& img, const std::vector<cv::Point2i>& points, UsingCamera is_front);
   cv::Point2i imageToVehicle(const cv::Point2i& img_point); // 이미지 좌표 -> 차량 좌표계 변환
   cv::Point2i vehicleToImage(const cv::Point2i& vehicle_point); // 차량 좌표계 -> 이미지 좌표 변환
   cv::Point2f pixelToNormalized(const cv::Point2f& pixel, UsingCamera is_front); // 포인트 정규화
   // 변수들
   Mode mode_{Mode::DRIVING}; 
   bool lane_request_ = false;
   int expected_lane_width_;
   int expected_parking_lane_width_;
   DebugDrawOption debug_draw_option_{DebugDrawOption::ALL};

   // IPM 변환 행렬
   cv::Mat front_M_;         
   cv::Mat rear_M_;          

   // ROS2 통신
   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr front_cam_sub_;
   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rear_cam_sub_;

   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_debug_pub_;
   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rear_debug_pub_;
   
   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_debug_on_real_pub_;  
   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rear_debug_on_real_pub_;
   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr DEBUG_pub_;

   // 변수형 퍼블리셔
   rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr front_waypoint_pub_;
   rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr rear_waypoint_pub_;
   rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr width_plot_pub_; // width 변화
   rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr width_plot_pub_rear_; // width 변화

   // Param Handler
   rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

   // 슬라이딩 윈도우
   // 전방
   std::vector<cv::Point2i> prev_left_points_front_;
   std::vector<cv::Point2i> prev_right_points_front_;
   std::vector<float> prev_confidence_front_;
   bool first_frame_front_ = true;
   // 후방
   std::vector<cv::Point2i> prev_left_points_rear_;
   std::vector<cv::Point2i> prev_right_points_rear_;
   std::vector<float> prev_confidence_rear_;
   bool first_frame_rear_ = true;

   // 주차선 검출 Flag
   bool right_angle_detected_flag = false;


   // 카메라 intrinsic
   // const cv::Mat front_camera_matrix_ = (cv::Mat_<double>(3,3) <<
   // 638.433158, 0, 332.586432,
   // 0, 639.580011, 248.493045,
   // 0, 0, 1);
   const cv::Mat wide_front_camera_matrix_ = (cv::Mat_<double>(3,3) <<
   505.844573, 0, 300.088078,
   0, 520.071999, 272.405645,
   0, 0, 1);
   // const cv::Mat rear_camera_matrix_ = (cv::Mat_<double>(3,3) <<
   // 577.342746, 0, 318.850211, 
   // 0, 581.499986, 232.537614, 
   // 0, 0, 1);
   const cv::Mat wide_rear_camera_matrix_ = (cv::Mat_<double>(3,3) <<
   416.325380, 0, 316.931237,
   0, 421.930054, 246.635248,
   0, 0, 1);
};

extern std::shared_ptr<VisionNode> vision_node;

#endif  // LANE_DETECTION_VISION_NODE_HPP_
