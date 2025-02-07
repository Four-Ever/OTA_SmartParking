// src/lane_detection_node.cpp
#include "lane_detection/lane_detection_node.hpp"

LaneDetectionNode::LaneDetectionNode() : Node("lane_detection_node") {
    // 디버그 이미지 퍼블리셔 초기화
    front_debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/lane_detection/front_debug_image", 10);

    rear_debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/lane_detection/rear_debug_image", 10);

    // 카메라 초기화
    if (!initialize_camera()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera!");
        return;
    }

    // 타이머 설정 (30fps)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), // 20fps 설정
        std::bind(&LaneDetectionNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Lane detection node has been initialized");
}

LaneDetectionNode::~LaneDetectionNode() {
    if (front_cam_.isOpened()) {
        front_cam_.release();
    }
    if (rear_cam_.isOpened()) { 
        rear_cam_.release();
    }
}

bool LaneDetectionNode::initialize_camera() {
    // 환경 변수로 실행 모드 확인
   const char* mode = std::getenv("CAMERA_MODE");
   
   if (mode && std::string(mode) == "SSH") {
       // 첫 번째 카메라 - SSH 모드
        std::string pipeline1 = "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)" + std::to_string(FRAME_WIDTH) + ", " +
            "height=(int)" + std::to_string(FRAME_HEIGHT) + ", "
            "format=(string)NV12, framerate=(fraction)30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "appsink";
       
       // 두 번째 카메라 - SSH 모드
        std::string pipeline2 = "nvarguscamerasrc sensor-id=1 ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)" + std::to_string(FRAME_WIDTH) + ", " +
            "height=(int)" + std::to_string(FRAME_HEIGHT) + ", "
            "format=(string)NV12, framerate=(fraction)30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "appsink";
           
        front_cam_.open(pipeline1, cv::CAP_GSTREAMER);
        rear_cam_.open(pipeline2, cv::CAP_GSTREAMER);
    } 
    else 
    {
       // 기본 모드: 직접 접근
        front_cam_.open(FRONT_CAMERA_ID);
        rear_cam_.open(REAR_CAMERA_ID);
    }

   // 두 카메라 모두 열렸는지 확인
    if (!front_cam_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open front camera!");
        return false;
    }
    if (!rear_cam_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open rear camera!");
        front_cam_.release();
        return false;
    }

   // 기본 모드일 때만 설정
    if (!mode || std::string(mode) != "SSH") 
    {
       front_cam_.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
       front_cam_.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
       front_cam_.set(cv::CAP_PROP_FPS, 30);

       rear_cam_.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
       rear_cam_.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
       rear_cam_.set(cv::CAP_PROP_FPS, 30);
    }

    return true;
}

void LaneDetectionNode::timer_callback() {
    cv::Mat front_frame, rear_frame;
    if (!front_cam_.read(front_frame)) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to capture front camera frame!");
        return;
    }
    if (!rear_cam_.read(rear_frame)) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to capture rear camera frame!");
        return;
    }

    cv::Mat front_output_frame = front_frame.clone();
    process_frame(front_frame, front_output_frame);

    cv::Mat rear_output_frame = rear_frame.clone();
    process_frame(rear_frame, rear_output_frame);

    // 디버그 이미지 발행
    sensor_msgs::msg::Image::SharedPtr front_debug_msg = 
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", front_output_frame).toImageMsg();
    front_debug_pub_->publish(*front_debug_msg);

    // 디버그 이미지 발행
    sensor_msgs::msg::Image::SharedPtr rear_debug_msg = 
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rear_output_frame).toImageMsg();
    rear_debug_pub_->publish(*rear_debug_msg);
}

void LaneDetectionNode::process_frame(const cv::Mat& input_frame, cv::Mat& output_frame) {
    // 그레이스케일 변환
    cv::Mat gray;
    cv::cvtColor(input_frame, gray, cv::COLOR_BGR2GRAY);

    // 가우시안 블러
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    // Canny 엣지 검출
    cv::Mat edges;
    cv::Canny(gray, edges, CANNY_LOW, CANNY_HIGH);

    // ROI 마스크 생성
    cv::Mat mask = cv::Mat::zeros(edges.size(), edges.type());
    int roi_height = static_cast<int>(input_frame.rows * ROI_HEIGHT_RATIO);
    std::vector<cv::Point> roi_points = {
        cv::Point(0, input_frame.rows),
        cv::Point(0, roi_height),
        cv::Point(input_frame.cols, roi_height),
        cv::Point(input_frame.cols, input_frame.rows)
    };
    cv::fillConvexPoly(mask, roi_points, cv::Scalar(255));
    
    // ROI 적용
    cv::Mat roi;
    cv::bitwise_and(edges, mask, roi);

    // 차선 검출
    std::vector<cv::Vec4i> lines = detect_lanes(roi);

    // 검출된 차선 그리기
    for (const auto& line : lines) {
        cv::line(output_frame, 
                cv::Point(line[0], line[1]), 
                cv::Point(line[2], line[3]),
                cv::Scalar(0, 0, 255), 2);
    }
}

std::vector<cv::Vec4i> LaneDetectionNode::detect_lanes(const cv::Mat& frame) {
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(frame, lines, 1, CV_PI/180, HOUGH_THRESHOLD, 
                    MIN_LINE_LENGTH, MAX_LINE_GAP);
    
    std::vector<cv::Vec4i> filtered_lines;
    for (const auto& line : lines) {
        double slope = (line[3] - line[1]) / 
                      static_cast<double>(line[2] - line[0]);
        
        // 수평선 제외 (기울기가 너무 작은 선)
        if (std::abs(slope) < 0.1) continue;

        filtered_lines.push_back(line);
    }

    return filtered_lines;
}