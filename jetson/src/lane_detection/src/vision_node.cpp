// src/vision_node.cpp
#include "lane_detection/vision_node.hpp"
#include "lane_detection/CentralGateway.h"
#include "lane_detection/Protocol.h"
std::shared_ptr<VisionNode> vision_node = nullptr;

VisionNode::VisionNode(const rclcpp::NodeOptions &options)
    : Node("vision_node", options)
{
    // 파라미터 선언
    // this->declare_parameter("mode", "driving");
    this->declare_parameter("mode", "offstate");                 // 실제 동작때.
    this->declare_parameter("expected_lane_width", 500);         // Edit Param
    this->declare_parameter("expected_parking_lane_width", 500); // Edit Param
    this->declare_parameter("lane_request", false);

    // Subscriber 생성
    front_cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "front_camera/real_image", 10,
        std::bind(&VisionNode::processFrontImage, this, std::placeholders::_1));

    // Subscriber 생성 시
    rear_cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "rear_camera/real_image", 10,
        std::bind(&VisionNode::processRearImage, this, std::placeholders::_1));

    // Publisher 생성
    front_debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("front_camera/debug_image", 10);
    rear_debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("rear_camera/debug_image", 10);

    // 역변환
    front_debug_on_real_pub_ = this->create_publisher<sensor_msgs::msg::Image>("front_camera/debug_on_real_image", 10);
    rear_debug_on_real_pub_ = this->create_publisher<sensor_msgs::msg::Image>("rear_camera/debug_on_real_image", 10);

    front_waypoint_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("front_camera/waypoints", 10);
    rear_waypoint_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("rear_camera/waypoints", 10);

    // 변수 모니터링
    width_plot_pub_ = this->create_publisher<std_msgs::msg::Int32>("param/lane_width_plot", 10);
    width_plot_pub_rear_ = this->create_publisher<std_msgs::msg::Int32>("param/lane_width_plot_rear", 10);

    // IPM 변환 행렬 파라미터 초기화
    std::vector<cv::Point2f> front_src_points = {
        cv::Point2f(0, 390), cv::Point2f(640, 390),
        // cv::Point2f(110, 200), cv::Point2f(530, 200)
        // rear : cv::Point2f(200, 0), cv::Point2f(440, 0)};
        cv::Point2f(175, 180), cv::Point2f(465, 180)};
    std::vector<cv::Point2f> rear_src_points = {
        cv::Point2f(0, 430), cv::Point2f(640, 430),
        cv::Point2f(170, 50), cv::Point2f(470, 50)};
    std::vector<cv::Point2f> dst_points = {
        cv::Point2f(0, IMG_HEIGHT), cv::Point2f(IMG_WIDTH, IMG_HEIGHT),
        cv::Point2f(0, 0), cv::Point2f(IMG_WIDTH, 0)};
    front_M_ = cv::getPerspectiveTransform(front_src_points, dst_points);
    rear_M_ = cv::getPerspectiveTransform(rear_src_points, dst_points);

    // 모드 설정
    std::string mode_str = this->get_parameter("mode").as_string();
    if (mode_str == "driving")
    {
        mode_ = Mode::DRIVING;
    }
    else if (mode_str == "parking")
    {
        mode_ = Mode::PARKING;
    }
    else
    {
        mode_ = Mode::OFF;
    }
    expected_lane_width_ = this->get_parameter("expected_lane_width").as_int();
    expected_parking_lane_width_ = this->get_parameter("expected_parking_lane_width").as_int();
    lane_request_ = this->get_parameter("lane_request").as_bool();

    // 파라미터 핸들러
    params_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            for (const auto &param : parameters)
            {
                if (param.get_name() == "mode")
                {
                    std::string mode_str = param.as_string();
                    if (mode_str == "driving")
                    {
                        mode_ = Mode::DRIVING;
                    }
                    else if (mode_str == "parking")
                    {
                        mode_ = Mode::PARKING;
                    }
                    else
                    {
                        mode_ = Mode::OFF;
                    }
                    RCLCPP_INFO(this->get_logger(), "Mode changed to: %s", param.as_string().c_str());
                }
                else if (param.get_name() == "expected_lane_width")
                {
                    expected_lane_width_ = param.as_int();
                    RCLCPP_INFO(this->get_logger(), "expected_lane_width changed to: %d", expected_lane_width_);
                }
                else if (param.get_name() == "expected_parking_lane_width")
                {
                    expected_parking_lane_width_ = param.as_int();
                    RCLCPP_INFO(this->get_logger(), "expected_parking_lane_width changed to: %d", expected_parking_lane_width_);
                }
                else if (param.get_name() == "lane_request")
                {
                    lane_request_ = param.as_bool();
                    RCLCPP_INFO(this->get_logger(), "lane_request changed to: %d", lane_request_);
                }
            }
            return result;
        });
}

// <FIND> 전방 이미지 처리
void VisionNode::processFrontImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    if (mode_ == Mode::DRIVING) // 주행 모드
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

        ResultVisionProcess results = detectDrivingLanes(cv_ptr->image);
        std::vector<cv::Point2i> waypoints; // 차량 좌표계
        float sum_window_confidences = 0;
        waypoints = results.waypoints;
        sum_window_confidences = results.window_confidence;

        std::vector<cv::Point2f> waypoints_img_float; // 이미지 좌표계 (float)
        std::vector<cv::Point2f> original_points;     // 역변환된 waypoint
        std::vector<cv::Point2i> result_waypoints;
        // 역변환 좌표계 변환
        for (const auto &point : waypoints)
        {
            cv::Point2i img_point = vehicleToImage(point); // 이미지 좌표계로 변환
            cv::Point2f float_point = cv::Point2f(static_cast<float>(img_point.x), static_cast<float>(img_point.y));
            waypoints_img_float.push_back(float_point);
        }
        // IPM 역변환 적용
        cv::perspectiveTransform(waypoints_img_float, original_points, front_M_.inv());
        for (const auto &point : original_points)
        {
            // 정규화 적용
            cv::Point2f norm = pixelToNormalized(point, UsingCamera::Front);
            cv::Point2i original_points = cv::Point2i(static_cast<int>(norm.x * 100), static_cast<int>(norm.y * 100));
            result_waypoints.push_back(original_points);
        }

        // sum_window_confidences 변경
        sum_window_confidences = sum_window_confidences * 10 - 30;

        RCLCPP_INFO(this->get_logger(), "1 : y : %d, x : %d", result_waypoints[0].y, result_waypoints[0].x);
        RCLCPP_INFO(this->get_logger(), "2 : y : %d, x : %d", result_waypoints[1].y, result_waypoints[1].x);
        RCLCPP_INFO(this->get_logger(), "3 : y : %d, x : %d", result_waypoints[2].y, result_waypoints[2].x);
        RCLCPP_INFO(this->get_logger(), "4 : y : %d, x : %d", result_waypoints[3].y, result_waypoints[3].x);
        RCLCPP_INFO(this->get_logger(), "sum_window_confidences : %d", static_cast<int>(sum_window_confidences));
#ifndef DEBUG_CGW
        // VCU로 전송
        auto msg1 = std::make_shared<CCU_Cordi_data1_Msg>();
        auto msg2 = std::make_shared<CCU_Cordi_data2_Msg>();
        msg1->SetCordiY1(result_waypoints[0].y);
        msg1->SetCordiX1(result_waypoints[0].x);
        msg1->SetCordiY2(result_waypoints[1].y);
        msg1->SetCordiX2(result_waypoints[1].x);

        msg2->SetCordiY3(result_waypoints[2].y);
        msg2->SetCordiX3(result_waypoints[2].x);
        msg2->SetCordiY4(result_waypoints[3].y);
        msg2->SetCordiX4(result_waypoints[3].x);

        RCLCPP_INFO(this->get_logger(), "1 : y : %d, x : %d", result_waypoints[0].y, result_waypoints[0].x);
        RCLCPP_INFO(this->get_logger(), "2 : y : %d, x : %d", result_waypoints[1].y, result_waypoints[1].x);
        RCLCPP_INFO(this->get_logger(), "3 : y : %d, x : %d", result_waypoints[2].y, result_waypoints[2].x);
        RCLCPP_INFO(this->get_logger(), "4 : y : %d, x : %d", result_waypoints[3].y, result_waypoints[3].x);
        RCLCPP_INFO(this->get_logger(), "sum_window_confidences : %d", static_cast<int>(sum_window_confidences));

        msg2->SetUsingCamera(1); // UsingCamera::Front
        msg2->SetTrustValue(static_cast<int>(sum_window_confidences));
        std::shared_ptr<IMessage> imsg1 = msg1;
        std::shared_ptr<IMessage> imsg2 = msg2;
        CGW->can_socket_->async_send(imsg1);
        CGW->can_socket_->async_send(imsg2);
#endif
        // cv::Mat debug_img = cv_ptr->image.clone(); // 처리 완료됨
        // drawDebugImage(debug_img, waypoints, UsingCamera::Front);
        // publishWaypoints(result_waypoints, UsingCamera::Front);
    }
    else if (mode_ == Mode::PARKING) // 주차 모드
    {
        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        // std::vector<cv::Point2i> waypoints;
        // waypoints = detectFrontParkingLanes(cv_ptr->image);
        // cv::Mat debug_img = cv_ptr->image.clone(); // 처리 완료됨
        // drawDebugImage(debug_img, waypoints, UsingCamera::Front);
        // publishWaypoints(waypoints, UsingCamera::Front);
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
// 수행 시간 체크용 PRINT
#ifdef DEBUG_TIME
    RCLCPP_INFO(this->get_logger(), "processFrontImage Processing Time: %ld ms", duration.count());
#endif
}

// <FIND> 후방 이미지 처리
void VisionNode::processRearImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    if (mode_ == Mode::PARKING)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        ResultVisionProcess results = detectRearParkingLanes(cv_ptr->image);
        ///////////////////
        std::vector<cv::Point2i> waypoints; // 차량 좌표계
        float sum_window_confidences = 0;
        waypoints = results.waypoints;
        sum_window_confidences = results.window_confidence;

        std::vector<cv::Point2f> waypoints_img_float; // 이미지 좌표계 (float)
        std::vector<cv::Point2f> original_points;     // 역변환된 waypoint
        std::vector<cv::Point2i> result_waypoints;
        // 역변환 좌표계 변환
        for (const auto &point : waypoints)
        {
            cv::Point2i img_point = vehicleToImage(point); // 이미지 좌표계로 변환
            cv::Point2f float_point = cv::Point2f(static_cast<float>(img_point.x), static_cast<float>(img_point.y));
            waypoints_img_float.push_back(float_point);
        }
        // IPM 역변환 적용
        cv::perspectiveTransform(waypoints_img_float, original_points, rear_M_.inv());
        for (const auto &point : original_points)
        {
            // 정규화 적용
            cv::Point2f norm = pixelToNormalized(point, UsingCamera::Front);
            cv::Point2i original_points = cv::Point2i(static_cast<int>(norm.x * 100), static_cast<int>(norm.y * 100));
            result_waypoints.push_back(original_points);
        }

        // sum_window_confidences 변경
        sum_window_confidences = sum_window_confidences * 10 - 30;

        RCLCPP_INFO(this->get_logger(), "1 : y : %d, x : %d", result_waypoints[0].y, result_waypoints[0].x);
        RCLCPP_INFO(this->get_logger(), "2 : y : %d, x : %d", result_waypoints[1].y, result_waypoints[1].x);
        RCLCPP_INFO(this->get_logger(), "3 : y : %d, x : %d", result_waypoints[2].y, result_waypoints[2].x);
        RCLCPP_INFO(this->get_logger(), "4 : y : %d, x : %d", result_waypoints[3].y, result_waypoints[3].x);
        RCLCPP_INFO(this->get_logger(), "sum_window_confidences : %d", static_cast<int>(sum_window_confidences));
#ifndef DEBUG_CGW
        // VCU로 전송
        auto msg1 = std::make_shared<CCU_Cordi_data1_Msg>();
        auto msg2 = std::make_shared<CCU_Cordi_data2_Msg>();
        msg1->SetCordiY1(result_waypoints[0].y);
        msg1->SetCordiX1(result_waypoints[0].x);
        msg1->SetCordiY2(result_waypoints[1].y);
        msg1->SetCordiX2(result_waypoints[1].x);

        msg2->SetCordiY3(result_waypoints[2].y);
        msg2->SetCordiX3(result_waypoints[2].x);
        msg2->SetCordiY4(result_waypoints[3].y);
        msg2->SetCordiX4(result_waypoints[3].x);

        // RCLCPP_INFO(this->get_logger(), "1 : y : %d, x : %d", result_waypoints[0].y, result_waypoints[0].x);
        // RCLCPP_INFO(this->get_logger(), "2 : y : %d, x : %d", result_waypoints[1].y, result_waypoints[1].x);
        // RCLCPP_INFO(this->get_logger(), "3 : y : %d, x : %d", result_waypoints[2].y, result_waypoints[2].x);
        // RCLCPP_INFO(this->get_logger(), "4 : y : %d, x : %d", result_waypoints[3].y, result_waypoints[3].x);
        // RCLCPP_INFO(this->get_logger(), "sum_window_confidences : %d", static_cast<int>(sum_window_confidences));

        msg2->SetUsingCamera(2); // UsingCamera::Rear
        msg2->SetTrustValue(static_cast<int>(sum_window_confidences));
        std::shared_ptr<IMessage> imsg1 = msg1;
        std::shared_ptr<IMessage> imsg2 = msg2;
        CGW->can_socket_->async_send(imsg1);
        CGW->can_socket_->async_send(imsg2);
#endif
        // cv::Mat debug_img = cv_ptr->image.clone(); // 처리 완료됨
        // drawDebugImage(debug_img, waypoints, UsingCamera::Rear);
        // publishWaypoints(result_waypoints, UsingCamera::Rear);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // 수행 시간 체크용 PRINT
    // RCLCPP_INFO(this->get_logger(), "processRearImage Processing Time: %ld ms", duration.count());
}
// 전방 카메라 주행 모드 차선 검출
ResultVisionProcess VisionNode::detectDrivingLanes(const cv::Mat &img)
{
    static int frame_counter_front = 0;
    frame_counter_front++;

    // 1. 이미지 전처리
    cv::Mat warped;
    cv::warpPerspective(img, warped, front_M_, cv::Size(IMG_WIDTH, IMG_HEIGHT));

    // HSV 색상 공간으로 변환 (흰색 마스크 색상정보 정확히 추출)
    cv::Mat hsv;
    cv::cvtColor(warped, hsv, cv::COLOR_BGR2HSV);

    // 흰색 차선 마스크 생성
    cv::Mat white_mask;
    cv::inRange(hsv, cv::Scalar(0, 0, 160), // 흰색의 낮은 HSV 범위
                cv::Scalar(180, 30, 255),   // 흰색의 높은 HSV 범위
                white_mask);

    // 원본 그레이스케일에 마스크 적용
    cv::Mat gray; // (밝기 정보 유지)
    cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
    cv::Mat masked_gray;

    // BitWise처리
    cv::bitwise_and(gray, gray, masked_gray, white_mask);

    // 가우시안 블러로 노이즈 제거
    cv::GaussianBlur(masked_gray, masked_gray, cv::Size(5, 5), 0);

    // 모폴로지 연산
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // 작은 커널 사용
    // cv::morphologyEx(masked_gray, masked_gray, cv::MORPH_OPEN, kernel); // 침식 -> 팽창
    cv::morphologyEx(masked_gray, masked_gray, cv::MORPH_CLOSE, kernel); // 팽창 -> 침식

    // Canny 엣지 검출
    cv::Mat edges;
    cv::Canny(masked_gray, edges, 50, 150);

// 디버그 이미지용
#ifdef DEBUG_IMAGE
    cv::Mat debug_img = warped.clone();
#endif
    // 가로선(직각선) 검출 ------------------------------------------------------------------------

    // ROI 설정 - 이미지의 위쪽 절반만 사용
    cv::Mat right_angle_roi = edges(cv::Rect(0, 0, IMG_WIDTH, IMG_HEIGHT / 2));

    std::vector<cv::Vec4i> horizontal_lines;
    cv::HoughLinesP(right_angle_roi, horizontal_lines, 1, CV_PI / 180, 50, 50, 10);

    bool right_angle_line_detected = false;
    float right_angle_line_angle = 0.0f;

    // 가로선 필터링 및 처리
    for (const auto &line : horizontal_lines)
    {
        float dx = line[2] - line[0];
        float dy = line[3] - line[1];
        float angle = std::atan2(dy, dx) * 180 / CV_PI;
        float length = std::sqrt(dx * dx + dy * dy);

        // 수평에 가까운 선(-20도 ~ 20도)이고 충분히 긴 경우
        if (std::abs(angle) < 20 && length > IMG_WIDTH / 2)
        {
            // 주차선 발견
            right_angle_line_detected = true;
            right_angle_line_angle = angle;
            RCLCPP_INFO(this->get_logger(), "Right Angle line detected! Angle: %.2f", right_angle_line_angle);
#ifdef DEBUG_IMAGE
            // 디버그 이미지에 주차선 표시
            cv::line(debug_img,
                     cv::Point(line[0], line[1]),
                     cv::Point(line[2], line[3]),
                     cv::Scalar(0, 0, 255), 3); // 빨간색으로 표시
#endif
            break;
        }
    }

    if (!right_angle_detected_flag && right_angle_line_detected) // 검출 플래그 꺼져있고, 라인찾음
    {
        right_angle_detected_flag = true;
#ifndef DEBUG_CGW
        // VCU로 전송
        // right_angle_lane_detected 0 : 미검출, 1: 검출
        auto msg = std::make_shared<CCU_RightAngle_detect_Msg>();
        msg->SetRightAngleLaneDetected(static_cast<uint8_t>(right_angle_detected_flag));
        std::shared_ptr<IMessage> imsg = msg;
        CGW->can_socket_->async_send(imsg);
#endif
    }
    else if (right_angle_detected_flag && !right_angle_line_detected) // 검출 플래그 켜져있고, 라인못찾음
    {
        right_angle_detected_flag = false;
    }

    // CUDA 사용 버전
    // cv::cuda::GpuMat d_white_mask(white_mask);  // GPU로 데이터 전송
    // cv::cuda::GpuMat d_lines;  // GPU 메모리에 lines 할당

    // // GPU에서 HoughLinesP 실행
    // cv::Ptr<cv::cuda::HoughSegmentDetector> hough = cv::cuda::createHoughSegmentDetector(1, CV_PI/180.0f, 50, 100);
    // hough->detect(d_white_mask, d_lines);

    // // GPU에서 결과 가져오기
    // std::vector<cv::Vec4i> horizontal_lines;
    // if (!d_lines.empty()) {
    //     cv::Mat h_lines(d_lines);
    //     horizontal_lines.resize(h_lines.cols);
    //     for(int i = 0; i < h_lines.cols; i++) {
    //         cv::Vec4f line = h_lines.at<cv::Vec4f>(0, i);
    //         horizontal_lines[i] = cv::Vec4i(line[0], line[1], line[2], line[3]);
    //     }
    // }

    // ----------------------------------------------

    // 2. ROI 기반 차선 검출
    const int windows = 8;
    int window_height = warped.rows / windows;
    const int window_width = 100;                // 윈도우 크기는 고정
    const int search_range = window_width * 2.5; // 탐색 범위는 더 넓게   Edit Param
    size_t minpix = 50;                          // 최소 픽셀 수          Edit Param

    std::vector<cv::Point2i> left_points, right_points;
    std::vector<float> window_confidences;
    float sum_window_confidences = 0;
    window_confidences.reserve(windows);

    int leftx_current, rightx_current; // 이미지 상의 좌표계

    // 첫 프레임에서만 히스토그램으로 초기 위치 찾기
    cv::Mat bottom = edges(cv::Rect(0, edges.rows - window_height, edges.cols, window_height));
    std::vector<int> histogram(edges.cols, 0);
    for (int x = 0; x < edges.cols; x++)
    {
        histogram[x] = cv::sum(bottom.col(x))[0];
    }

    int midpoint = edges.cols / 2;
    leftx_current = std::max_element(histogram.begin(),
                                     histogram.begin() + midpoint) -
                    histogram.begin();
    rightx_current = std::max_element(histogram.begin() + midpoint,
                                      histogram.end()) -
                     histogram.begin();

    if (!isValidLaneWidth(cv::Point2i(leftx_current - IMG_WIDTH / 2, 0), cv::Point2i(rightx_current - IMG_WIDTH / 2, 0), expected_lane_width_)) // 비정상적인 차선일때,
    {
        // 이전 프레임의 가장 아래 포인트를 시작점으로 사용
        if (!prev_left_points_front_.empty() && !prev_right_points_front_.empty())
        {
            cv::Point2i prev_left = findNearestPoint(prev_left_points_front_, window_height / 2);
            cv::Point2i prev_right = findNearestPoint(prev_right_points_front_, window_height / 2);
            leftx_current = prev_left.x + IMG_WIDTH / 2; // 이미지 좌표계로 변환
            rightx_current = prev_right.x + IMG_WIDTH / 2;
        }
        else
        {
            // 이미지 중앙을 기준으로 expected_lane_width_ 만큼 좌우로 설정
            int center = edges.cols / 2;
            leftx_current = center - expected_lane_width_ / 2;
            rightx_current = center + expected_lane_width_ / 2;
        }
    }

    for (int window = 0; window < windows; window++)
    {
        // 이미지 좌표계
        int win_y_low = edges.rows - (window + 1) * window_height;
        int win_y_high = edges.rows - window * window_height;
        int center_y = (win_y_low + win_y_high) / 2;

        cv::Rect left_roi(leftx_current - search_range / 2, win_y_low,
                          search_range, window_height);
        cv::Rect right_roi(rightx_current - search_range / 2, win_y_low,
                           search_range, window_height);

        left_roi &= cv::Rect(0, 0, edges.cols, edges.rows);
        right_roi &= cv::Rect(0, 0, edges.cols, edges.rows);

        bool found_left = false;
        bool found_right = false;
        cv::Point2i left_point, right_point;
        float now_window_confidence = 1.0;

        // 이전 프레임의 x축 근처 지점에서 윈도우 중심(x좌표) 찾기
        if (left_roi.width > 0 && left_roi.height > 0)
        {

            cv::Mat left_window = edges(left_roi);
            std::vector<cv::Point> left_nonzero;
            cv::findNonZero(left_window, left_nonzero);

            if (left_nonzero.size() > minpix)
            {
                int mean_x = 0;
                for (const auto &point : left_nonzero)
                {
                    mean_x += point.x;
                }
                leftx_current = left_roi.x + mean_x / left_nonzero.size();
                left_point = cv::Point2i(leftx_current - IMG_WIDTH / 2, IMG_HEIGHT - center_y); // 차량 중심 좌표계 변환
                found_left = true;
            }
        }

        if (right_roi.width > 0 && right_roi.height > 0)
        {
            cv::Mat right_window = edges(right_roi);
            std::vector<cv::Point> right_nonzero;
            cv::findNonZero(right_window, right_nonzero);

            if (right_nonzero.size() > minpix)
            {
                int mean_x = 0;
                for (const auto &point : right_nonzero)
                {
                    mean_x += point.x;
                }
                rightx_current = right_roi.x + mean_x / right_nonzero.size();
                right_point = cv::Point2i(rightx_current - IMG_WIDTH / 2, IMG_HEIGHT - center_y); // 차량 중심 좌표계 변환
                found_right = true;
            }
        }
        // 차선 탐지 여부에 따른 처리
        if (found_left && found_right)
        {
            int width = std::abs(right_point.x - left_point.x);
            float width_change_rate = 0.0f; // 차선 변화폭 변화량

            // 이전 프레임과의 변화율 계산
            if (!first_frame_front_ && !prev_left_points_front_.empty() && !prev_right_points_front_.empty())
            {
                cv::Point2i prev_left = findNearestPoint(prev_left_points_front_, IMG_HEIGHT - center_y);
                cv::Point2i prev_right = findNearestPoint(prev_right_points_front_, IMG_HEIGHT - center_y);
                int prev_width = std::abs(prev_right.x - prev_left.x);
                width_change_rate = std::abs(width - prev_width) / static_cast<float>(prev_width);
            }
            if (!isValidLaneWidth(left_point, right_point, expected_lane_width_))
            {
                // 급커브 가능성 (과거에 비해 점진적인 변화)
                if (width_change_rate < 0.3) // 0.3 parameter Edit Param
                {
                    now_window_confidence *= 0.7; // 급커브 상황
                }
                // 갑작스러운 변화 (오검출 가능성)
                else
                {
                    now_window_confidence *= 0.4; // 심각한 신뢰도 하락

                    // 이전 프레임 정보 활용
                    if (!first_frame_front_ && !prev_left_points_front_.empty() && !prev_right_points_front_.empty())
                    {
                        cv::Point2i prev_left = findNearestPoint(prev_left_points_front_, IMG_HEIGHT - center_y);
                        cv::Point2i prev_right = findNearestPoint(prev_right_points_front_, IMG_HEIGHT - center_y);

                        // 이전 프레임과 현재 프레임의 중간값 사용
                        left_point.x = (left_point.x + prev_left.x) / 2;
                        right_point.x = (right_point.x + prev_right.x) / 2;
                    }
                    // 이전 프레임이 없는 경우
                    else
                    {
                        int center_x = (left_point.x + right_point.x) / 2;
                        left_point.x = center_x - expected_lane_width_ / 2;
                        right_point.x = center_x + expected_lane_width_ / 2;
                    }
                }
            }
            else // 정상적인 탐지 -> 차선 폭 학습 // 신뢰도 1
            {
                // 가까운 윈도우(아래쪽)에서만 학습
                if (window < windows / 2) // 아래쪽 절반의 윈도우만 사용
                {
                    float learning_rate = 0.01f; // 학습률

                    // 이전 프레임과 비교해서 급격한 변화가 없을 때만 학습
                    if (width_change_rate < 0.1f)
                    {
                        expected_lane_width_ = static_cast<int>(
                            expected_lane_width_ * (1.0f - learning_rate) +
                            width * learning_rate);
                        // 디버그용
                        if (frame_counter_front % 10 == 0)
                        {
                            // RCLCPP_INFO(this->get_logger(), "Expected Lane Width: %d", expected_lane_width_);
                            // RCLCPP_INFO(this->get_logger(), "Real Lane Width: %d", width);
                        }
                    }
                }
            }
        }
        // 차선 소실 대응
        else if (!found_left && found_right)
        {
            // 오른쪽 차선만 발견된 경우
            left_point = cv::Point2i(right_point.x - expected_lane_width_, right_point.y);
            found_left = true;
            now_window_confidence *= 0.8;
        }
        else if (found_left && !found_right)
        {
            // 왼쪽 차선만 발견된 경우
            right_point = cv::Point2i(left_point.x + expected_lane_width_, left_point.y);
            found_right = true;
            now_window_confidence *= 0.8;
        }
        else if (!found_left && !found_right)
        {
            // 둘 다 발견되지 않은 경우
            if (!first_frame_front_ && !prev_left_points_front_.empty() && !prev_right_points_front_.empty())
            {
                // 이전 프레임의 해당 높이의 포인트 찾기
                left_point = findNearestPoint(prev_left_points_front_, IMG_HEIGHT - center_y);
                right_point = findNearestPoint(prev_right_points_front_, IMG_HEIGHT - center_y);
                found_left = found_right = true;
                now_window_confidence *= 0.6;
            }
            else
            {
                // 기본값 사용
                left_point = cv::Point2i(-expected_lane_width_ / 2, IMG_HEIGHT - center_y);
                right_point = cv::Point2i(expected_lane_width_ / 2, IMG_HEIGHT - center_y);
                now_window_confidence *= 0.4;
            }
        }

        if (found_left)
            left_points.push_back(left_point);
        if (found_right)
            right_points.push_back(right_point);

        // 디버그용 윈도우 표시 DEBUG
        leftx_current = left_point.x + IMG_WIDTH / 2;
        rightx_current = right_point.x + IMG_WIDTH / 2;
#ifdef DEBUG_IMAGE
        if (static_cast<int>(debug_draw_option_) & static_cast<int>(DebugDrawOption::ROI_BOX))
        {

            cv::rectangle(debug_img,
                          cv::Point(leftx_current - search_range / 2, win_y_low),
                          cv::Point(leftx_current + search_range / 2, win_y_high),
                          cv::Scalar(128, 128, 128), 1); // 회색으로 탐색 범위 표시

            // 오른쪽 차선 탐색 범위
            cv::rectangle(debug_img,
                          cv::Point(rightx_current - search_range / 2, win_y_low),
                          cv::Point(rightx_current + search_range / 2, win_y_high),
                          cv::Scalar(128, 128, 128), 1); // 회색으로 탐색 범위 표시
        }
        if (static_cast<int>(debug_draw_option_) & static_cast<int>(DebugDrawOption::SLIDING_WINDOWS_BOX))
        {

            // 실제 윈도우 표시
            cv::rectangle(debug_img,
                          cv::Point(leftx_current - window_width / 2, win_y_low),
                          cv::Point(leftx_current + window_width / 2, win_y_high),
                          getColorByConfidence(now_window_confidence), 2);
            cv::rectangle(debug_img,
                          cv::Point(rightx_current - window_width / 2, win_y_low),
                          cv::Point(rightx_current + window_width / 2, win_y_high),
                          getColorByConfidence(now_window_confidence), 2);
        }
#endif

        window_confidences.push_back(now_window_confidence);
        sum_window_confidences += now_window_confidence;
    }

    // 이전 프레임 정보와 혼합 (프레임별로 급격한 변화 없애기)
    if (!first_frame_front_)
    {
        smoothPoints(left_points, prev_left_points_front_, window_confidences);
        smoothPoints(right_points, prev_right_points_front_, window_confidences);
    }

    // 웨이포인트 생성
    std::vector<cv::Point2i> waypoints;
    std::vector<cv::Point2i> raw_waypoints;
    cv::Mat coef;
    waypoints.reserve(windows);
    raw_waypoints.reserve(windows);
    if (!left_points.empty() && !right_points.empty())
    {
        for (int i = 0; i < windows; i++)
        {
            int target_y = IMG_HEIGHT - window_height / 2 - i * window_height;
            cv::Point2i left = findNearestPoint(left_points, target_y);
            cv::Point2i right = findNearestPoint(right_points, target_y);
            raw_waypoints.push_back(cv::Point2i((left.x + right.x) / 2, target_y));
        }

        // 2. regression 적용 (window_confidences를 가중치로 사용)
        auto result = regressionWaypoints(raw_waypoints, window_confidences, 4);
        waypoints = result.waypoints;
        coef = result.coefficients;
    }

    // 현재 프레임 정보 저장
    prev_left_points_front_ = left_points;
    prev_right_points_front_ = right_points;
    prev_confidence_front_ = window_confidences;
    first_frame_front_ = false;

// ----------------------------------------------------------
// 디버그 발행 (DEBUG)
#ifdef DEBUG_IMAGE
    for (const auto &point : raw_waypoints) // 8개의 점
    {
        if (static_cast<int>(debug_draw_option_) & static_cast<int>(DebugDrawOption::MID_POINT_OF_LANE))
        {
            cv::circle(debug_img, cv::Point(point.x + IMG_WIDTH / 2, IMG_HEIGHT - point.y),
                       3, cv::Scalar(0, 255, 0), -1);
        }
    }
#endif
    // regression된 곡선 그리기 (마젠타)
    if (!waypoints.empty())
    {
        // 부드러운 곡선을 위해 더 많은 점 생성
        for (int y = 0; y < IMG_HEIGHT; y += 5) // 5픽셀 간격
        {
            double x = coef.at<double>(0, 0) * y * y +
                       coef.at<double>(1, 0) * y +
                       coef.at<double>(2, 0);

            // 이전 점이 있으면 선으로 연결
            if (y > 0)
            {
                double prev_x = coef.at<double>(0, 0) * (y - 5) * (y - 5) +
                                coef.at<double>(1, 0) * (y - 5) +
                                coef.at<double>(2, 0);
#ifdef DEBUG_IMAGE
                if (static_cast<int>(debug_draw_option_) & static_cast<int>(DebugDrawOption::REGRESSION_CURVE))
                {
                    cv::line(debug_img,
                             cv::Point(prev_x + IMG_WIDTH / 2, IMG_HEIGHT - (y - 5)),
                             cv::Point(x + IMG_WIDTH / 2, IMG_HEIGHT - y),
                             cv::Scalar(255, 0, 255), 2);
                }
#endif
            }
        }

// regression된 waypoints (큰 마젠타 점)
#ifdef DEBUG_IMAGE
        for (const auto &point : waypoints)
        {
            if (static_cast<int>(debug_draw_option_) & static_cast<int>(DebugDrawOption::WAY_POINT))
            {
                cv::Point2i img_point = vehicleToImage(point);
                cv::circle(debug_img, img_point, 5, cv::Scalar(255, 0, 255), -1);
            }
        }
#endif
    }
#ifdef DEBUG_IMAGE
    sensor_msgs::msg::Image::SharedPtr processed_debug_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_img).toImageMsg();

    // 역변환 수행
    cv::Mat original_image_with_debug;
    cv::warpPerspective(debug_img, original_image_with_debug, front_M_.inv(), cv::Size(IMG_WIDTH, IMG_HEIGHT));

    sensor_msgs::msg::Image::SharedPtr original_image_with_debug_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", original_image_with_debug).toImageMsg();
    // if (frame_counter_front % 3 == 0)
    // {
    std_msgs::msg::Int32 param_width_msg;
    param_width_msg.data = expected_lane_width_;
    width_plot_pub_->publish(param_width_msg); // 차선 폭

    front_debug_pub_->publish(*processed_debug_msg); // Processed 된 화면
    front_debug_on_real_pub_->publish(*original_image_with_debug_msg);
#endif
    // }

    return {waypoints, sum_window_confidences};
}

// 후방 카메라 주차 모드 차선 검출
ResultVisionProcess VisionNode::detectRearParkingLanes(const cv::Mat &img)
{
    static int frame_counter_rear = 0;
    frame_counter_rear++;

    // 1. 이미지 전처리
    cv::Mat warped;
    cv::warpPerspective(img, warped, rear_M_, cv::Size(IMG_WIDTH, IMG_HEIGHT));

    // HSV 색상 공간으로 변환
    cv::Mat hsv;
    cv::cvtColor(warped, hsv, cv::COLOR_BGR2HSV);

    // 흰색 차선 마스크 생성
    cv::Mat white_mask;
    cv::inRange(hsv, cv::Scalar(0, 0, 160), // 흰색의 낮은 HSV 범위
                cv::Scalar(180, 30, 255),   // 흰색의 높은 HSV 범위
                white_mask);

    // 원본 그레이스케일에 마스크 적용
    cv::Mat gray;
    cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
    cv::Mat masked_gray;

    // BitWise처리
    cv::bitwise_and(gray, gray, masked_gray, white_mask);

    // 가우시안 블러로 노이즈 제거
    cv::GaussianBlur(masked_gray, masked_gray, cv::Size(5, 5), 0);

    // 모폴로지 연산
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // 작은 커널 사용
    // cv::morphologyEx(masked_gray, masked_gray, cv::MORPH_OPEN, kernel);  // 침식 -> 팽창
    cv::morphologyEx(masked_gray, masked_gray, cv::MORPH_CLOSE, kernel); // 팽창 -> 침식

    // Canny 엣지 검출
    cv::Mat edges;
    cv::Canny(masked_gray, edges, 50, 150);

// 디버그 이미지용
#ifdef DEBUG_IMAGE
    cv::Mat debug_img = warped.clone();
#endif
    // 가로선(직각선) 검출 ------------------------------------------------------------------------
    // 전체 ROI 설정
    cv::Mat full_roi = edges;
    float parking_line_angle = 100.0f; // 미검출 값
    bool line_in_bottom = false;       // 아래 라인에 있다면

    if (lane_request_)
    {
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(full_roi, lines, 1, CV_PI / 180, 50, 50, 10);

        for (const auto &line : lines)
        {
            float dx = line[2] - line[0];
            float dy = line[3] - line[1];
            float angle = std::atan2(dy, dx) * 180 / CV_PI;
            float length = std::sqrt(dx * dx + dy * dy);

            // 수평에 가까운 선(-20도 ~ 20도)이고 충분히 긴 경우
            if (std::abs(angle) < 20 && length > IMG_WIDTH / 2)
            {
                parking_line_angle = angle;

                // 선의 y좌표가 이미지 아래쪽에 있는지 확인 Edit Param
                float y_avg = (line[1] + line[3]) / 2.0f;
                if (std::abs(angle) < 5 && y_avg > IMG_HEIGHT * 2 / 3)
                {
                    line_in_bottom = true;
                }

                RCLCPP_INFO(this->get_logger(), "Parking line detected! Angle: %.2f, In bottom: %d",
                            parking_line_angle, line_in_bottom);
#ifdef DEBUG_IMAGE
                cv::line(debug_img,
                         cv::Point(line[0], line[1]),
                         cv::Point(line[2], line[3]),
                         cv::Scalar(0, 0, 255), 3);
#endif
                break;
            }
        }
#ifndef DEBUG_CGW
        // VCU로 전송
        // 주차 각도 검출
        auto msg1 = std::make_shared<CCU_ParkingAngle_detect_Msg>();
        msg1->SetParkingBackLaneAngle(static_cast<int>(parking_line_angle));
        std::shared_ptr<IMessage> imsg1 = msg1;
        CGW->can_socket_->async_send(imsg1);
        // 주차 완료
        auto msg2 = std::make_shared<CCU_Parking_Complete_Msg>();
        msg2->SetParkingBackLaneDetected(static_cast<int>(line_in_bottom));
        std::shared_ptr<IMessage> imsg2 = msg2;
        CGW->can_socket_->async_send(imsg2);
#endif
    }

    // CUDA 사용 버전
    // cv::cuda::GpuMat d_white_mask(white_mask);  // GPU로 데이터 전송
    // cv::cuda::GpuMat d_lines;  // GPU 메모리에 lines 할당

    // // GPU에서 HoughLinesP 실행
    // cv::Ptr<cv::cuda::HoughSegmentDetector> hough = cv::cuda::createHoughSegmentDetector(1, CV_PI/180.0f, 50, 100);
    // hough->detect(d_white_mask, d_lines);

    // // GPU에서 결과 가져오기
    // std::vector<cv::Vec4i> horizontal_lines;
    // if (!d_lines.empty()) {
    //     cv::Mat h_lines(d_lines);
    //     horizontal_lines.resize(h_lines.cols);
    //     for(int i = 0; i < h_lines.cols; i++) {
    //         cv::Vec4f line = h_lines.at<cv::Vec4f>(0, i);
    //         horizontal_lines[i] = cv::Vec4i(line[0], line[1], line[2], line[3]);
    //     }
    // }

    // ----------------------------------------------

    // 2. ROI 기반 차선 검출
    const int windows = 8;
    int window_height = warped.rows / windows;
    const int window_width = 100;              // 윈도우 크기는 고정
    const int search_range = window_width * 2; // 탐색 범위는 더 넓게   Edit Param
    size_t minpix = 50;                        // 최소 픽셀 수          Edit Param

    std::vector<cv::Point2i> left_points, right_points;
    std::vector<float> window_confidences;
    float sum_window_confidences = 0;
    window_confidences.reserve(windows);

    int leftx_current, rightx_current; // 이미지 상의 좌표계

    // 첫 프레임에서만 히스토그램으로 초기 위치 찾기
    cv::Mat bottom = edges(cv::Rect(0, edges.rows - window_height, edges.cols, window_height));
    std::vector<int> histogram(edges.cols, 0);
    for (int x = 0; x < edges.cols; x++)
    {
        histogram[x] = cv::sum(bottom.col(x))[0];
    }

    int midpoint = edges.cols / 2;
    leftx_current = std::max_element(histogram.begin(),
                                     histogram.begin() + midpoint) -
                    histogram.begin();
    rightx_current = std::max_element(histogram.begin() + midpoint,
                                      histogram.end()) -
                     histogram.begin();

    if (!isValidLaneWidth(cv::Point2i(leftx_current - IMG_WIDTH / 2, 0), cv::Point2i(rightx_current - IMG_WIDTH / 2, 0), expected_parking_lane_width_)) // 비정상적인 차선일때,
    {
        // 이전 프레임의 가장 아래 포인트를 시작점으로 사용
        if (!prev_left_points_rear_.empty() && !prev_right_points_rear_.empty())
        {
            cv::Point2i prev_left = findNearestPoint(prev_left_points_rear_, window_height / 2);
            cv::Point2i prev_right = findNearestPoint(prev_right_points_rear_, window_height / 2);
            leftx_current = prev_left.x + IMG_WIDTH / 2; // 이미지 좌표계로 변환
            rightx_current = prev_right.x + IMG_WIDTH / 2;
        }
        else
        {
            // 이미지 중앙을 기준으로 expected_parking_lane_width_ 만큼 좌우로 설정
            int center = edges.cols / 2;
            leftx_current = center - expected_parking_lane_width_ / 2;
            rightx_current = center + expected_parking_lane_width_ / 2;
        }
    }

    for (int window = 0; window < windows; window++)
    {
        // 이미지 좌표계
        int win_y_low = edges.rows - (window + 1) * window_height;
        int win_y_high = edges.rows - window * window_height;
        int center_y = (win_y_low + win_y_high) / 2;

        cv::Rect left_roi(leftx_current - search_range / 2, win_y_low,
                          search_range, window_height);
        cv::Rect right_roi(rightx_current - search_range / 2, win_y_low,
                           search_range, window_height);

        left_roi &= cv::Rect(0, 0, edges.cols, edges.rows);
        right_roi &= cv::Rect(0, 0, edges.cols, edges.rows);

        bool found_left = false;
        bool found_right = false;
        cv::Point2i left_point, right_point;
        float now_window_confidence = 1.0;

        // 이전 프레임의 x축 근처 지점에서 윈도우 중심(x좌표) 찾기
        if (left_roi.width > 0 && left_roi.height > 0)
        {

            cv::Mat left_window = edges(left_roi);
            std::vector<cv::Point> left_nonzero;
            cv::findNonZero(left_window, left_nonzero);

            if (left_nonzero.size() > minpix)
            {
                int mean_x = 0;
                for (const auto &point : left_nonzero)
                {
                    mean_x += point.x;
                }
                leftx_current = left_roi.x + mean_x / left_nonzero.size();
                left_point = cv::Point2i(leftx_current - IMG_WIDTH / 2, IMG_HEIGHT - center_y); // 차량 중심 좌표계 변환
                found_left = true;
            }
        }

        if (right_roi.width > 0 && right_roi.height > 0)
        {
            cv::Mat right_window = edges(right_roi);
            std::vector<cv::Point> right_nonzero;
            cv::findNonZero(right_window, right_nonzero);

            if (right_nonzero.size() > minpix)
            {
                int mean_x = 0;
                for (const auto &point : right_nonzero)
                {
                    mean_x += point.x;
                }
                rightx_current = right_roi.x + mean_x / right_nonzero.size();
                right_point = cv::Point2i(rightx_current - IMG_WIDTH / 2, IMG_HEIGHT - center_y); // 차량 중심 좌표계 변환
                found_right = true;
            }
        }
        // 차선 탐지 여부에 따른 처리
        if (found_left && found_right)
        {
            int width = std::abs(right_point.x - left_point.x);
            float width_change_rate = 0.0f; // 차선 변화폭 변화량

            // 이전 프레임과의 변화율 계산
            if (!first_frame_rear_ && !prev_left_points_rear_.empty() && !prev_right_points_rear_.empty())
            {
                cv::Point2i prev_left = findNearestPoint(prev_left_points_rear_, IMG_HEIGHT - center_y);
                cv::Point2i prev_right = findNearestPoint(prev_right_points_rear_, IMG_HEIGHT - center_y);
                int prev_width = std::abs(prev_right.x - prev_left.x);
                width_change_rate = std::abs(width - prev_width) / static_cast<float>(prev_width);
            }
            if (!isValidLaneWidth(left_point, right_point, expected_parking_lane_width_))
            {
                // 급커브 가능성 (과거에 비해 점진적인 변화)
                if (width_change_rate < 0.3) // 0.3 parameter Edit Param
                {
                    now_window_confidence *= 0.7; // 급커브 상황
                }
                // 갑작스러운 변화 (오검출 가능성)
                else
                {
                    now_window_confidence *= 0.4; // 심각한 신뢰도 하락

                    // 이전 프레임 정보 활용
                    if (!first_frame_rear_ && !prev_left_points_rear_.empty() && !prev_right_points_rear_.empty())
                    {
                        cv::Point2i prev_left = findNearestPoint(prev_left_points_rear_, IMG_HEIGHT - center_y);
                        cv::Point2i prev_right = findNearestPoint(prev_right_points_rear_, IMG_HEIGHT - center_y);

                        // 이전 프레임과 현재 프레임의 중간값 사용
                        left_point.x = (left_point.x + prev_left.x) / 2;
                        right_point.x = (right_point.x + prev_right.x) / 2;
                    }
                    // 이전 프레임이 없는 경우
                    else
                    {
                        int center_x = (left_point.x + right_point.x) / 2;
                        left_point.x = center_x - expected_parking_lane_width_ / 2;
                        right_point.x = center_x + expected_parking_lane_width_ / 2;
                    }
                }
            }
            else // 정상적인 탐지 -> 차선 폭 학습 // 신뢰도 1
            {
                // 가까운 윈도우(아래쪽)에서만 학습
                if (window < windows / 2) // 아래쪽 절반의 윈도우만 사용
                {
                    float learning_rate = 0.01f; // 학습률

                    // 이전 프레임과 비교해서 급격한 변화가 없을 때만 학습
                    if (width_change_rate < 0.1f)
                    {
                        expected_parking_lane_width_ = static_cast<int>(
                            expected_parking_lane_width_ * (1.0f - learning_rate) +
                            width * learning_rate);
                        // 디버그용
                        if (frame_counter_rear % 10 == 0)
                        {
                            // RCLCPP_INFO(this->get_logger(), "Expected Lane Width: %d", expected_parking_lane_width_);
                            // RCLCPP_INFO(this->get_logger(), "Real Lane Width: %d", width);
                        }
                    }
                }
            }
        }
        // 차선 소실 대응
        else if (!found_left && found_right)
        {
            // 오른쪽 차선만 발견된 경우
            left_point = cv::Point2i(right_point.x - expected_parking_lane_width_, right_point.y);
            found_left = true;
            now_window_confidence *= 0.8;
        }
        else if (found_left && !found_right)
        {
            // 왼쪽 차선만 발견된 경우
            right_point = cv::Point2i(left_point.x + expected_parking_lane_width_, left_point.y);
            found_right = true;
            now_window_confidence *= 0.8;
        }
        else if (!found_left && !found_right)
        {
            // 둘 다 발견되지 않은 경우
            if (!first_frame_rear_ && !prev_left_points_rear_.empty() && !prev_right_points_rear_.empty())
            {
                // 이전 프레임의 해당 높이의 포인트 찾기
                left_point = findNearestPoint(prev_left_points_rear_, IMG_HEIGHT - center_y);
                right_point = findNearestPoint(prev_right_points_rear_, IMG_HEIGHT - center_y);
                found_left = found_right = true;
                now_window_confidence *= 0.6;
            }
            else
            {
                // 기본값 사용
                left_point = cv::Point2i(-expected_parking_lane_width_ / 2, IMG_HEIGHT - center_y);
                right_point = cv::Point2i(expected_parking_lane_width_ / 2, IMG_HEIGHT - center_y);
                now_window_confidence *= 0.4;
            }
        }

        if (found_left)
            left_points.push_back(left_point);
        if (found_right)
            right_points.push_back(right_point);

        // 디버그용 윈도우 표시 DEBUG
        leftx_current = left_point.x + IMG_WIDTH / 2;
        rightx_current = right_point.x + IMG_WIDTH / 2;
#ifdef DEBUG_IMAGE
        if (static_cast<int>(debug_draw_option_) & static_cast<int>(DebugDrawOption::ROI_BOX))
        {

            cv::rectangle(debug_img,
                          cv::Point(leftx_current - search_range / 2, win_y_low),
                          cv::Point(leftx_current + search_range / 2, win_y_high),
                          cv::Scalar(128, 128, 128), 1); // 회색으로 탐색 범위 표시

            // 오른쪽 차선 탐색 범위
            cv::rectangle(debug_img,
                          cv::Point(rightx_current - search_range / 2, win_y_low),
                          cv::Point(rightx_current + search_range / 2, win_y_high),
                          cv::Scalar(128, 128, 128), 1); // 회색으로 탐색 범위 표시
        }
        if (static_cast<int>(debug_draw_option_) & static_cast<int>(DebugDrawOption::SLIDING_WINDOWS_BOX))
        {

            // 실제 윈도우 표시
            cv::rectangle(debug_img,
                          cv::Point(leftx_current - window_width / 2, win_y_low),
                          cv::Point(leftx_current + window_width / 2, win_y_high),
                          getColorByConfidence(now_window_confidence), 2);
            cv::rectangle(debug_img,
                          cv::Point(rightx_current - window_width / 2, win_y_low),
                          cv::Point(rightx_current + window_width / 2, win_y_high),
                          getColorByConfidence(now_window_confidence), 2);
        }
#endif
        window_confidences.push_back(now_window_confidence);
        sum_window_confidences += now_window_confidence;
    }

    // 이전 프레임 정보와 혼합 (프레임별로 급격한 변화 없애기)
    if (!first_frame_rear_)
    {
        smoothPoints(left_points, prev_left_points_rear_, window_confidences);
        smoothPoints(right_points, prev_right_points_rear_, window_confidences);
    }

    // 웨이포인트 생성
    std::vector<cv::Point2i> waypoints;
    std::vector<cv::Point2i> raw_waypoints;
    cv::Mat coef;
    waypoints.reserve(windows);
    raw_waypoints.reserve(windows);
    if (!left_points.empty() && !right_points.empty())
    {
        for (int i = 0; i < windows; i++)
        {
            int target_y = IMG_HEIGHT - window_height / 2 - i * window_height;
            cv::Point2i left = findNearestPoint(left_points, target_y);
            cv::Point2i right = findNearestPoint(right_points, target_y);
            raw_waypoints.push_back(cv::Point2i((left.x + right.x) / 2, target_y));
        }

        // 2. regression 적용 (window_confidences를 가중치로 사용)
        auto result = regressionWaypoints(raw_waypoints, window_confidences, 4);
        waypoints = result.waypoints;
        coef = result.coefficients;
    }

    // 현재 프레임 정보 저장
    prev_left_points_rear_ = left_points;
    prev_right_points_rear_ = right_points;
    prev_confidence_rear_ = window_confidences;
    first_frame_rear_ = false;

// ----------------------------------------------------------
// 디버그 발행 (DEBUG)
#ifdef DEBUG_IMAGE
    for (const auto &point : raw_waypoints) // 8개의 점
    {
        if (static_cast<int>(debug_draw_option_) & static_cast<int>(DebugDrawOption::MID_POINT_OF_LANE))
        {
            cv::circle(debug_img, cv::Point(point.x + IMG_WIDTH / 2, IMG_HEIGHT - point.y),
                       3, cv::Scalar(0, 255, 0), -1);
        }
    }
#endif
    // regression된 곡선 그리기 (마젠타)
    if (!waypoints.empty())
    {
        // 부드러운 곡선을 위해 더 많은 점 생성
        for (int y = 0; y < IMG_HEIGHT; y += 5) // 5픽셀 간격
        {
            double x = coef.at<double>(0, 0) * y * y +
                       coef.at<double>(1, 0) * y +
                       coef.at<double>(2, 0);

            // 이전 점이 있으면 선으로 연결
            if (y > 0)
            {
                double prev_x = coef.at<double>(0, 0) * (y - 5) * (y - 5) +
                                coef.at<double>(1, 0) * (y - 5) +
                                coef.at<double>(2, 0);
#ifdef DEBUG_IMAGE
                if (static_cast<int>(debug_draw_option_) & static_cast<int>(DebugDrawOption::REGRESSION_CURVE))
                {
                    cv::line(debug_img,
                             cv::Point(prev_x + IMG_WIDTH / 2, IMG_HEIGHT - (y - 5)),
                             cv::Point(x + IMG_WIDTH / 2, IMG_HEIGHT - y),
                             cv::Scalar(255, 0, 255), 2);
                }
#endif
            }
        }

// regression된 waypoints (큰 마젠타 점)
#ifdef DEBUG_IMAGE
        for (const auto &point : waypoints)
        {
            if (static_cast<int>(debug_draw_option_) & static_cast<int>(DebugDrawOption::WAY_POINT))
            {
                cv::Point2i img_point = vehicleToImage(point);
                cv::circle(debug_img, img_point, 5, cv::Scalar(255, 0, 255), -1);
            }
        }
#endif
    }
#ifdef DEBUG_IMAGE
    sensor_msgs::msg::Image::SharedPtr processed_debug_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_img).toImageMsg();

    // 역변환 수행
    cv::Mat original_image_with_debug;
    cv::warpPerspective(debug_img, original_image_with_debug, rear_M_.inv(), cv::Size(IMG_WIDTH, IMG_HEIGHT));

    sensor_msgs::msg::Image::SharedPtr original_image_with_debug_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", original_image_with_debug).toImageMsg();
    // if (frame_counter_rear % 3 == 0)
    // {
    std_msgs::msg::Int32 param_width_msg;
    param_width_msg.data = expected_parking_lane_width_;
    width_plot_pub_rear_->publish(param_width_msg); // 후방 주차

    rear_debug_pub_->publish(*processed_debug_msg); // Processed 된 화면
    rear_debug_on_real_pub_->publish(*original_image_with_debug_msg);
#endif
    // }

    return {waypoints, sum_window_confidences};
}

// 전방카메라 주차(사용 x)
std::vector<cv::Point2i> VisionNode::detectFrontParkingLanes(const cv::Mat &img)
{
    // 1. IPM 변환
    cv::Mat warped;
    cv::warpPerspective(img, warped, front_M_, cv::Size(IMG_WIDTH, IMG_HEIGHT));

    // 2. 이미지 처리
    cv::Mat gray, edges;
    cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    cv::Canny(gray, edges, 30, 90);

    // 3. 주차선 검출
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 30, 20, 10);

    // 4. 웨이포인트 생성 (주차 공간 중심으로)
    std::vector<cv::Point2i> waypoints;
    std::vector<int> x_coords;

    for (const auto &line : lines)
    {
        float angle = std::atan2(line[3] - line[1], line[2] - line[0]) * 180 / CV_PI;
        // 수직에 가까운 선만 선택 (주차선)
        if (std::abs(angle) > 75)
        {
            x_coords.push_back((line[0] + line[2]) / 2);
        }
    }

    if (x_coords.size() >= 2)
    {
        std::sort(x_coords.begin(), x_coords.end());
        int center_x = (x_coords.front() + x_coords.back()) / 2 - IMG_WIDTH / 2;

        // 4개의 웨이포인트 생성
        for (int i = 0; i < 4; i++)
        {
            int y = 360 - (i * 120);
            waypoints.push_back(cv::Point2i(center_x, y));
        }
    }

    return waypoints;
}
// 목표 포인트 찾기 (by y좌표)
cv::Point2i VisionNode::findNearestPoint(const std::vector<cv::Point2i> &points, int target_y)
{
    cv::Point2i nearest;
    int min_dist = std::numeric_limits<int>::max();

    for (const auto &point : points)
    {
        int dist = std::abs(point.y - target_y);
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest = point;
        }
    }
    return nearest;
}

void VisionNode::smoothPoints(std::vector<cv::Point2i> &current_points,
                              const std::vector<cv::Point2i> &prev_points,
                              const std::vector<float> &confidences)
{
    if (prev_points.empty())
        return;
    for (size_t i = 0; i < current_points.size(); i++)
    {
        cv::Point2i prev = findNearestPoint(prev_points, current_points[i].y);

        // 신뢰도가 높을수록 현재 프레임에 더 높은 가중치
        float smooth_factor = 0.3f * (1.0f - confidences[i]);
        // 0.3는 최대 smoothing 정도

        current_points[i].x = static_cast<int>(current_points[i].x * (1 - smooth_factor) + prev.x * smooth_factor);
    }
}
bool VisionNode::isValidLaneWidth(const cv::Point2i &left, const cv::Point2i &right, int lane_width)
{
    int width = abs(right.x - left.x);
    int offset = 400;                                                      // 차량의 폭보다 좁아지는 경우는 없음 Edit Param
    return (width > static_cast<int>(lane_width - 200) && width > offset); // 적절한 차선 폭 범위 Edit Param
}

cv::Scalar VisionNode::getColorByConfidence(float confidence)
{                                // 슬라이딩 윈도우 색상 하드 코딩
    const float epsilon = 1e-5f; // 작은 오차 허용 값
    if (confidence >= 1.0f - epsilon)
    {
        return cv::Scalar(0, 255, 0); // 초록색
    }
    else if (confidence >= 0.8f - epsilon)
    {
        return cv::Scalar(255, 0, 0); // 파란색
    }
    else if (confidence >= 0.7f - epsilon)
    {
        return cv::Scalar(0, 255, 255); // 노란색
    }
    else if (confidence >= 0.6f - epsilon)
    {
        return cv::Scalar(0, 0, 255); // 빨간색
    }
    else if (confidence >= 0.4f - epsilon)
    {
        return cv::Scalar(255, 0, 255); // 보라색
    }
    else
    {
        return cv::Scalar(0, 0, 0); // 검은색
    }
}

RegressionResult VisionNode::regressionWaypoints(const std::vector<cv::Point2i> &waypoints,
                                                 const std::vector<float> &confidences,
                                                 int num_points)
{
    if (waypoints.empty() || confidences.empty())
        return {waypoints, cv::Mat()};

    // 1. 신뢰도를 가중치로 사용하여 2차 다항식 피팅
    std::vector<double> x_values, y_values, weights;
    for (size_t i = 0; i < waypoints.size(); i++)
    {
        x_values.push_back(waypoints[i].y);
        y_values.push_back(waypoints[i].x);
        weights.push_back(confidences[i]);
    }

    // 2차 다항식: y = ax² + bx
    // 중요: 상수항을 0으로 고정하여 (0,0)을 반드시 지나가도록 함
    cv::Mat A(x_values.size(), 2, CV_64F);       // [x², x]
    cv::Mat b(x_values.size(), 1, CV_64F);       // y values
    cv::Mat W = cv::Mat::diag(cv::Mat(weights)); // 가중치 행렬

    // 행렬 A 구성
    for (size_t i = 0; i < x_values.size(); i++)
    {
        A.at<double>(i, 0) = x_values[i] * x_values[i];
        A.at<double>(i, 1) = x_values[i];
        b.at<double>(i, 0) = y_values[i];
    }

    // 가중치 적용된 최소제곱법으로 계수 구하기
    cv::Mat coef = (A.t() * W * A).inv() * A.t() * W * b;

    // 2. 새로운 웨이포인트 생성
    std::vector<cv::Point2i> new_waypoints;

    // 시작점을 (0,0)으로, 마지막 점을 이미지 상단으로
    new_waypoints.push_back(cv::Point2i(0, 0)); // 첫 번째 포인트는 항상 (0,0)

    int y_step = IMG_HEIGHT / num_points;
    for (int i = 1; i < num_points; i++)
    {
        int y = i * y_step;
        double x = coef.at<double>(0, 0) * y * y +
                   coef.at<double>(1, 0) * y;
        new_waypoints.push_back(cv::Point2i(static_cast<int>(x), y));
    }

    return {new_waypoints, coef};
}

// 웨이포인트 발행
void VisionNode::publishWaypoints(const std::vector<cv::Point2i> &points, UsingCamera is_front)
{

    std_msgs::msg::Int32MultiArray msg;
    for (int i = 0; i < static_cast<int>(points.size()); i++)
    {
        msg.data.push_back(points[i].x);
        msg.data.push_back(points[i].y);
    }
    if (is_front == UsingCamera::Front)
    {
        front_waypoint_pub_->publish(msg);
    }
    else if (is_front == UsingCamera::Rear)
    {
        rear_waypoint_pub_->publish(msg);
    }
}

void VisionNode::drawDebugImage(cv::Mat &img, const std::vector<cv::Point2i> &points, UsingCamera is_front)
{
    for (const auto &point : points)
    {
        cv::circle(img, cv::Point(point.x + IMG_WIDTH / 2, IMG_HEIGHT - point.y),
                   5, cv::Scalar(0, 255, 0), -1);
    }
    auto debug_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();

    if (is_front == UsingCamera::Front)
    {
        // front_debug_on_real_pub_->publish(*debug_msg);
    }
    else if (is_front == UsingCamera::Rear)
    {
        // rear_debug_on_real_pub_->publish(*debug_msg);
    }
}
// 이미지 좌표 -> 차량 좌표계 변환
cv::Point2i VisionNode::imageToVehicle(const cv::Point2i &img_point)
{
    // 이미지 좌표계: 좌상단이 원점, x축은 우측으로, y축은 아래로
    // 차량 좌표계: 이미지 중앙 하단이 원점, x축은 전방으로, y축은 좌측으로
    return cv::Point2i(
        img_point.x - IMG_WIDTH / 2, // 중앙 기준으로 변환
        IMG_HEIGHT - img_point.y     // y축 방향 반전
    );
}

// 차량 좌표계 -> 이미지 좌표 변환
cv::Point2i VisionNode::vehicleToImage(const cv::Point2i &vehicle_point)
{
    return cv::Point2i(
        vehicle_point.x + IMG_WIDTH / 2, // 이미지 중앙으로 이동
        IMG_HEIGHT - vehicle_point.y     // y축 방향 반전
    );
}
// 픽셀 -> 포인트 정규화
cv::Point2f VisionNode::pixelToNormalized(const cv::Point2f &pixel, UsingCamera is_front)
{
    // 카메라 내부 파라미터를 사용한 정규화
    const cv::Mat &camera_matrix = (is_front == UsingCamera::Front) ? wide_front_camera_matrix_ : wide_rear_camera_matrix_;

    return cv::Point2f(
        (pixel.x - camera_matrix.at<double>(0, 2)) / camera_matrix.at<double>(0, 0),
        (pixel.y - camera_matrix.at<double>(1, 2)) / camera_matrix.at<double>(1, 1));
}
