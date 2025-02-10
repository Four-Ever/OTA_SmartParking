// src/vision_node.cpp
#include "lane_detection/vision_node.hpp"

VisionNode::VisionNode(const rclcpp::NodeOptions &options)
    : Node("vision_node", options)
{
    // 파라미터 선언
    this->declare_parameter("mode", "driving");
    //    this->declare_parameter("mode", "parking");

    // Subscriber 생성
    front_cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "front_camera/image", 10,
        std::bind(&VisionNode::processFrontImage, this, std::placeholders::_1));

    // Subscriber 생성 시
    rear_cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "rear_camera/image", 10,
        std::bind(&VisionNode::processRearImage, this, std::placeholders::_1));

    // Publisher 생성
    front_debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("front_camera/debug", 10);
    rear_debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("rear_camera/debug", 10);
    front_waypoint_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("front_camera/waypoints", 10);
    rear_waypoint_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("rear_camera/waypoints", 10);

    // 디버그 퍼블리셔 생성
    debug_warped_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug/warped", 10);
    debug_edges_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug/edges", 10);
    debug_lines_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug/lines", 10);

    // IPM 변환 행렬 초기화
    std::vector<cv::Point2f> src_points = {
        cv::Point2f(0, 480), cv::Point2f(640, 480),
        cv::Point2f(100, 240), cv::Point2f(540, 240)};
    std::vector<cv::Point2f> dst_points = {
        cv::Point2f(0, 480), cv::Point2f(640, 480),
        cv::Point2f(0, 0), cv::Point2f(640, 0)};
    front_M_ = cv::getPerspectiveTransform(src_points, dst_points);
    rear_M_ = front_M_.clone();

    // 모드 설정
    std::string mode_str = this->get_parameter("mode").as_string();
    mode_ = (mode_str == "driving") ? Mode::DRIVING : Mode::PARKING;

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
                    mode_ = (param.as_string() == "driving") ? Mode::DRIVING : Mode::PARKING;
                    RCLCPP_INFO(this->get_logger(), "Mode changed to: %s", param.as_string().c_str());
                }
            }
            return result;
        });
}

void VisionNode::processFrontImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    std::vector<cv::Point2i> waypoints;
    if (mode_ == Mode::DRIVING)
    {
        waypoints = detectDrivingLanes(cv_ptr->image);
    }
    else
    {
        waypoints = detectFrontParkingLanes(cv_ptr->image);
    }

    cv::Mat debug_img = cv_ptr->image.clone();
    drawDebugImage(debug_img, waypoints, UsingCamera::Front);
    publishWaypoints(waypoints, UsingCamera::Front);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // RCLCPP_INFO(this->get_logger(), "processFrontImage Processing Time: %ld ms", duration.count());
}

void VisionNode::processRearImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (mode_ == Mode::PARKING)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        auto waypoints = detectRearParkingLanes(cv_ptr->image);

        cv::Mat debug_img = cv_ptr->image.clone();
        drawDebugImage(debug_img, waypoints, UsingCamera::Rear);
        publishWaypoints(waypoints, UsingCamera::Rear);
    }
}

void VisionNode::smoothPoints(std::vector<cv::Point2i> &current_points,
                              const std::vector<cv::Point2i> &prev_points)
{
    if (prev_points.empty())
        return;
    const float smooth_factor = 0.3; // 값이 클수록 이전 프레임의 영향이 커짐

    for (size_t i = 0; i < current_points.size(); i++)
    {
        cv::Point2i prev = findNearestPoint(prev_points, current_points[i].y);
        current_points[i].x = static_cast<int>(
            current_points[i].x * (1 - smooth_factor) + prev.x * smooth_factor);
    }
}

std::vector<cv::Point2i> VisionNode::detectDrivingLanes(const cv::Mat &img)
{
    static int frame_counter = 0;
    frame_counter++;

    // 1. 이미지 전처리
    cv::Mat warped;
    cv::warpPerspective(img, warped, front_M_, cv::Size(640, 480));

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
    cv::bitwise_and(gray, gray, masked_gray, white_mask);

    // 가우시안 블러로 노이즈 제거
    cv::GaussianBlur(masked_gray, masked_gray, cv::Size(5, 5), 0);

    // 이진화
    cv::Mat binary;
    cv::threshold(masked_gray, binary, 160, 255, cv::THRESH_BINARY);
    // 모폴로지 연산
    if (!binary.empty())
    {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // 작은 커널 사용
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    }
    // Canny 엣지 검출
    cv::Mat edges;
    cv::Canny(masked_gray, edges, 50, 150);

    // 디버그 메시지
    sensor_msgs::msg::Image::SharedPtr warped_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", warped).toImageMsg();

    cv::Mat edges_color;
    cv::cvtColor(edges, edges_color, cv::COLOR_GRAY2BGR);
    sensor_msgs::msg::Image::SharedPtr edges_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", edges_color).toImageMsg();

    cv::Mat debug_img = warped.clone();
    float confidence = 1.0;

    // 2. ROI 기반 차선 검출
    const int windows = 8;
    int window_height = warped.rows / windows;
    int window_width = 100;
    size_t minpix = 50;                  // 최소 픽셀 수 Param Edit
    const int expected_lane_width = 300; // 예상 차선 폭 (픽셀 단위) Param Edit

    std::vector<cv::Point2i> left_points, right_points;

    // 히스토그램으로 초기 차선 위치 찾기
    cv::Mat bottom = edges(cv::Rect(0, edges.rows - window_height,
                                    edges.cols, window_height));
    std::vector<int> histogram(edges.cols, 0);
    for (int x = 0; x < edges.cols; x++)
    {
        histogram[x] = cv::sum(bottom.col(x))[0];
    }

    int midpoint = edges.cols / 2;
    int leftx_current = std::max_element(histogram.begin(),
                                         histogram.begin() + midpoint) -
                        histogram.begin();
    int rightx_current = std::max_element(histogram.begin() + midpoint,
                                          histogram.end()) -
                         histogram.begin();

    for (int window = 0; window < windows; window++)
    {
        int win_y_low = edges.rows - (window + 1) * window_height;
        int win_y_high = edges.rows - window * window_height;
        int center_y = (win_y_low + win_y_high) / 2;

        // 왼쪽 윈도우
        int win_xleft_low = leftx_current - window_width / 2;
        int win_xleft_high = leftx_current + window_width / 2;

        // 오른쪽 윈도우
        int win_xright_low = rightx_current - window_width / 2;
        int win_xright_high = rightx_current + window_width / 2;

        // 디버그용 윈도우 표시
        cv::rectangle(debug_img,
                      cv::Point(win_xleft_low, win_y_low),
                      cv::Point(win_xleft_high, win_y_high),
                      cv::Scalar(0, 255, 0), 2);
        cv::rectangle(debug_img,
                      cv::Point(win_xright_low, win_y_low),
                      cv::Point(win_xright_high, win_y_high),
                      cv::Scalar(0, 255, 0), 2);

        // ROI 설정 및 범위 체크
        cv::Rect left_roi(win_xleft_low, win_y_low, window_width, window_height);
        cv::Rect right_roi(win_xright_low, win_y_low, window_width, window_height);

        left_roi &= cv::Rect(0, 0, edges.cols, edges.rows);
        right_roi &= cv::Rect(0, 0, edges.cols, edges.rows);

        bool found_left = false;
        bool found_right = false;
        cv::Point2i left_point, right_point;

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
                leftx_current = win_xleft_low + mean_x / left_nonzero.size();
                left_point = cv::Point2i(leftx_current - 320, 480 - center_y);
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
                rightx_current = win_xright_low + mean_x / right_nonzero.size();
                right_point = cv::Point2i(rightx_current - 320, 480 - center_y);
                found_right = true;
            }
        }
        // 차선 소실 대응
        if (!found_left && found_right)
        {
            // 오른쪽 차선만 발견된 경우
            left_point = cv::Point2i(right_point.x - expected_lane_width, right_point.y);
            found_left = true;
            confidence *= 0.8;
        }
        else if (found_left && !found_right)
        {
            // 왼쪽 차선만 발견된 경우
            right_point = cv::Point2i(left_point.x + expected_lane_width, left_point.y);
            found_right = true;
            confidence *= 0.8;
        }
        else if (!found_left && !found_right)
        {
            // 둘 다 발견되지 않은 경우
            if (!first_frame_ && !prev_left_points_.empty() && !prev_right_points_.empty())
            {
                // 이전 프레임의 해당 높이의 포인트 찾기
                left_point = findNearestPoint(prev_left_points_, 480 - center_y);
                right_point = findNearestPoint(prev_right_points_, 480 - center_y);
                found_left = found_right = true;
                confidence *= 0.6;
            }
            else
            {
                // 기본값 사용
                left_point = cv::Point2i(-expected_lane_width / 2, 480 - center_y);
                right_point = cv::Point2i(expected_lane_width / 2, 480 - center_y);
                confidence *= 0.4;
            }
        }

        if (found_left)
            left_points.push_back(left_point);
        if (found_right)
            right_points.push_back(right_point);
    }

    // 이전 프레임 정보와 혼합 (프레임별로 급격한 변화 없애기)
    if (!first_frame_)
    {
        smoothPoints(left_points, prev_left_points_);
        smoothPoints(right_points, prev_right_points_);
    }

    // 웨이포인트 생성
    std::vector<cv::Point2i> waypoints;
    waypoints.reserve(windows);

    if (!left_points.empty() && !right_points.empty())
    {
        for (int i = 0; i < windows; i++)
        {
            int target_y = 480 - window_height / 2 - i * window_height;
            cv::Point2i left = findNearestPoint(left_points, target_y);
            cv::Point2i right = findNearestPoint(right_points, target_y);

            if (!isValidLaneWidth(left, right))
            {
                confidence *= 0.8;
                // 차선 폭이 잘못된 경우 예상 폭 사용
                int center_x = (left.x + right.x) / 2;
                left = cv::Point2i(center_x - expected_lane_width / 2, target_y);
                right = cv::Point2i(center_x + expected_lane_width / 2, target_y);
            }

            waypoints.push_back(cv::Point2i((left.x + right.x) / 2, target_y));
        }
    }

    // 현재 프레임 정보 저장
    prev_left_points_ = left_points;
    prev_right_points_ = right_points;
    prev_confidence_ = confidence;
    first_frame_ = false;

    // 디버그 발행
    sensor_msgs::msg::Image::SharedPtr lines_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_img).toImageMsg();

    if (frame_counter % 3 == 0)
    {
        debug_warped_pub_->publish(*warped_msg);
        debug_edges_pub_->publish(*edges_msg);
        debug_lines_pub_->publish(*lines_msg);
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
std::vector<cv::Point2i> VisionNode::detectFrontParkingLanes(const cv::Mat &img)
{
    // 1. IPM 변환
    cv::Mat warped;
    cv::warpPerspective(img, warped, front_M_, cv::Size(640, 480));

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
        int center_x = (x_coords.front() + x_coords.back()) / 2 - 320;

        // 4개의 웨이포인트 생성
        for (int i = 0; i < 4; i++)
        {
            int y = 360 - (i * 120);
            waypoints.push_back(cv::Point2i(center_x, y));
        }
    }

    return waypoints;
}

// 후방 카메라 주차 모드 차선 검출
std::vector<cv::Point2i> VisionNode::detectRearParkingLanes(const cv::Mat &img)
{
    // 1. IPM 변환
    cv::Mat warped;
    cv::warpPerspective(img, warped, rear_M_, cv::Size(640, 480));

    // 2. 이미지 처리
    cv::Mat gray, edges;
    cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    cv::Canny(gray, edges, 50, 150);

    // 3. 주차선 검출
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 30, 20, 10);

    // 4. 웨이포인트 생성
    std::vector<cv::Point2i> waypoints;
    if (!lines.empty())
    {
        // 차량 후방부터 시작하는 4개의 웨이포인트
        for (int i = 0; i < 4; i++)
        {
            int y = 120 + (i * 120);                // 120, 240, 360, 480
            waypoints.push_back(cv::Point2i(0, y)); // 중앙 정렬
        }
    }

    return waypoints;
}

void VisionNode::publishWaypoints(const std::vector<cv::Point2i> &points, UsingCamera is_front)
{
    std_msgs::msg::Int32MultiArray msg;
    int skip_num = 2; // window num / 4
    for (int i = 0; i < static_cast<int>(points.size()); i += skip_num)
    {
        if (i < static_cast<int>(points.size()))
        {
            msg.data.push_back(points[i].x);
            msg.data.push_back(points[i].y);
        }
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
        cv::circle(img, cv::Point(point.x + 320, 480 - point.y),
                   5, cv::Scalar(0, 255, 0), -1);
    }
    auto debug_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();

    if (is_front == UsingCamera::Front)
    {
        front_debug_pub_->publish(*debug_msg);
    }
    else if (is_front == UsingCamera::Rear)
    {
        rear_debug_pub_->publish(*debug_msg);
    }
}