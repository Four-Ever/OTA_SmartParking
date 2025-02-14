// src/camera_node.cpp
#include "lane_detection/camera_node.hpp"
#define DEBUG

std::shared_ptr<CameraNode> camera_node = nullptr;

CameraNode::CameraNode(const rclcpp::NodeOptions &options)
    : Node("camera_node", options)
{
    // 파라미터 선언    
    this->declare_parameter("front_camera_id", 0);
    this->declare_parameter("rear_camera_id", 1);
    this->declare_parameter("image_width", 640);
    this->declare_parameter("image_height", 480);
    this->declare_parameter("fps", 20.0);
    this->declare_parameter("use_calibration", false);

    // 파라미터 가져오기
    front_cam_id_ = this->get_parameter("front_camera_id").as_int();
    rear_cam_id_ = this->get_parameter("rear_camera_id").as_int();
    img_width_ = this->get_parameter("image_width").as_int();
    img_height_ = this->get_parameter("image_height").as_int();
    double fps = this->get_parameter("fps").as_double();
    use_calibration_ = this->get_parameter("use_calibration").as_bool();

    // 캘리브레이션 파라미터 가져오기
    if (use_calibration_)
    {
        loadCalibrationParams();
    }

    // 카메라 초기화
    if (!front_cap_.open(front_cam_id_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open front camera");
        return;
    }
#ifndef DEBUG
    if (!rear_cap_.open(rear_cam_id_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open rear camera");
        return;
    }
#endif

    // 카메라 설정
    front_cap_.set(cv::CAP_PROP_FRAME_WIDTH, img_width_);
    front_cap_.set(cv::CAP_PROP_FRAME_HEIGHT, img_height_);
#ifndef DEBUG
    rear_cap_.set(cv::CAP_PROP_FRAME_WIDTH, img_width_);
    rear_cap_.set(cv::CAP_PROP_FRAME_HEIGHT, img_height_);
#endif

    // Publisher 생성
    front_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "front_camera/image", 10);
    rear_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "rear_camera/image", 10);

    // 타이머 생성
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / fps)),
        std::bind(&CameraNode::publishImages, this));
}

CameraNode::~CameraNode()
{
    front_cap_.release();
    rear_cap_.release();
}

void CameraNode::publishImages() // 주기적으로 카메라 이미지 발행
{
    cv::Mat front_frame, rear_frame;

    if (front_cap_.read(front_frame))
    {
        // 카메라 캘리브레이션 (왜곡 보정 적용)
        cv::Mat processed_frame;
        if (use_calibration_)
        {
            cv::undistort(front_frame, processed_frame,
                          front_camera_matrix_, front_dist_coeffs_);
        }
        else
        {
            processed_frame = front_frame;
        }

        sensor_msgs::msg::Image::SharedPtr front_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", processed_frame).toImageMsg();
        front_pub_->publish(*front_msg);
    }

    if (rear_cap_.read(rear_frame))
    {
        // 180도 회전
        cv::Mat processed_frame;
        if (use_calibration_)
        {
            cv::undistort(rear_frame, processed_frame,
                          rear_camera_matrix_, rear_dist_coeffs_);
        }
        else
        {
            processed_frame = rear_frame;
        }

        cv::Mat rotated_rear_frame;
        //    cv::rotate(processed_frame, rotated_rear_frame, cv::ROTATE_180);

        sensor_msgs::msg::Image::SharedPtr rear_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rotated_rear_frame).toImageMsg();
        rear_pub_->publish(*rear_msg);
    }
}

void CameraNode::loadCalibrationParams()
{
    try
    {
        // 전방 카메라 파라미터
        cv::FileStorage front_fs("front_calibration.yaml", cv::FileStorage::READ);
        if (!front_fs.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open front camera calibration file");
            return;
        }
        front_fs["camera_matrix"] >> front_camera_matrix_;
        front_fs["distortion_coefficients"] >> front_dist_coeffs_;
        front_fs.release();

        // 후방 카메라 파라미터
        cv::FileStorage rear_fs("rear_calibration.yaml", cv::FileStorage::READ);
        if (!rear_fs.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open rear camera calibration file");
            return;
        }
        rear_fs["camera_matrix"] >> rear_camera_matrix_;
        rear_fs["distortion_coefficients"] >> rear_dist_coeffs_;
        rear_fs.release();

        RCLCPP_INFO(this->get_logger(), "Camera calibration parameters loaded successfully");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error loading calibration files: %s", e.what());
        use_calibration_ = false;
    }
}