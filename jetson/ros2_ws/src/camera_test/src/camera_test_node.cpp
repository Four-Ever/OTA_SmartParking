// src/camera_test_node.cpp
    #include "camera_test/camera_test_node.hpp"

    CameraTestNode::CameraTestNode() : Node("camera_test_node") {
        // 퍼블리셔 초기화
        front_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/front_camera/image", 10);
        rear_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/rear_camera/image", 10);

        // 카메라 초기화
        if (!initialize_cameras()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize cameras!");
            return;
        }

        // 타이머 설정 (30fps = 33.3ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&CameraTestNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Camera test node has been initialized");
    }

    CameraTestNode::~CameraTestNode() {
        // 카메라 릴리스
        if (front_cam_.isOpened()) front_cam_.release();
        if (rear_cam_.isOpened()) rear_cam_.release();
    }

    bool CameraTestNode::initialize_cameras() {
        // 전방 카메라 초기화
        front_cam_.open(FRONT_CAM_ID);
        if (!front_cam_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open front camera!");
            return false;
        }
        front_cam_.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
        front_cam_.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
        front_cam_.set(cv::CAP_PROP_FPS, 30);
        front_cam_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

        // 후방 카메라 초기화
        rear_cam_.open(REAR_CAM_ID);
        if (!rear_cam_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open rear camera!");
            return false;
        }
        rear_cam_.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
        rear_cam_.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
        rear_cam_.set(cv::CAP_PROP_FPS, 30);
        rear_cam_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));

        return true;
    }

    void CameraTestNode::timer_callback() {
        cv::Mat front_frame, rear_frame;
        
        // 전방 카메라 프레임 읽기
        if (front_cam_.read(front_frame)) {
            // OpenCV 이미지를 ROS 메시지로 변환
            sensor_msgs::msg::Image::SharedPtr front_msg = 
                cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", front_frame).toImageMsg();
            front_pub_->publish(*front_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to capture front camera frame");
        }

        // 후방 카메라 프레임 읽기
        if (rear_cam_.read(rear_frame)) {
            // OpenCV 이미지를 ROS 메시지로 변환
            sensor_msgs::msg::Image::SharedPtr rear_msg = 
                cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rear_frame).toImageMsg();
            rear_pub_->publish(*rear_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to capture rear camera frame");
        }
    }
