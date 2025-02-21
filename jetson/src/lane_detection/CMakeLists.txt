cmake_minimum_required(VERSION 3.5)
project(lane_detection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# CUDA 지원 활성화
enable_language(CUDA)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
set(OpenCV_DIR /usr/lib/aarch64-linux-gnu/cmake/opencv4)
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
set(OpenCV_CUDA_LIBRARIES 
    "/home/jetson/opencv-4.2.0/build/lib/libopencv_cudafeatures2d.so"
    "/home/jetson/opencv-4.2.0/build/lib/libopencv_cudaimgproc.so"
)
# find_package(OpenCV 4.2 REQUIRED COMPONENTS core imgproc highgui cudafeatures2d cudaimgproc)
find_package(OpenCV 4.2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(ament_index_cpp REQUIRED)
find_package(CUDA REQUIRED)

# CUDA 컴파일러 플래그 설정
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_53")  # Jetson Nano용

# CUDA 지원 컴파일 옵션
add_definitions(-DWITH_CUDA)

# 헤더 파일 디렉토리 포함
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  /usr/local/include/opencv4
)

# 실행 파일 생성
add_executable(lane_detection_node 
  src/main.cpp
  src/camera_node.cpp
  src/vision_node.cpp
  src/CentralGateway.cpp 
  src/LockBasedQueue.cpp 
  src/MySocketCAN.cpp 
  src/MyWebSocketSession.cpp 
  src/Protocol.cpp 
  src/SocketCAN.cpp 
  src/WebSocketServer.cpp
  src/OTADownloader.cpp
)

# 의존성 연결
ament_target_dependencies(lane_detection_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  Boost
  ament_index_cpp
)

target_link_libraries(lane_detection_node
  ${OpenCV_LIBRARIES}
  ${OpenCV_CUDA_LIBRARIES}
  ${CUDA_LIBRARIES}
  opencv_cudafeatures2d
  opencv_cudaimgproc
  opencv_core
  opencv_imgproc
  opencv_highgui
)

# 실행 파일 설치 규칙
install(TARGETS lane_detection_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(
  FILES 
  config/front_calibration.yaml
  config/rear_calibration.yaml
  DESTINATION config/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
