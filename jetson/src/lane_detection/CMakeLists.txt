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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(ament_index_cpp REQUIRED)

# 헤더 파일 디렉토리 포함
include_directories(include)

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
