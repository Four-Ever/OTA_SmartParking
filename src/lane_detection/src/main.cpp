#include <rclcpp/rclcpp.hpp>
#include "lane_detection/camera_node.hpp"
#include "lane_detection/vision_node.hpp"

int main(int argc, char * argv[])
{
   rclcpp::init(argc, argv);
   
   // MultiThreadedExecutor 생성
   rclcpp::executors::MultiThreadedExecutor executor;
   
   // 노드 생성
   auto camera_node = std::make_shared<CameraNode>();
   auto vision_node = std::make_shared<VisionNode>();
   
   // 노드 추가
   executor.add_node(camera_node);
   executor.add_node(vision_node);
   
   // 실행
   executor.spin();
   
   rclcpp::shutdown();
   return 0;
}