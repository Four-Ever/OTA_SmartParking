#include <rclcpp/rclcpp.hpp>
#include "lane_detection/camera_node.hpp"
#include "lane_detection/vision_node.hpp"
#include "lane_detection/CentralGateway.h"
#include "lane_detection/DevOption.hpp"
#include <iostream>

int main(int argc, char *argv[])
{
   try
   {
#ifndef DEBUG_CGW
      if (!CGW->Init())
      {
         return 1;
      }
#endif
      rclcpp::init(argc, argv);

      // MultiThreadedExecutor 생성
      rclcpp::executors::MultiThreadedExecutor executor;

      // 노드 생성
      camera_node = std::make_shared<CameraNode>();
      vision_node = std::make_shared<VisionNode>();

      // 노드 추가
      executor.add_node(camera_node);
      executor.add_node(vision_node);
#ifndef DEBUG_CGW
      CGW->Start();
#endif
      // std::cout << "Enter눌러 종료" << std::endl;
      // std::cin.get();

      // 실행
      executor.spin();

// end
#ifndef DEBUG_CGW
      CGW->Stop();
#endif
      rclcpp::shutdown();
   }
   catch (const std::exception &e)
   {
      std::cerr << "Error: " << e.what() << std::endl;
      return 1;
   }

   return 0;
}
