#include <rclcpp/rclcpp.hpp>
#include "lane_detection/camera_node.hpp"
#include "lane_detection/vision_node.hpp"
#include "lane_detection/CentralGateway.h"
#include "lane_detection/DevOption.hpp"
#include <iostream>
#include <signal.h>
#include <string>
#include <thread>
#include "lane_detection/OTADownloader.h"

std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_ptr = nullptr;

void signalHandler(int signum)
{
      std::cout << "\nCtrl+C 감지. 프로그램을 종료합니다..." << std::endl;

      if (executor_ptr)
      {
            executor_ptr->cancel();
      }

#ifndef DEBUG_CGW
      if (CGW)
      {
            CGW->Stop();
      }
#endif

      rclcpp::shutdown();
      exit(signum);
}

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
            signal(SIGINT, signalHandler);
            signal(SIGTERM, signalHandler);

            // MultiThreadedExecutor 생성
            executor_ptr = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

            // 노드 생성
            camera_node = std::make_shared<CameraNode>();
            vision_node = std::make_shared<VisionNode>();

            // 노드 추가
            executor_ptr->add_node(camera_node);
            executor_ptr->add_node(vision_node);
#ifndef DEBUG_CGW
            CGW->Start();
            std::thread download_thread(DownloadFirmware);

#endif
            // std::cout << "Enter눌러 종료" << std::endl;
            // std::cin.get();

            // 실행
            executor_ptr->spin();

#ifndef DEBUG_CGW
            if (downloadThread.joinable()) {
                  downloadThread.join();

#endif

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
