#!/bin/bash

echo "Building lane_detection package..."
colcon build --packages-select lane_detection

# source 설정이 되어있지 않은 경우를 대비
echo "Sourcing setup files..."
source install/setup.bash

echo "Running lane_detection_node..."
ros2 run lane_detection lane_detection_node