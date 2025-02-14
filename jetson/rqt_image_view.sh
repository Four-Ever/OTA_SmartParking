#!/bin/bash

echo "Sourcing setup files..."
. ~/ros2_ws/install/setup.bash  # source 대신 . 사용

echo "Running rqt_image_view..."
ros2 run rqt_image_view rqt_image_view