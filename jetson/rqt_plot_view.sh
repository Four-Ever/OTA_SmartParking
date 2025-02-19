#!/bin/bash

echo "Sourcing setup files..."
. ~/ros2_ws/install/setup.bash  # source 대신 . 사용

echo "Running rqt_plot..."
ros2 run rqt_plot rqt_plot