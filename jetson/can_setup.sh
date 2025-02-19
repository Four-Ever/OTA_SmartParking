#!/bin/bash

echo "CAN 설정..."

sudo slcand -o -c -s6 /dev/ttyACM0 can0
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000
