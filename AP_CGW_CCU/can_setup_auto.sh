#!/bin/bash

echo "CAN 설정..."

MAX_PORT=30

for i in $(seq 0 $MAX_PORT)
do
	port="/dev/ttyACM$i"

	if [ -e "$port" ]; then
		echo "CAN setup 완료..."
		sudo slcand -o -c -s6 "$port" can0
	fi
done
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000
