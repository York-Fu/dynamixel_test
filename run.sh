#! /bin/bash

sudo chmod 777 /dev/ttyUSB0
setserial /dev/ttyUSB0 low_latency
rosrun dynamixel_test dynamixel_test_node
