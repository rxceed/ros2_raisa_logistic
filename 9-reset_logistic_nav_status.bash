#!/bin/bash
source /opt/ros/jazzy/setup.bash
. install/setup.bash 
ros2 service call logistic_nav_status custom_interface/srv/LogisticNavStatus "{action: 1, state: 0}"