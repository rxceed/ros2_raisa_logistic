#!/bin/bash
source /opt/ros/jazzy/setup.bash
. install/setup.bash 
ros2 bag record --topics uwb_pose2d reeman_pose2d dual_leg action_forecast logistic_nav_status reeman_speed