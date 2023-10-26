#!/bin/bash
# 脚本名称: compile.sh
# 脚本描述: 编译event-detector

# Source the ROS setup files (assuming you're using ROS Melodic, modify if using a different version)
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# Compile the event-detector package
catkin_make --source src/event-detector --build build/event-detector

# Inform the user of successful compilation
echo "Compilation of event-detector package complete!"

