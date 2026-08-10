#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/roberto-grajales/bota_ws/install/setup.bash
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/home/roberto-grajales/bota_ws/install/ethercat_publisher/lib:$LD_LIBRARY_PATH
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/roberto-grajales/udp_transport.xml
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec /home/roberto-grajales/bota_ws/build/ethercat_publisher/ethercat_publisher_node "$@"
