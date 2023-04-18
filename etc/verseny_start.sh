#!/bin/bash
source /opt/ros/melodic/setup.bash
source ~/wheeltec_robot/devel/setup.bash
source ~/megoldas_ws/devel/setup.bash

export ROS_IP=192.168.0.100
export ROS_MASTER_URI=http://192.168.0.100:11311

echo "IP 192.168.0.100"


echo "[INFO] Starting roscore"
screen -dmS roscore bash -c 'roscore'
sleep 4
echo "[INFO] Starting turn_on_wheeltec_robot"
screen -dmS turn_on_wheeltec_robot bash -c 'roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch'
echo "[INFO] Starting lsn10 LIDAR"
screen -dmS lsn10 bash -c 'roslaunch lsn10 lsn10.launch'
sleep 4
echo "[INFO] Starting mozgas"
screen -dmS megoldas1 bash -c 'roslaunch megoldas_zala23 megoldas2.launch'
echo "[INFO] Starting vizu"
screen -dmS vizu bash -c 'rosrun megoldas_zala23 path_and_steering'
sleep 120
echo "[INFO] STOP"
rosrun megoldas_zala23 stop_all.sh
