# Home

# `megoldas_zala23` ROS package
🤖 Autonóm robotverseny zalaegerszegi forduló megoldás template 2023

# Robot
Wheeltec / Roboworks Rosbot mini Ackermann robot
![main](etc/wheeltec_roboworks_ack01.png)

- Nvidia Jetson Nano
- Orbbec Depth Camera
- LSN10 LIDAR


# Használat
```
ssh wheeltec@192.168.0.104
ssh wheeltec@192.168.0.100
```

default password: `dongguan` 

`nano ~/.bashrc` --> IP áítírása

`source ~/.bashrc`

## ROS használat
```
mkdir -p ~/megoldas_ws/src
cd ~/megoldas_ws/
catkin init
cd ~/megoldas_ws/src/
git clone https://github.com/robotverseny/megoldas_zala23
cd ~/megoldas_ws/
catkin build megoldas_zala23
echo "source /home/wheeltec/megoldas_ws/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc
```

```
rosrun rqt_robot_steering rqt_robot_steering
```


# Workspace
```
~/wheeltec_robot/src
~/catkin_workspace/src
~/megoldas_ws/src/
```

# Launch

```
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
roslaunch lsn10 lsn10.launch
```


# Topic

```
rostopic hz /scan
rostopic echo /scan -n1
rostopic type /scan
```

```
sensor_msgs/LaserScan
```

# nmtui
```
nmtui
```

# Install 

```
sudo apt install mc screen
```

