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

# Workspace
```
~/wheeltec_robot/src
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
