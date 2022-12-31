# `megoldas_zala23` ROS package
🤖 Autonóm robotverseny zalaegerszegi forduló megoldás template 2023

# Használat
```
ssh wheeltec@192.168.0.104
ssh wheeltec@192.168.0.100
```

default password: `dongguan` 

`nano ~/.bashrc` --> IP áítírása

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
