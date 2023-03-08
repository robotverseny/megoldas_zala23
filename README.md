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

Egyszerre több viruális terminalt indít (`sudo apt install mc screen`, ha nem lenne telepítve), úgy mint: `roscore`, `turn_on_wheeltec_robot`, `lsn10_lidar`, `megoldas1.launch`:
```
rosrun megoldas_zala23 verseny_start.sh
rosrun megoldas_zala23 stop_all.sh
```

A `verseny_start` indít a `stop_all` peding mindent leállít értelemszerűen. További kezelés:

- list screen: `screen -ls`
- restore screen:  `screen -r roscore` / `screen -r turn_on_wheeltec_robot` /  `screen -r megoldas1 `
- detach: `Ctrl-a` + `Ctrl-d`


Külön:
```
roscore
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
roslaunch lsn10 lsn10.launch
roslaunch megoldas_zala23 megoldas1.launch
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

<img src="https://user-images.githubusercontent.com/11504709/160778891-0c06e338-405f-43c6-8aac-928af33c057e.png" width="50%" />

# Install 

```
sudo apt install mc screen
```

# Language
```
sudo dpkg-reconfigure locales
```
`reboot`

![](etc/locales.png)

# Gif

![](etc/left_right01.gif)
![](etc/trajectory01.gif)


