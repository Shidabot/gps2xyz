# gps2xyz

## Overview
 This package is used to convert GNSS data in rosbag to xyz and display trajectory on rviz. It is able to plot marker points of incoming gps positions. 
 
 Type: sensor_msgs/NavSatFix.
 
## Building

```
cd ~/catkin_ws/src
git clone https://github.com/Shidabot/gps2xyz.git
cd ..
catkin_make 
```

## Modify the launch file 

Modify the following values based on your rosbag 

```
origin_latitude_value
origin_longitude_value
origin_altitude_value
latitude_resolution
longitude_resolution
altitude_resolution
```
## Run

```
source ./devel/setup.bash
roslaunch gps_to_xyz gps_to_xyz.launch
rosbag play your.bag
```

## Result

What can I see? Manba out.

![gps](https://github.com/Shidabot/gps2xyz/assets/67732407/49d140b3-814a-4b5d-91fb-65fced8a2e38)
