# hikvision_ros
simple preview bridge node of  hikvision IP camera for ROS 



## dependence (within `/hikvision_ros_opencv3`)

* cv_bridge
* image_transport
* OpenCV3

## dependence (choice within `/hikvision_ros` and `hikvision_ros_arm`)

* image_transport

## usage



***instant run***

```sh
mkdir -p catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Kyronxu/hikvision_ros.git
cd ..
catkin_make
```

Publish ` sensor_msgs::Image`

```sh
roscore
source devel/setup.bash
roslaunch hikvision_ros hik.launch
roslaunch hikvision_ros start_four_hik.launch
roslaunch hikvision_ros start_front_hik.launch
roslaunch hikvision_ros start_left_hik.launch
roslaunch hikvision_ros start_back_hik.launch
roslaunch hikvision_ros start_right_hik.launch
```



***parameters***

You can specify some camera and steam parameters by `hik.launch`

```xml
<arg name="ip_addr" default="192.168.5.100"/>
<arg name="user_name" default="admin"/>
<arg name="password" default="admin"/>
<arg name="port" default="8000"/>
<arg name="channel" default="1"/>
```

Or in command line

```sh
roslaunch hikvision_ros hik.launch ip_addr:=192.168.5.100 password:=123456
```



***support for camera_calibration***

you can use [camera_calibration](http://wiki.ros.org/camera_calibration/)  to calibrate hikvision camera, **hik_ros**  provides *set_camera_info* sevice for receiving and storing camera's calibration parameters. 

***example***

```sh
# open camera
roslaunch hikvision_ros hik.launch

# see topic name
rostopic list

# [output]
# ➜  ~ rostopic list
# /hik_cam_node/camera_info
# /hik_cam_node/hik_camera
# ...

# calibrate
rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.03 image:=/hik_cam_node/hik_camera  camera:=/hik_cam_node/hik_camera
```

then begin calibration. After calibration you straightly press **commit** button,  **hik_ros** has the ability to save the calibration parameter to `camera_info_url`, which is set in launch file OR use default path (  `~/.ros/camera_info` )   

```sh
# [output]
#[ INFO] [1551582484.454024618]: New camera info received
#[ INFO] [1551582484.454296067]: [ hik_camera ] Write camera_info to ~/.ros/camera_info/hik_camera.yaml success.
```

## Add frame delay detection by zhaoyu
```sh
<arg name="frame_rate" default="10"/> 
<arg name="if_shutdown" default="true"/>
```
启动节点前，请先获取相机的帧率，并修改”frame_rate“。
"if_shutdown"为true时，检测到相机延时会关闭节点。
