```
git clone https://github.com/uptopia/wrs_stock_pose.git
git checkout <branch-name>
cd ..
catkin_make
```
# wrs_stock_pose

## Dependencies
### Branch: ArUco
* Install [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
* Install ArUco `$ pip3 install opencv-contrib-python`

## Software environment
* Ubuntu 18.04
* ROS Melodic
* Python 3.6.9
* opencv 4.5.1 cv2 (?)
* opencv-contrib-python-4.5.2.52
* cv_bridge (python3 待測試) (?)
* Install pcl (?)
* Install cv_bridge(?)

## Detect ArUco Markers

```
<Terminal 1>
    $roscore

<Terminal 2> Realsense D435i
    $cd <realsense-ros_ws>

    [ROS Topic: /camera/depth_registered/points]
    $roslaunch realsense2_camera rs_rgbd.launch
 
<Terminal 3> Detect ArUco Markers
    $cd <ws_name>
    $. devel/setup.bash
    $rosrun rosrun detect_aruco detect_aruco_ros.py
```

## Detect ArUco Markers' Cloud Pose

### Usage 1: Combine Realsense, Detect_Aruco, Rviz into launch file
```
<Terminal 1> Detect ArUco Markers
    $cd <ws_name>
    $. devel/setup.bash
    $roslaunch detect_aruco detect_aruco.launch

<Terminal 2> Detect ArUco Marker Cloud Pose
    $cd <ws_name>
    $. devel/setup.bash
    $rosrun detect_aruco_pose detect_aruco_pose
```

### Usage 2
```
<Terminal 1>
    $roscore

<Terminal 2> Realsense D435i
    $cd <realsense-ros_ws>

    [ROS Topic: /camera/depth_registered/points]
    $roslaunch realsense2_camera rs_rgbd.launch
 
<Terminal 3> Detect ArUco Marker
    $cd <ws_name>
    $. devel/setup.bash
    $rosrun rosrun detect_aruco detect_aruco_ros.py

<Terminal 4> Detect ArUco Marker Cloud Pose
    $cd <ws_name>
    $. devel/setup.bash
    $rosrun detect_aruco_pose detect_aruco_pose

<Terminal 5> Rviz
    $rviz
```