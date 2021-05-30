```
cd ~
mkdir ~/<ws_name>/src && cd ~/<ws_name>/src
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

### Branch: YOLOv4
* Install [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
* Install [darknet_ros](https://github.com/SamKaiYang/darknet_ros.git)

### Branch: Yolact
* Install [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
* Install [yolact](https://github.com/dbolya/yolact)
* Install [yolact_ros](https://github.com/Eruvae/yolact_ros)
* Install [yolact_ros_msgs](https://github.com/Eruvae/yolact_ros_msgs)

### Branch: ArUco + YOLOv4
* Install [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
* Install [darknet_ros](https://github.com/SamKaiYang/darknet_ros.git)
* Install ArUco `$ pip3 install opencv-contrib-python`

### Branch: ArUco + Yolact
* Install [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
* Install [yolact](https://github.com/dbolya/yolact)
* Install [yolact_ros](https://github.com/Eruvae/yolact_ros)
* Install [yolact_ros_msgs](https://github.com/Eruvae/yolact_ros_msgs)
* Install ArUco `$ pip3 install opencv-contrib-python`

## Software environment
* Ubuntu 18.04
* ROS Melodic
* Python 3.6.9
* opencv 4.5.1 cv2 (?)
* cv_bridge (python3 待測試) (?)
* Install pcl (?)
* Install cv_bridge(?)
