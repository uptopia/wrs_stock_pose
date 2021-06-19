#!/usr/bin/env python
# # -*- coding: utf-8 -*- #
# roscore

# cd ~/realsense_ws
# . devel/setup.bash
# roslaunch realsense2_camera rs_camera.launch
# roslaunch realsense2_camera rs_rgbd.launch filters:=pointcloud

# rostopic list

# cd ~/Documents/learn_ros/src/beginner_tutorials/src
# python aruco_ros.py

# written by Shang-Wen, Wong
# 2021.3.2
# https://github.com/njanirudh/Aruco_Tracker


# """
# Framework   : OpenCV Aruco
# Description : Calibration of camera and using that for finding pose of multiple markers
# Status      : Working
# References  :
#     1) https://docs.opencv.org/3.4.0/d5/dae/tutorial_aruco_detection.html
#     2) https://docs.opencv.org/3.4.3/dc/dbb/tutorial_py_calibration.html
#     3) https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html
# """

import rospy
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from aruco_msgs.msg import ArucoMarkerArray

import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError

import numpy as np


# def imageDepthCallback(data):
#     bridge = CvBridge()
#     try:
#         cv_image = bridge.imgmsg_to_cv2(data, data.encoding)
#         cv_image8 = cv2.convertScaleAbs(cv_image)
#         # pix = (data.width/2, data.height/2)
#         cv2.imshow('Depth', cv_image8)
#         cv2.waitKey(1)
#         # sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % (self.topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
#         # sys.stdout.flush()
#     except CvBridgeError as e:
#         print(e)
#         return

# def imageRGBCallback(data):
#     bridge = CvBridge()
#     try:
#         cv_image = bridge.imgmsg_to_cv2(data, data.encoding)
#         cv2.imshow('Color', cv_image)
#         cv2.waitKey(1)

#     except CvBridgeError as e:
#         print(e)
#         return


# 相機內部參數,畸變係數
# (1) 查詢Realsense相機參數資訊：$rs-enumerate-devices -c
#     Intrinsic of "Color" / 640x480 / {YUYV/RGB8/BGR8/RGBA8/BGRA8/Y16}
#     Width:      	640
#     Height:     	480
#     PPX:        	326.004974365234
#     PPY:        	237.414825439453
#     Fx:         	615.178771972656
#     Fy:         	615.507995605469
#     Distortion: 	Inverse Brown Conrady
#     Coeffs:     	0  	0  	0  	0  	0
#     FOV (deg):  	54.96 x 42.6

# (2) Realsense d453i: rostopic echo /camera/color/camera_info
#     D: [0.0, 0.0, 0.0, 0.0, 0.0]
#     K: [615.1787719726562, 0.0, 326.0049743652344, 0.0, 615.5079956054688, 237.41482543945312, 0.0, 0.0, 1.0]
#     R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
#     P: [615.1787719726562, 0.0, 326.0049743652344, 0.0, 0.0, 615.5079956054688, 237.41482543945312, 0.0, 0.0, 0.0, 1.0, 0.0]

# c_x = 326.004974365234 #PPX
# c_y = 237.414825439453 #PPY
# f_x = 615.178771972656 #FX
# f_y = 615.507995605469 #FY

# k_1 = 0.0
# k_2 = 0.0
# p_1 = 0.0
# p_2 = 0.0
# k_3 = 0.0

# intrinsic_matrix = np.array([[f_x, 0.0, c_x],
#                     [0.0, f_y, c_y],
#                     [0.0, 0.0, 1.0]])

# distortion_coefficients = np.array([k_1, k_2, p_1, p_2, k_3])

class ArUcoMarkerPosture():
    def __init__(self):
        self.sub_camera_info = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.get_camera_info)    
        self.sub_markers = rospy.Subscriber('/camera/color/image_raw', Image, self.detect_aruco_markers)
        
        # self.pub_corners = rospy.Publisher('aruco_corners', Float64MultiArray, queue_size = 1)
        self.pub_corners_pose = rospy.Publisher('aruco_corners_new', ArucoMarkerArray, queue_size = 1)
                
    def get_camera_info(self, camera_info):
        self.height = camera_info.height
        self.width = camera_info.width
        self.frame_id = camera_info.header.frame_id        
        self.intrinsic_matrix = np.array(camera_info.K).reshape(3, 3)
        self.distortion_coefficients = np.array(camera_info.D)

    def detect_aruco_markers(self, data):
        global corners
        # Constant parameters used in Aruco methods
        markerLength = 0.020
        axisLength = 0.010
        ARUCO_PARAMETERS = aruco.DetectorParameters_create()
        # ARUCO_PARAMETERS.adaptiveThreshConstant = 10
        ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_1000)

        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")#data.encoding)
        except CvBridgeError as e:
            print(e)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect Aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters = ARUCO_PARAMETERS)

        font = cv2.FONT_HERSHEY_SIMPLEX
        if ids is not None:
            # self.publish_aruco_marker_corners(corners)

            ### NOTICE: some Opencv version use 2 output "rvec, tvec", others use 3 output "rvec, tvec, _"!!!!
            rvecs, tvecs = aruco.estimatePoseSingleMarkers(corners, markerLength, self.intrinsic_matrix, self.distortion_coefficients)
            # (rvec - tvec).any()  # get rid of that nasty numpy value array error

            # print("ids", ids[:][0])
            self.publish_aruco_corners_pose(ids, corners, rvecs, tvecs)

            # # Print corners and ids to the console
            # for i, corner in zip(ids, corners):
            #     print('ID: {};\nCorners:\n{}'.format(i, corner))    

            strg = ''
            for n in range(0, ids.size):
                strg += str(ids[n][0])+', '
                aruco.drawAxis(cv_image, self.intrinsic_matrix, self.distortion_coefficients, rvecs[n], tvecs[n], axisLength)
                # print(rvecs[n])
                # print(tvecs[n])


            # Outline all of the markers detected in our image
            aruco.drawDetectedMarkers(cv_image, corners)#, ids)#, borderColor = (0, 0, 255))
            cv2.putText(cv_image, "Id: " + strg, (10,30), font, 1, (0,0,255), 2, cv2.LINE_AA)

        else:
            # code to show 'No Ids' when no markers are found
            cv2.putText(cv_image, "No Ids", (10,30), font, 1, (0,0,255), 2, cv2.LINE_AA)

        cv2.imshow('cv_image', cv_image)
        cv2.waitKey(1)

    def publish_aruco_corners_pose(self, ids, corners, rvecs, tvecs):
        aruco_msgs = ArucoMarkerArray()

        ids_list = []
        corners_list = []
        rvecs_list = []
        tvecs_list = []

        for id, corner, rvec, tvec in zip(ids, corners, rvecs, tvecs):
            print('ID: {};\nCorners:\n{}'.format(id, corner))

            ids_list.append(int(id))

            for angle in range(0, len(corner)):
                corners_list.append(corner[0][angle][0]) # angle.pixel_x
                corners_list.append(corner[0][angle][1]) # angle.pixel_y

            
            rvecs_list.append(rvec[0][0])
            rvecs_list.append(rvec[0][1])
            rvecs_list.append(rvec[0][2])

            tvecs_list.append(tvec[0][0])
            tvecs_list.append(tvec[0][1])
            tvecs_list.append(tvec[0][2])         

        aruco_msgs.ids = ids_list
        aruco_msgs.corners = corners_list
        aruco_msgs.rvecs = rvecs_list
        aruco_msgs.tvecs = tvecs_list

        self.pub_corners_pose.publish(aruco_msgs)

    # def publish_aruco_marker_corners(self, corners):              
    #     corners_list = []
    #     for num in range(0, len(corners)):
    #         corner = corners[num][0]
            
    #         for angle in range(0, len(corner)):                
    #             corners_list.append(corner[angle][0]) # angle.pixel_x
    #             corners_list.append(corner[angle][1]) # angle.pixel_y
        
    #     corners_ros_msg = Float64MultiArray()
    #     corners_ros_msg.data = corners_list
    #     # rospy.loginfo(corners_ros_msg)     
    #     self.pub_corners.publish(corners_ros_msg)


if __name__ == '__main__':

    rospy.init_node("detect_aruco_markers")

    try:
        ArUcoMarkerPosture()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()