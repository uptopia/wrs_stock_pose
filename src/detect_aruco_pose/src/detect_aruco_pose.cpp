// ============================//
//   Get Aruco Markers' Cloud
// ============================//
// written by Shang-Wen, Wong.
// 2021.5.30

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

//tf
// #include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

//====
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//====

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> //"pcl::fromROSMsg"

// aruco
#include <aruco_msgs/ArucoMarkerArray.h>

#include <boost/make_shared.hpp>

//opencv
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/kdtree/kdtree_flann.h>

// C++
#include <vector>
#include <iostream>
#include <algorithm>

#include <Eigen/Core>
#include <pcl/common/common.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf/tf.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointTRGB;
typedef boost::shared_ptr<pcl::PointCloud<PointTRGB>> PointCloudTRGBPtr;

using namespace std;

struct Center2D
{
    int x;
    int y;
};

struct Center3D
{
    float x;
    float y;
    float z;
};

struct ArucoMarker
{
    int id;
    std::vector<float> corner_pixel; //x1, y1, ..., x4, y4 
    Center2D center_pixel;
    Center3D center_point;
    PointCloudTRGBPtr marker_cloud;
    std::vector<double> rvec;
    std::vector<double> tvec;
    // Eigen::Matrix4f marker_pose; #from aruco rvec,tvec/pointcloud normal
};

std::vector<ArucoMarker> marker_all{};


// bool save_organized_cloud = true;

std::string file_path_cloud_organized = "organized_cloud_tmp.pcd";

pcl::PointCloud<PointTRGB>::Ptr organized_cloud_ori(new pcl::PointCloud<PointTRGB>);

ros::Publisher aruco_cloud_pub;
sensor_msgs::PointCloud2 aruco_cloud_msg;


void get_3d_center(pcl::PointCloud<PointTRGB>::Ptr & cloud_ori, pcl::PointCloud<PointTRGB>::Ptr & cloud_obj, Center2D center_pixel, Center3D& center_point)
{
    //==========================================//
    // Get Center 3D Points
    // map 2D center_pixel to 3D center_point
    //==========================================//
    // int center_x = sauces_all[n].center_pixel.x;
    // int center_y = sauces_all[n].center_pixel.y;

    PointTRGB center_pt_3d = cloud_ori->at(center_pixel.x, center_pixel.y);
    cout << "\tCenter_pt_3d = " << center_pt_3d.x << ", " << center_pt_3d.y << ", " << center_pt_3d.z << endl;

    // if Center_pt_3d is NAN, use all cluster's points
    if(!pcl_isfinite(center_pt_3d.x) || !pcl_isfinite(center_pt_3d.y) || !pcl_isfinite(center_pt_3d.z))
    {
        int total_points = cloud_obj->size();
        center_pt_3d.x = 0;
        center_pt_3d.y = 0;
        center_pt_3d.z = 0;

        for(int kk = 0; kk < total_points; ++kk)
        {
            PointTRGB pt = cloud_obj->points[kk];
            center_pt_3d.x += pt.x;
            center_pt_3d.y += pt.y;
            center_pt_3d.z += pt.z;
        }

        center_pt_3d.x /= total_points;
        center_pt_3d.y /= total_points;
        center_pt_3d.z /= total_points;

        cout << "\t**Center_pt_3d = " << center_pt_3d.x << ", " << center_pt_3d.y << ", " << center_pt_3d.z << endl;
    }
    center_point.x = center_pt_3d.x;
    center_point.y = center_pt_3d.y;
    center_point.z = center_pt_3d.z;         
}

void aruco_corners_cb(const std_msgs::Float64MultiArray::ConstPtr& corners_msg)
{
    // cout << "aruco_corners_cb" << endl;
    // ROS_INFO("I heard: [%f],[%f],[%f],[%f]", corners_msg->data.at(0),corners_msg->data.at(1),corners_msg->data.at(2),corners_msg->data.at(3));
    
    int total_marker_num = 0;

    if(!corners_msg->data.empty())
    {
        total_marker_num = corners_msg->data.size() / 8; 
        marker_all.resize(total_marker_num);

        for(int k = 0; k < total_marker_num; ++k)
        {
            int x1 = corners_msg->data.at(8*k);
            int y1 = corners_msg->data.at(8*k+1);
            int x3 = corners_msg->data.at(8*k+4);       
            int y3 = corners_msg->data.at(8*k+5);

            // marker_all[k].id = corners_msg->markers[k].ID;
            
            marker_all[k].corner_pixel.assign(corners_msg->data.begin()+8*k, corners_msg->data.begin()+8*k+8);
            marker_all[k].center_pixel.x = int((x1 +x3)/2.0);
            marker_all[k].center_pixel.y = int((y1 +y3)/2.0);
        }
    }
    else
        total_marker_num = 0;
        marker_all.resize(total_marker_num);
 
    cout << "Total ArUco Markers Detected = " << total_marker_num << endl;   //ERROR: display 1 even if no obj detected
}

void aruco_corners_new_cb(const aruco_msgs::ArucoMarkerArray::ConstPtr& aruco_msg)
{
    // cout << "aruco_corners_cb" << endl;
    // ROS_INFO("I heard: [%f],[%f],[%f],[%f]", corners_msg->data.at(0),corners_msg->data.at(1),corners_msg->data.at(2),corners_msg->data.at(3));
    
    int total_marker_num = 0;
    cout<<aruco_msg->ids[0]<<", "<<aruco_msg->ids.empty()<<", "<< aruco_msg->ids.size()<<","<<(aruco_msg->ids.size()!=0)<<endl;
    // if(!aruco_msg->ids.empty())
    if(aruco_msg->ids.size()!=0)
    {
        total_marker_num = aruco_msg->corners.size() / 8; 
        cout<<"total_marker_num:"<<aruco_msg->corners.size()<<","<<aruco_msg->corners.size()/8<<endl;
        cout<<"rvecs size: "<<aruco_msg->rvecs.size()<<endl;
        cout<<"tvecs size: "<<aruco_msg->tvecs.size()<<endl;
        marker_all.resize(total_marker_num);

        for(int k = 0; k < total_marker_num; ++k)
        {
            
            int x1 = aruco_msg->corners.at(8*k);            
            int y1 = aruco_msg->corners.at(8*k+1);            
            int x3 = aruco_msg->corners.at(8*k+4);            
            int y3 = aruco_msg->corners.at(8*k+5);

            marker_all[k].id = aruco_msg->ids[k];            
            marker_all[k].corner_pixel.assign(aruco_msg->corners.begin()+8*k, aruco_msg->corners.begin()+8*k+8);            
            marker_all[k].center_pixel.x = int((x1 +x3)/2.0);            
            marker_all[k].center_pixel.y = int((y1 +y3)/2.0);            
            marker_all[k].rvec = {aruco_msg->rvecs.at(3*k), aruco_msg->rvecs.at(3*k+1), aruco_msg->rvecs.at(3*k+2)};            
            marker_all[k].tvec = {aruco_msg->tvecs.at(3*k), aruco_msg->tvecs.at(3*k+1), aruco_msg->tvecs.at(3*k+2)};
            
            cout<<"k:"<<k<<endl;
            cout<<"rvec:"<<marker_all[k].rvec[0]<<" "<<marker_all[k].rvec[1]<<" "<<marker_all[k].rvec[2]<<endl;
            cout<<"tvec:"<<marker_all[k].tvec[0]<<" "<<marker_all[k].tvec[1]<<" "<<marker_all[k].tvec[2]<<endl;            
        }
    }
    else
        total_marker_num = 0;
        marker_all.resize(total_marker_num);
 
    cout << "Total ArUco Markers Detected = " << total_marker_num << endl;   //ERROR: display 1 even if no obj detected

}

void CalculatePCA(pcl::PointCloud<PointTRGB>::Ptr & cloud, Eigen::Matrix3f eigenVectorsPCA)
{
    Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	// Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
 
	// std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
	// std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
	// std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;
}

void aruco_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& organized_cloud_msg)
{
    //==================================================//
    // 有序點雲 Organized Point Cloud; Depth Point Cloud
    // Subscribe "/camera/depth_registered/points" topic
    //==================================================//  

    int height = organized_cloud_msg->height; //480
    int width = organized_cloud_msg->width;   //640  
    int points = height * width;

    if((points==0))
    {
        cout<<"PointCloud No points!!!!!!\n";
        //break? pass?
    }
    else
    {
        // 將點雲格式由sensor_msgs/PointCloud2轉成pcl/PointCloud(PointXYZ, PointXYZRGB)
        organized_cloud_ori->clear();
        pcl::fromROSMsg(*organized_cloud_msg, *organized_cloud_ori);
      
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr aruco_clouds(new pcl::PointCloud<pcl::PointXYZRGBNormal>);        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr aruco_clouds_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);

        for(int n = 0; n < marker_all.size(); ++n)
        {
            cout << "Marker #" << n << endl;
            
            //TODO: 改畫marker的方法 要貼合marker 而不是bounding box

            //=========================================//
            // Extract Sauce's Depth Cloud(Orgainized)
            // 2D pixel mapping to 3D points
            //=========================================//
            marker_all[n].marker_cloud = boost::make_shared<pcl::PointCloud<PointTRGB>>();
         
            int x1 = marker_all[n].corner_pixel[0]; //corner #1
            int y1 = marker_all[n].corner_pixel[1];
            int x2 = marker_all[n].corner_pixel[2]; //corner #2
            int y2 = marker_all[n].corner_pixel[3];
            int x3 = marker_all[n].corner_pixel[4]; //corner #3
            int y3 = marker_all[n].corner_pixel[5];
            int x4 = marker_all[n].corner_pixel[6]; //corner #4        
            int y4 = marker_all[n].corner_pixel[7];

            std::vector<int> x_pixel{x1, x2, x3, x4};
            std::vector<int> y_pixel{y1, y2, y3, y4};
                      
            int xmin = *std::min_element(x_pixel.begin(), x_pixel.end());
            int xmax = *std::max_element(x_pixel.begin(), x_pixel.end());            
            int ymin = *std::min_element(y_pixel.begin(), y_pixel.end());
            int ymax = *std::max_element(y_pixel.begin(), y_pixel.end());
                       
            //Ensure the 2D pixels are inside image's max width, height
            if(xmin < 0) xmin = 0;
            if(ymin < 0) ymin = 0;      
            if(xmax > width-1) xmax = width;
            if(ymax > height-1) ymax = height;

            cout<<"\timgwidth, imgHeight = "<< width <<",  "<< height<<endl;
            cout<< "\tPixel (xmin, xmax, ymin, ymax) = "<< xmin << ", " << xmax <<", " << ymin << ", " << ymax << endl;            

            //Map 2D pixel to 3D points
            for(int i = xmin; i <= xmax; i++)
            {
                for(int j = ymin; j<= ymax; j++)
                {
                    PointTRGB depth_pt = organized_cloud_ori->at(i, j);
                
                    if(pcl_isfinite(depth_pt.x) && pcl_isfinite(depth_pt.y) && pcl_isfinite(depth_pt.z))
                    {
                        marker_all[n].marker_cloud->push_back(depth_pt);
                    }
                }
            }
            cout << "\tExtract [depth_cloud] = " << marker_all[n].marker_cloud->size() << endl;
            // *aruco_clouds = *aruco_clouds + *(marker_all[n].marker_cloud);
            *aruco_clouds_tmp = *aruco_clouds_tmp + *(marker_all[n].marker_cloud);
            
            //=========================================//
            // Rodrigues→3*3 rotation matrix→Quaternion
            //=========================================//
            //https://answers.ros.org/question/358188/converting-cvmat-rotation-matrix-to-quaternion/
            cv::Mat rot(3, 3, CV_64FC1);
            cv::Rodrigues(marker_all[n].rvec, rot); //cv::Rodrigues(cv::Vec3d({0.0, 0.0, 0.0}), rot);
            cout << "rot:" << rot << endl;

            // Convert to a tf2::Matrix3x3
            tf2::Matrix3x3 tf2_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
                                   rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
                                   rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));
            
            // tf2::Quaternion qqq;
            // tf2_rot.getRotation(qqq);
            // cout<<"qqq:"<<qqq.x()<<","<<qqq.y()<<","<<qqq.z()<<","<<qqq.w()<<endl;
                     
            //TODO: 若直接用ar mark 算出來的tvec則tf座標系位置有誤(需檢查是否座標系有誤);用點雲中間點求的話 中心點位在maker點雲中央
            // tf2::Vector3 vv = tf2::Vector3(marker_all[n].tvec[0],marker_all[n].tvec[1],marker_all[n].tvec[2]);
            
            get_3d_center(organized_cloud_ori, marker_all[n].marker_cloud, marker_all[n].center_pixel, marker_all[n].center_point); // Create a transform and convert to a Pose
            tf2::Vector3 vv = tf2::Vector3(marker_all[n].center_point.x,marker_all[n].center_point.y,marker_all[n].center_point.z);

            tf2::Transform tf2_transform(tf2_rot, vv);//tf2::Vector3());
            geometry_msgs::Pose pose_msg;
            tf2::toMsg(tf2_transform, pose_msg);
            cout<<"pose_msg:"<<pose_msg<<endl;

            std::string marker_coord_name = "marker_" + std::to_string(n);
            // cout<<"Marker_coord_name = "<< marker_coord_name <<endl;

            geometry_msgs::TransformStamped trans_Cam2tmp;
            trans_Cam2tmp.header.stamp = ros::Time::now();
            trans_Cam2tmp.header.frame_id = "camera_color_optical_frame";
            trans_Cam2tmp.child_frame_id = marker_coord_name;//"AAA_frame";            
            trans_Cam2tmp.transform.translation.x = pose_msg.position.x;
            trans_Cam2tmp.transform.translation.y = pose_msg.position.y;
            trans_Cam2tmp.transform.translation.z = pose_msg.position.z;
            trans_Cam2tmp.transform.rotation.x = pose_msg.orientation.x;
            trans_Cam2tmp.transform.rotation.y = pose_msg.orientation.y;
            trans_Cam2tmp.transform.rotation.z = pose_msg.orientation.z;
            trans_Cam2tmp.transform.rotation.w = pose_msg.orientation.w;

            static tf2_ros::StaticTransformBroadcaster sbr_tmp;
            sbr_tmp.sendTransform(trans_Cam2tmp);

            //顯示Target Pose
            geometry_msgs::TransformStamped trans_Cam2Tgt;
            trans_Cam2Tgt.header.stamp = ros::Time::now();
            trans_Cam2Tgt.header.frame_id = "camera_color_optical_frame";
            trans_Cam2Tgt.child_frame_id = "Target_frame";
            trans_Cam2Tgt.transform.translation.x = marker_all[n].center_point.x;// - 0.05;
            trans_Cam2Tgt.transform.translation.y = marker_all[n].center_point.y;  
            trans_Cam2Tgt.transform.translation.z = marker_all[n].center_point.z;
            trans_Cam2Tgt.transform.rotation.x = 0;
            trans_Cam2Tgt.transform.rotation.y = 0;
            trans_Cam2Tgt.transform.rotation.z = 1;
            trans_Cam2Tgt.transform.rotation.w = 0; 

            static tf2_ros::StaticTransformBroadcaster sbr2;
            sbr2.sendTransform(trans_Cam2Tgt);

            geometry_msgs::Pose pose_msg_tgt;
            pose_msg_tgt.position.x = trans_Cam2Tgt.transform.translation.x;
            pose_msg_tgt.position.y = trans_Cam2Tgt.transform.translation.y;
            pose_msg_tgt.position.z = trans_Cam2Tgt.transform.translation.z;
            pose_msg_tgt.orientation.x = trans_Cam2Tgt.transform.rotation.x;
            pose_msg_tgt.orientation.y = trans_Cam2Tgt.transform.rotation.y;
            pose_msg_tgt.orientation.z = trans_Cam2Tgt.transform.rotation.z;
            pose_msg_tgt.orientation.w = trans_Cam2Tgt.transform.rotation.w;
            cout<<"pose_msg_tgt:"<<pose_msg_tgt<<endl;


            // //=========================================//
            // // Estimate marker coordinate using point cloud 
            // //=========================================//
            // pcl::PointCloud<PointTRGB>::Ptr tmp(new pcl::PointCloud<PointTRGB>);
            // float leaf = 0.005;
            // pcl::VoxelGrid<PointTRGB> vg;
            // vg.setInputCloud(marker_all[n].marker_cloud);
            // vg.setLeafSize(leaf, leaf, leaf);
            // vg.filter(*tmp);

            // pcl::NormalEstimation<PointTRGB, pcl::Normal> nor;
            // nor.setInputCloud(marker_all[n].marker_cloud);
            // nor.setSearchSurface(tmp);

            // //以kdtree作为索引方式
            // pcl::search::KdTree<PointTRGB>::Ptr treeA(new pcl::search::KdTree<PointTRGB>);            
            // nor.setSearchMethod(treeA);            
            
            // //存储输出数据集
            // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);            
            
            // nor.setRadiusSearch(0.03);                      
            // nor.compute(*normals);

            // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tmpall(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            // pcl::copyPointCloud<PointTRGB,pcl::PointXYZRGBNormal>(*marker_all[n].marker_cloud, *tmpall);
            // for(int num = 0; num<tmpall->size(); num++)
            // {
            //     tmpall->points[num].normal_x = normals->points[num].normal_x;
            //     tmpall->points[num].normal_y = normals->points[num].normal_y;
            //     tmpall->points[num].normal_z = normals->points[num].normal_z;
            // }
            // *aruco_clouds = *aruco_clouds + *tmpall;//*(marker_all[n].marker_cloud);

            // //計算中心點位置
            // get_3d_center(organized_cloud_ori, marker_all[n].marker_cloud, marker_all[n].center_pixel, marker_all[n].center_point);
            // // cout<<marker_all[n].center_point.x<<", "<<marker_all[n].center_point.y<<", "<<marker_all[n].center_point.z<<endl;

            // //計算中心點姿態
            // Eigen::Matrix3f vect;            
            // CalculatePCA(marker_all[n].marker_cloud, vect);        
                        
            // double r, p, y;
            // tf2::Matrix3x3 mm;
            // mm.setValue(vect(0),vect(1),vect(2),vect(3),vect(4),vect(5),vect(6),vect(7),vect(8));
            // mm.getRPY(r,p,y); //0,1            

            // tf2::Quaternion quat;
            // quat.setRPY(r,p,y);

            // std::string marker_coord_name = "marker_" + std::to_string(n);
            // // cout<<"Marker_coord_name = "<< marker_coord_name <<endl;            

            // geometry_msgs::TransformStamped trans_Cam2Mrk;
            // trans_Cam2Mrk.header.stamp = ros::Time::now();
            // trans_Cam2Mrk.header.frame_id = "camera_color_optical_frame";
            // trans_Cam2Mrk.child_frame_id = marker_coord_name;//"AAA_frame";
            // trans_Cam2Mrk.transform.translation.x = marker_all[n].center_point.x;
            // trans_Cam2Mrk.transform.translation.y = marker_all[n].center_point.y;  
            // trans_Cam2Mrk.transform.translation.z = marker_all[n].center_point.z;
            // trans_Cam2Mrk.transform.rotation.x = quat.x();//0;
            // trans_Cam2Mrk.transform.rotation.y = quat.y();//0;
            // trans_Cam2Mrk.transform.rotation.z = quat.z();//0;
            // trans_Cam2Mrk.transform.rotation.w = quat.w();//1;

            // static tf2_ros::StaticTransformBroadcaster sbr1;
            // sbr1.sendTransform(trans_Cam2Mrk);

            // //顯示Target Pose
            // geometry_msgs::TransformStamped trans_Cam2Tgt;
            // trans_Cam2Tgt.header.stamp = ros::Time::now();
            // trans_Cam2Tgt.header.frame_id = "camera_color_optical_frame";
            // trans_Cam2Tgt.child_frame_id = "Target_frame";
            // trans_Cam2Tgt.transform.translation.x = marker_all[n].center_point.x;// - 0.05;
            // trans_Cam2Tgt.transform.translation.y = marker_all[n].center_point.y;  
            // trans_Cam2Tgt.transform.translation.z = marker_all[n].center_point.z;
            // trans_Cam2Tgt.transform.rotation.x = 0;
            // trans_Cam2Tgt.transform.rotation.y = 0;
            // trans_Cam2Tgt.transform.rotation.z = 1;
            // trans_Cam2Tgt.transform.rotation.w = 0; 

            // static tf2_ros::StaticTransformBroadcaster sbr2;
            // sbr2.sendTransform(trans_Cam2Tgt);

            // geometry_msgs::Pose pose_msg_tgt;
            // pose_msg_tgt.position.x = trans_Cam2Tgt.transform.translation.x;
            // pose_msg_tgt.position.y = trans_Cam2Tgt.transform.translation.y;
            // pose_msg_tgt.position.z = trans_Cam2Tgt.transform.translation.z;
            // pose_msg_tgt.orientation.x = trans_Cam2Tgt.transform.rotation.x;
            // pose_msg_tgt.orientation.y = trans_Cam2Tgt.transform.rotation.y;
            // pose_msg_tgt.orientation.z = trans_Cam2Tgt.transform.rotation.z;
            // pose_msg_tgt.orientation.w = trans_Cam2Tgt.transform.rotation.w;
            // cout<<"pose_msg_tgt:"<<pose_msg_tgt<<endl;

            // //計算轉換矩陣
            // // tf2::Matrix3x3 m1(quat);
            // // tf2::Matrix3x3 m2(tf2::Quaternion(0,0,1,0));

            // // tf2::Transform transform;            
            // // transform.setOrigin( tf::Vector3(marker_all[n].center_point.x, marker_all[n].center_point.y, marker_all[n].center_point.z));
            // // transform.setRotation( tf::Quaternion(0, 0, 0, 1)); 
            // // cout<<transform.
            // // <<endl;

            // // tf2::Matrix3x3 Matrix_tmp;
            // // tf::Vector3 v6,v7,v8;
            // // Matrix_tmp.setRotation(quat);
            // // v6=Matrix_tmp[0];
            // // v7=Matrix_tmp[1];
            // // v8=Matrix_tmp[2];
            // // std::cout<<"四元數q2對應的旋轉矩陣M:"<<v6[0]<<","<<v6[1]<<","<<v6[2]<<std::endl;
            // // std::cout<<"                       "<<v7[0]<<","<<v7[1]<<","<<v7[2]<<std::endl;
            // // std::cout<<"                       "<<v8[0]<<","<<v8[1]<<","<<v8[2]<<std::endl;


            // // cout<<m1<<endl;
            // // cout<<m2<<endl;
            // // m2.getRotation(quat);
            // // cout<<m2<<endl;

            // // //ROS tf
            // // static tf::TransformBroadcaster br;
            // // tf::Transform transform;
            
            // // transform.setOrigin( tf::Vector3(marker_all[n].center_point.x, marker_all[n].center_point.y, marker_all[n].center_point.z));
            // // transform.setRotation( tf::Quaternion(0, 0, 0, 1));             
            // // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_color_optical_frame", marker_coord_name));//camera_depth_frame
                        
            // // Eigen::Matrix3f vect;            
            // // CalculatePCA(marker_all[n].marker_cloud, vect);
            // // std::cout << "特征向量ve(3x3):\n" << vect << std::endl;
            // // tf2::Quaternion quat;
            // // tf2::Matrix3x3 mm;
            // // mm.setValue(vect(0),vect(1),vect(2),vect(3),vect(4),vect(5),vect(6),vect(7),vect(8));
            // // // mm.getRotation(quat);
            // // double r, p, y;
            
            // // // mm.to
            // // mm.getRPY(r,p,y); //0,1
            // // cout<<"rpy:"<<r<<", "<<p<<", "<<y<<endl;
            // // // r+=3.14159;
            // // // quat.normalize();
            // // // cout<<"quat:"<<quat.x()<<", "<<quat.y()<<", "<<quat.z()<<", "<<quat.w()<<endl;

            // ////ROS tf2
            // // static tf2_ros::StaticTransformBroadcaster sbr;
            // // geometry_msgs::TransformStamped transform;
            // // transform.header.stamp = ros::Time::now();
            // // transform.header.frame_id = "camera_color_optical_frame"; //"camera_link";
            // // transform.child_frame_id = marker_coord_name;
            // // transform.transform.translation.x = marker_all[n].center_point.x;
            // // transform.transform.translation.y = marker_all[n].center_point.y;
            // // transform.transform.translation.z = marker_all[n].center_point.z;

            // // // tf2::Quaternion quat;
            // // // quat.setRPY(0.0, 0.0, 0.0); //roll, pitch, yaw (around X, Y, Z)
            // // //quat.setRPY(tf::createQuaternionFromRPY (-1 * thetax, -1 * thetay, -1 * thetaz));
            // // cout<<"rpy:"<<r<<", "<<p<<", "<<y<<endl;
            // // cout<<"quat:"<<quat.x()<<", "<<quat.y()<<", "<<quat.z()<<", "<<quat.w()<<endl;
            // // quat.setRPY(r,p,y);            
            // // transform.transform.rotation.x = quat.x();
            // // transform.transform.rotation.y = quat.y();
            // // transform.transform.rotation.z = quat.z();
            // // transform.transform.rotation.w = quat.w();

            // // sbr.sendTransform(transform);

            // // // cout<<normals->points[0].normal_x<<", "<<normals->points[0].normal_y<<", "<<normals->points[0].normal_z<<", "<<normals->points[0].curvature<<endl;
        }

        //Publish pcl::PointCloud to ROS sensor::PointCloud2, and to topic
        // pcl::toROSMsg(*aruco_clouds, aruco_cloud_msg);
        pcl::toROSMsg(*aruco_clouds_tmp, aruco_cloud_msg);        
        aruco_cloud_msg.header.frame_id = "camera_color_optical_frame";        
        aruco_cloud_pub.publish(aruco_cloud_msg);        
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_cloud");
    cout<<"inside\n";
    ros::NodeHandle nh;

    // ArucoMarkers的角點 Corners of Aruco Markers
    // ros::Subscriber sub_aruco_corners = nh.subscribe<std_msgs::Float64MultiArray>("/aruco_corners", 1, aruco_corners_cb);
    ros::Subscriber sub_aruco_corners_new = nh.subscribe<aruco_msgs::ArucoMarkerArray>("/aruco_corners_new", 1, aruco_corners_new_cb);

    // ArUco Markers' Cloud
    ros::Subscriber sub_aruco_cloud = nh.subscribe("/camera/depth_registered/points", 1, aruco_cloud_cb);

    aruco_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/aruco_cloud", 1);
 
    cout<<"try\n";
    ros::spin();

    return 0;
}

