#include "pcl_ros/transforms.h"
#include <fstream>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <string>
#include <sys/io.h>

std::string ins_data_dir;
std::string pcd_timestamp_path;

std::string ins_odom_dir;
std::string lidar_odom_dir = "../../../src/LIO-SAM/config/lidar_pose.txt";

std::string ins_pose_dir;
std::string lidar_pose_dir;

typedef sensor_msgs::PointCloud2 PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

static int g_index = 0;

// 写出带有时间戳的lidar_pose.txt数据 -- 格式：x,y,z,qw,qx,qy,qz
void WriteLaserOdomFileCallback(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
    double time =
        laserOdometry->header.stamp.sec + 1e-9 * laserOdometry->header.stamp.nsec;
    double qx = laserOdometry->pose.pose.orientation.x;
    double qy = laserOdometry->pose.pose.orientation.y;
    double qz = laserOdometry->pose.pose.orientation.z;
    double qw = laserOdometry->pose.pose.orientation.w;
    double x = laserOdometry->pose.pose.position.x;
    double y = laserOdometry->pose.pose.position.y;
    double z = laserOdometry->pose.pose.position.z;

    std::ofstream ofs;
    ofs.open(lidar_odom_dir, std::ios::app);
    if (!ofs.is_open())
    {
        ROS_INFO("Create lidar_pose.txt failed!");
        return;
    }
    // write txt file
    if(g_index >0){
      ofs << g_index << " " << std::setiosflags(std::ios::fixed) << std::setprecision(6)
        << time << " " << x << " " << y << " " << z << " " << qw << " " << qx
        << " " << qy << " " << qz << std::endl;
    }
    g_index = g_index + 1;
    ROS_INFO_STREAM("write lidar_pose frame:"<< g_index);
    ofs.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "writeLidarPose");
    ros::NodeHandle nh;
    
    /// 检查lidar_pose.txt文件
    if (access(lidar_odom_dir.c_str(), 0) == 0) {
      if (remove(lidar_odom_dir.c_str()) == 0) {
        ROS_INFO("Delete old lidar_pose.txt success!");
      } else {
        ROS_ERROR("Delete old lidar_pose.txt failed!");
      }
    }
    /// 订阅激光里程计tf，保存数据lidar_pose.txt
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(
        "lio_sam/mapping/odometry", 1000, WriteLaserOdomFileCallback);
    
    ros::spin();
    return 0;
}
