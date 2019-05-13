/*
    此代码尝试进行直线行驶时的避障
    具体思路是通过调用点云数据，检测在直线行驶时前方30cm以内的障碍物
    若存在障碍物，则将原有状态地图更新后
*/
#define PI 3.1415926

// common headers
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <cmath>
#include <sstream>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <laser_geometry/laser_geometry.h>

using namespace std;
using namespace cv;
using namespace pcl;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_frame (new pcl::PointCloud<pcl::PointXYZ>);
// 轨迹上每个点的实时曲率半径， 以及从控制器处获取到的速度和角速度
float current_curvature = 0;
float current_v = 0;
float current_w = 0;
std::vector<float> curvatures;

// 由里程计获取到的位置信息
float position_x = 0;
float position_y = 0;
float position_z = 0.0;

cv::Mat laser_with_predict = cv::Mat::zeros((480,480), CV_8UC1);
cv::Mat image_raw;

// 激光传感器的参数
float angle_start = 0；
float angle_end = 0；
float angle_increment = 0；
float capture_start = -1.04719753;
float capture_end = 1.04719753;

float target_start = -0.52359877;
float target_end = 0.52359877;

// 储存激光传感器返回数据中我们感兴趣的部分
sensor_msgs::PointCloud cloud;
std::vector<sensor_msgs/LaserScan> capture_points;
std::vector<sensor_msgs/LaserScan> target_points;

// 激光到点云的转换，以及tf变换
laser_geometry::LaserProjection projector_;
tf::TransformListener listener_;


class pcl_to_map:
private:
    // topic names
    std::string sub_laser_scan_topic;
    std::string sub_rgb_image_topic;
    std::string sub_odom_topic;
    std::string sub_init_pose_topic;
    std::string sub_goal_pose_topic;

    std::string pub_trans_pcl_topic;
    std::string pub_cmd_vel_topics;

    // ROS publishers & subscribers
    ros::Subscriber cmd_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber rgb_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber init_pose_sub;
    ros::Subscriber goal_pose_sub;

    ros::Publisher trans_pcl_pub;
    ros::Publisher cmd_pub;

    // 控制是否进行避障运算的flags
    std::bool is_straight_going;

    void laserCallback (const geometry_msgs::LaserScan::ConstPtr& msg)
    {
        if (is_straight_going == true)
        {
            angle_start = msg->angle_min;
            angle_end = msg->angle_max;
            angle_increment = msg->angle_increment;
            // @TODO
            // 此处需要根据实验调整感兴趣的激光雷达的扫描范围
            // 机器人前方120度范围
            int capture_index_start = int((capture_start - angle_start)/angle_increment);
            int capture_index_end = int((capture_end - angle_start)/angle_increment);
            // 机器人前方60度范围
            int target_index_start = int((target_start - angle_start)/angle_increment);
            int target_index_end = int ((target_end - angle_start)/angle_increment);
            // 遍历我们感兴趣的部分激光数据
            float min_distance = 999;
            int min_distance_index = 0;
            for (int i = capture_index_start; i<=capture_index_end; i++)
            {
                capture_points.push_back(msg->ranges[i]);
                if (i <= target_index_end && i >= target_index_end)
                {
                    target_points.push_back(msg->ranges[i]);
                    if (msg->ranges[i] < min_distance)
                    {
                        min_distance = msg->ranges[i];
                        min_distance_index = i;
                    }
                }
            }
            // 当由激光传感器计算出前方60度夹角范围内存在距离小于0.3m的点时，判断存在障碍物，停止机器人的轨迹规划与运动控制，启动避障程序
            // 将视觉传感器坐标系下的点云转换到base_link坐标系下，进而转换到map坐标系下，然后更新地图
            // 同时由odom话题读取到机器人当前时刻的位置和速度，作为新的路径规划的起点
            if (min_distance <= 0.3)
            {
                //首先发布一个空的Twist消息，使机器人停下
                geometry_msgs::Twist vel;
                cmd_pub.publish(vel);

                // 激光信息转化为点云信息
                // 将激光信息由传感器坐标系转换到base_link坐标系
                if(!listener_.waitForTransform(
                msg->header.frame_id,
                "/base_link",
                msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
                ros::Duration(1.0)))
                {
                    return;
                }
                projector_.transformLaserScanToPointCloud("/base_link", *msg, cloud, listener_);
                // @TODO 添加激光数据可视化的部分
                // 将激光转换成的点云发布出去
                trans_pcl_pub.publish(cloud);
            }
        }
    }

    void imageCallback (const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image_raw = cv_ptr->image;
    }

    void odomCallback (const nav_msgs::Odometry& msg)
    {
        // 获取当前速度信息
        current_v = msg->twist.twist.linear.x;
        current_w = msg->twist.twist.angular.z;
        current_curvature = current_v / current_w;

        // 数值待定， 需能体现机器人在近似地直线运动
        if (current_curvature >= 2 && current_v > 0)
        {
            is_straight_going = true;
        }
        // @TODO
        // 添加当前速度信息的可视化部分

        // 获取当前位置信息
        position_x = msg->pose.pose.position.x;
        position_y = msg->pose.pose.position.y;
        position_z = msg->pose.pose.position.z;
    }

public:
    int run(int argc, char** argv)
    {
        ROS_INFO("-------INIT--------");
        ros::init("pcl_to_map", argc, argv);
        ros::NodeHandle nh;

        

    }
