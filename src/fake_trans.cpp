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

laser_geometry::LaserProjection projector_;
tf::TransformListener listener_;

ros::Publisher pcl_pub;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/base_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
  }

  sensor_msgs::PointCloud cloud;
  projector_.transformLaserScanToPointCloud("/base_link",*scan_in,
          cloud,listener_);

  // Do something with cloud.
}