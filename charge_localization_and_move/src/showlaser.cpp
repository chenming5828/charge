// roscpp
#include "ros/ros.h"

// For transform support
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"  
#include <sensor_msgs/PointCloud.h>
  
  
  
  
  


std::shared_ptr<tf2_ros::Buffer> tf_;
std::shared_ptr<tf2_ros::TransformListener> tfl_;

ros::Publisher cloud_pub_;



 
         



void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{

  geometry_msgs::PoseStamped ident;
  geometry_msgs::PoseStamped map_pose;
  ident.header.frame_id = "laser";
  ident.header.stamp = laser_scan->header.stamp;
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
  try
  {
    tf_->transform(ident, map_pose, "map");
    // std::cout << "------get laser's map pose " << std::endl;

            double yaw =  tf2::getYaw(map_pose.pose.orientation);
            double ox  =  map_pose.pose.position.x;
            double oy  =  map_pose.pose.position.y;

            sensor_msgs::PointCloud amcl_cloud;
            amcl_cloud.header.stamp = laser_scan->header.stamp;
            amcl_cloud.header.frame_id = "map";
            amcl_cloud.points.resize(laser_scan->ranges.size());
 
            //we'll also add an intensity channel to the cloud
            amcl_cloud.channels.resize(1);
            amcl_cloud.channels[0].name = "rgb";
            amcl_cloud.channels[0].values.resize(laser_scan->ranges.size());

            for(int i = 0; i < laser_scan->ranges.size(); ++i)
            {
              
              float range = laser_scan->ranges[i];
              if (range > laser_scan->range_min && range < laser_scan->range_max)
              {
                  float angle = laser_scan->angle_min + i*laser_scan->angle_increment;
                  // laser 
                  float x = range * cos(angle);
                  float y = range * sin(angle);
                  // to map
                  amcl_cloud.points[i].x = ox + x * cos(yaw) - y * sin(yaw);
                  amcl_cloud.points[i].y = oy + x * sin(yaw) + y * cos(yaw);
 
 
              }
              else
              {
                  amcl_cloud.points[i].x = std::numeric_limits<float>::quiet_NaN();
                  amcl_cloud.points[i].y = std::numeric_limits<float>::quiet_NaN();
              }
   
 
              amcl_cloud.points[i].z = 5;
              amcl_cloud.channels[0].values[i] = 255;
            }
            cloud_pub_.publish(amcl_cloud);


  }
  catch(tf2::TransformException e)
  {
    ROS_WARN("Failed to compute map pose, skipping scan (%s)", e.what());
  }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_laser");
    ros::NodeHandle nh;
    std::cout << "show_laser node up " << std::endl;
  
    tf_.reset(new tf2_ros::Buffer());
    tfl_.reset(new tf2_ros::TransformListener(*tf_));

    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud>("laser_point", 2, true);

    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_ = 
          new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, "scan", 100);

    tf2_ros::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_ = 
          new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                             *tf_,
                                                             "map",
                                                             100,
                                                             nh);
    laser_scan_filter_->registerCallback(boost::bind(laserReceived, _1));




  
  ros::spin();
  
  return 0;
}