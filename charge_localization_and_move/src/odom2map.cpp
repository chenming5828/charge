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

std::shared_ptr<tf2_ros::Buffer> tf_;
// tf listen
std::shared_ptr<tf2_ros::TransformListener> tfl_;

ros::Publisher pose_pub_;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    std::cout << "get odom" << std::endl;
    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header = odom_msg->header;
    odom_pose.pose = odom_msg->pose.pose;

    geometry_msgs::PoseStamped map_pose;
    try 
    {
        tf_->transform(odom_pose, map_pose, "map");
        
        ROS_ERROR("point of MAP in frame Position(x:%f y:%f z:%f)\n", 
            map_pose.pose.position.x,
            map_pose.pose.position.y,
            map_pose.pose.position.z);

        pose_pub_.publish(map_pose);

        // ROS_ERROR("----point of ODOM in frame Position(x:%f y:%f z:%f)\n", 
        //     odom_pose.pose.position.x,
        //     odom_pose.pose.position.y,
        //     odom_pose.pose.position.z);


    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
}

void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
    std::cout << "get laser" << std::endl;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom2map");
  ros::NodeHandle nh;

  std::cout << "ros init " << std::endl;

   
    tf_.reset(new tf2_ros::Buffer());
    tfl_.reset(new tf2_ros::TransformListener(*tf_));

 pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("continue_pose", 2, true);


  message_filters::Subscriber<nav_msgs::Odometry>* odom_sub_
     = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "odom", 100);

   tf2_ros::MessageFilter<nav_msgs::Odometry>* odom_filter_ = 
          new tf2_ros::MessageFilter<nav_msgs::Odometry>(*odom_sub_,
                                                               *tf_,
                                                               "map",
                                                                100,
                                                                nh);
  odom_filter_->registerCallback(boost::bind(odomCallback, _1));



  
  ros::spin();
  
  return 0;
}