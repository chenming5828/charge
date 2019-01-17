#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "nav_msgs/Odometry.h"




nav_msgs::Path path_odom;

void odom_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    std::cout <<"get ekf pose " << std::endl;
    geometry_msgs::PoseStamped pp;
    pp.header = msg->header;
    pp.header.frame_id = "map";
    pp.pose = msg->pose.pose;
    path_odom.poses.push_back(pp);
}


int main (int argc, char **argv)
{
    ros::init (argc, argv, "show_ekf");

    ros::NodeHandle ph;
    std::cout << "show_ekf node up " << std::endl;

    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("trajectory_ekf",2, true);

    ros::Subscriber odom_sub = ph.subscribe("robot_pose_ekf/odom_combined", 1, odom_callback);



    path_odom.header.stamp=ros::Time::now();
    path_odom.header.frame_id="map";



    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        path_pub.publish(path_odom);
        
        ros::spinOnce();              

        loop_rate.sleep();
    }

    return 0;
}
