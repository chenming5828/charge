#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>



nav_msgs::Path path_icp;

void improve_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  
    path_icp.poses.push_back(*msg);
}


int main (int argc, char **argv)
{
    ros::init (argc, argv, "show_trajectory");

    ros::NodeHandle ph;
    std::cout << "showpose node up " << std::endl;

    ros::Publisher path_pub_improve = ph.advertise<nav_msgs::Path>("trajectory_icp",2, true);

    ros::Subscriber improve_pose_sub = ph.subscribe("continue_pose", 1, improve_pose_callback);



    path_icp.header.stamp=ros::Time::now();
    path_icp.header.frame_id="map";



    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        path_pub_improve.publish(path_icp);
        
        ros::spinOnce();              

        loop_rate.sleep();
    }

    return 0;
}
