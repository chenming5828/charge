#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseStamped.h>


using namespace message_filters;

void callback(const geometry_msgs::PoseStampedConstPtr& image, const geometry_msgs::PoseStampedConstPtr& cam_info)
{
  std::cout << image->header.stamp << "  :  " << cam_info->header.stamp << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  ros::Publisher  pub_1 = nh.advertise<geometry_msgs::PoseStamped>("pose1",1);
  ros::Publisher  pub_2 = nh.advertise<geometry_msgs::PoseStamped>("pose2",1);

  message_filters::Subscriber<geometry_msgs::PoseStamped> pose1_sub(nh, "pose1", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose2_sub(nh, "pose2", 1);

  TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> sync(pose1_sub, pose2_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = 12;
        pose.pose.position.y = 12;
        pose.pose.position.z = 12;

        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        pub_1.publish(pose);
        // pose.header.stamp = ros::Time::now();
        pub_2.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}