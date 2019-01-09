#include "icp_pose.h"

boost::shared_ptr<IcpPose> icp_pose_ptr;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "charge_nav");
  ros::NodeHandle nh("~");


  icp_pose_ptr.reset(new IcpPose());
    ros::spin();
  icp_pose_ptr.reset();

  return 0;
}