#ifndef __ICP_POSE_H__
#define __ICP_POSE_H__


#include "ros/ros.h"
 

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"

#include "charge_localization_and_move/posewithscan.h"


#include <csm/csm_all.h>  
#undef min
#undef max



#define MAXITERATIONS (50)

class  IcpPose
{
    public:
        IcpPose();
        ~IcpPose();

    private:
        ros::NodeHandle nh_;


        charge_localization_and_move::posewithscan goal_;

        ros::Subscriber goal_sub_;
        ros::Subscriber arrive_sub_;


        ros::Subscriber laser_sub_;

        ros::Publisher ref_goal_pose_pub_;

        bool get_goal_;

 
               
        sm_params input_;
        sm_result output_;



        void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
        void initIcpParams();
        void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                            LDP& ldp);

        void goalReceived(const charge_localization_and_move::posewithscanConstPtr& msg);



};



#endif