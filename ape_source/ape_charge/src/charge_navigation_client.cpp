#include "ros/ros.h"

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
#include "message_filters/time_synchronizer.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "ape_msgs/PoseWithScan.h"
#include "ape_actions/ApeChargeAction.h"
#include "std_msgs/Bool.h"
#include <mutex>     
#include <actionlib/client/simple_action_client.h>


std::mutex mtx_;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, 
            geometry_msgs::PoseStamped> mySyncPolicy;

typedef actionlib::SimpleActionClient<ape_actions::ApeChargeAction> goalClient;

goalClient *g_client;




bool has_charge_goal = false;
ape_msgs::PoseWithScan charge_station;

ape_msgs::PoseWithScan pair_goal;
bool has_laser_saved = false;



void chargeSet(std_msgs::Bool msg)
{
    if(msg.data)
    {
        std::cout << "---set current pose as charge station----" << std::endl;
        if(has_laser_saved) 
        {
            mtx_.lock();
            charge_station = pair_goal;
            mtx_.unlock();
            has_charge_goal = true;

        }
        
    }
}

void doChargeCallBack(std_msgs::Bool msg)
{
    if(msg.data)
    {
        // start action
        if(has_charge_goal)
        {
            // std::cout << "---do charge----" << std::endl;     
            g_client->waitForServer();
            std::cout << "---start to go to charge station----" << std::endl;       
            ape_actions::ApeChargeGoal send_goal;
          
            
            send_goal.goal_scan = charge_station.scan;

            send_goal.goal_pose = charge_station.pose.pose;

            g_client->sendGoal(send_goal);
        }
        else
        {
            std::cout << "---has no  charge station----" << std::endl;       

        }


    }
}



void laserAndPoseReceived(const sensor_msgs::LaserScanConstPtr& laser_scan,
                        const geometry_msgs::PoseStampedConstPtr& msg)
{
    // std::cout << laser_scan->header.stamp << "      "<< msg->header.stamp << std::endl;
    mtx_.lock();
    pair_goal.pose = *msg;
    pair_goal.scan = *laser_scan;
    mtx_.unlock();
    has_laser_saved = true;
  
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "charge_navigation_client");
    ros::NodeHandle nh;


   
    message_filters::Subscriber<geometry_msgs::PoseStamped>* robot_pose_sub_
        = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh, "continue_pose", 1);
    
    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_ 
        = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, "scan", 1);
    
    message_filters::Synchronizer<mySyncPolicy>* sync_ = new  message_filters::Synchronizer<mySyncPolicy>(mySyncPolicy(10), *laser_scan_sub_, * robot_pose_sub_);

    sync_->registerCallback(boost::bind(&laserAndPoseReceived, _1, _2));


    ros::Subscriber set_charge_sub_ = nh.subscribe("set_charge", 1, &chargeSet);

    ros::Subscriber do_charge_sub_ = nh.subscribe("do_charge", 1, &doChargeCallBack);


    g_client = new goalClient("dock_charge", true);


    ros::spin();


    
 
 

    return 0;
}