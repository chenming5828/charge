#include "ros/ros.h"

#include "charge_localization_and_move/posewithscan.h"
#include "charge_localization_and_move/chargeAction.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

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
#include <tf/transform_broadcaster.h>

#include <csm/csm_all.h>   
#undef min
#undef max

typedef actionlib::SimpleActionServer<charge_localization_and_move::chargeAction> chargeServer;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient;



chargeServer* g_charge_server;

moveBaseClient* g_move_base;


void dockGetPreDock(double bx,double by,double ba, double& lx,double& ly,double& la)
{
    const double forward = 0.5;
    lx = bx + forward * cos(ba) ;
    ly = by + forward * sin(ba) ;
 
    la = ba ;
    
}

void executeCb(const charge_localization_and_move::chargeGoalConstPtr& goal) 
{
    std::cout << "---get a charge dock" << std::endl;
    move_base_msgs::MoveBaseGoal pre_goal;
    double pre_x;
    double pre_y;
    double pre_a;
    double bx = goal->goal_pose.position.x;
    double by = goal->goal_pose.position.y;
    double ba = tf2::getYaw(goal->goal_pose.orientation);
    dockGetPreDock(bx,by,ba, pre_x,pre_y,pre_a);
    pre_goal.target_pose.header.frame_id = "map";
    pre_goal.target_pose.header.stamp = ros::Time::now();
    pre_goal.target_pose.pose.position.x = pre_x;
    pre_goal.target_pose.pose.position.y = pre_y;
    pre_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pre_a);

    // tell move base to go to pre dock station
    g_move_base->sendGoal(pre_goal);
    
    std::cout << "go to pre dock" << std::endl;
    g_move_base->waitForResult();
    std::cout << "arrival in  pre dock" << std::endl;

    ros::NodeHandle n;
    while(n.ok())
    {

    }

    g_charge_server->setSucceeded();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "charge_navigation_server");
    ros::NodeHandle nh;

    g_charge_server = new chargeServer(nh, "dock_charge", boost::bind(&executeCb, _1), false);
    g_move_base = new moveBaseClient("move_base",true);
    g_charge_server->start();
    ros::spin();
 

    return 0;
}