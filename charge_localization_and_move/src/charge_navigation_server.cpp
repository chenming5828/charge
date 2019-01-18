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
#include <mutex>     


#include <csm/csm_all.h>   
#undef min
#undef max


typedef actionlib::SimpleActionServer<charge_localization_and_move::chargeAction> chargeServer;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient;


std::mutex g_mtx;

chargeServer* g_charge_server;

moveBaseClient* g_move_base;
sensor_msgs::LaserScan  lastest_scan;

sm_params input_;
sm_result output_;

void laserScanToLDP(sensor_msgs::LaserScan& scan_msg,
                                            LDP& ldp);


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

    sensor_msgs::LaserScan goal_scan = goal->goal_scan;

    ros::NodeHandle n;
    ros::Publisher vel_pub =  n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1, true);
    
    double e_yaw = 0;
    double e_yaw_pre = 0;
    double e_dis = 0;
    double e_dis_pre = 0;
    double kp_yaw = 6;
    double kd_yaw = 10;
    double kp_dis = 6;
    double kd_dis = 10;
    bool flag_pid = false;
    const double w_max = 0.3;
    double error_x ,error_y,error_yaw;
    while(n.ok())
    {
        g_mtx.lock();
        sensor_msgs::LaserScan scan_copy = lastest_scan ;
        g_mtx.unlock();

        LDP scan_cur;
        LDP scan_ref;
        laserScanToLDP(goal_scan,scan_ref);
        laserScanToLDP(scan_copy,scan_cur);

        input_.laser_ref  = scan_ref;
        input_.laser_sens = scan_cur;
        input_.min_reading =  goal_scan.range_min;
        input_.max_reading =  goal_scan.range_max;
 
        if (output_.cov_x_m)
        {
          gsl_matrix_free(output_.cov_x_m);
          output_.cov_x_m = 0;
        }
        if (output_.dx_dy1_m)
        {
          gsl_matrix_free(output_.dx_dy1_m);
          output_.dx_dy1_m = 0;
        }
        if (output_.dx_dy2_m)
        {
          gsl_matrix_free(output_.dx_dy2_m);
          output_.dx_dy2_m = 0;
        }
               
        sm_icp(&input_, &output_);

        if (output_.valid && (output_.error/output_.nvalid) < 0.05 )
        {
            
            geometry_msgs::Twist vel;
            vel.linear.y= 0;
            vel.linear.z= 0;
            vel.linear.x= -0.2;
            vel.angular.x= 0;
            vel.angular.y= 0;
            vel.angular.z= 0;
            if(output_.x[0] < 0.003 )
            {
              vel.linear.x= 0;
              vel_pub.publish(vel);
              std::cout << "end:  "<< output_.x[0] << "  "<< output_.x[1] << "  "<< output_.x[2]  << std::endl;
              break;
            }

            if(output_.x[0] < 0.1)
            {
              vel.linear.x= -0.1;
            }




            e_yaw = 0 - output_.x[2];
            e_dis = output_.x[1];
            if(flag_pid)
            {
              vel.angular.z = kp_dis * e_dis + kd_dis * e_dis_pre + kp_yaw * e_yaw + kd_yaw * e_yaw_pre;
            }
            else
            {
              flag_pid = true;
              error_x = output_.x[0];
              error_y = output_.x[1];
              error_yaw = output_.x[2];
              vel.angular.z = kp_dis * e_dis + kp_yaw * e_yaw; 
            }
            std::cout << "======================================================"<< std::endl;
            std::cout << error_x << "  "<< error_y << "  "<< error_yaw  << std::endl;
            std::cout << e_dis << "  "<< e_yaw << std::endl;
            std::cout << output_.x[0] << "  "<< output_.x[1] << "  "<< output_.x[2] << "   " << vel.angular.z << std::endl;
            vel_pub.publish(vel);


            e_dis_pre = e_dis;
            e_yaw_pre = e_yaw;


           
        }
        else
        {
            std::cout << "error with icp" << std::endl;
            std::cout << output_.valid << "  "<< output_.error/output_.nvalid << std::endl;


        }
        ld_free(scan_cur);
        ld_free(scan_ref);






    }
    geometry_msgs::Twist vel;
    vel.linear.y= 0;
    vel.linear.z= 0;
    vel.linear.x= 0;
    
    vel.angular.x= 0;
    vel.angular.y= 0;
    vel.angular.z= 0;

    vel_pub.publish(vel);
    vel_pub.publish(vel);
    vel_pub.publish(vel);
    std::cout << "arrival in dock station" << std::endl;
    g_charge_server->setSucceeded();
}


void laserScanToLDP(sensor_msgs::LaserScan& scan_msg,
                                            LDP& ldp)
{
  unsigned int n = scan_msg.ranges.size();
  ldp = ld_alloc_new(n);
 

  for (unsigned int i = 0; i < n; i++)
  {

 
    double r = scan_msg.ranges[i];
 
    if (r > scan_msg.range_min && r < scan_msg.range_max)
    {
 
 
      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1; 
    }
   
    ldp->theta[i]    = scan_msg.angle_min + i * scan_msg.angle_increment;
 
    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];
 
  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;
 
  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
    g_mtx.lock();
    lastest_scan = *laser_scan;
    g_mtx.unlock();

}

void initIcpParams()
{
 
    input_.laser[0] = 0.0;
    input_.laser[1] = 0.0;
    input_.laser[2] = 0.0;
 
    input_.first_guess[0] = 0;
    input_.first_guess[1] = 0;
    input_.first_guess[2] = 0;

    output_.cov_x_m = 0;
    output_.dx_dy1_m = 0;
    output_.dx_dy2_m = 0;
    input_.max_angular_correction_deg = 60.0;
    input_.max_linear_correction = 0.7;
    input_.max_correspondence_dist = 3.4;
    input_.max_iterations = 30;
    input_.epsilon_xy = 0.0001;
    input_.epsilon_theta = 0.0001;
    input_.sigma = 0.010;
    input_.use_corr_tricks = 1;
    input_.restart = 0;
    input_.restart_threshold_mean_error = 0.01;
    input_.restart_dt = 1.0;
    input_.restart_dtheta = 0.1;
    input_.clustering_threshold = 0.25;
    input_.orientation_neighbourhood = 20;
    input_.use_point_to_line_distance = 1;
    input_.do_alpha_test = 0;
    input_.do_alpha_test_thresholdDeg = 20.0; 
    input_.outliers_maxPerc = 0.90;
    input_.outliers_adaptive_order = 0.7;
    input_.outliers_adaptive_mult = 2.0;
    input_.do_visibility_test = 0; 
    input_.outliers_remove_doubles = 1;
    input_.do_compute_covariance = 0;
    input_.debug_verify_tricks = 0;
    input_.use_ml_weights = 0;
    input_.use_sigma_weights = 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "charge_navigation_server");
    ros::NodeHandle nh;

    ros::Subscriber laser_sub = nh.subscribe("scan", 1, &laserReceived);
    initIcpParams();

    g_charge_server = new chargeServer(nh, "dock_charge", boost::bind(&executeCb, _1), false);
    g_move_base = new moveBaseClient("move_base",true);
    g_charge_server->start();
    ros::spin();
 

    return 0;
}