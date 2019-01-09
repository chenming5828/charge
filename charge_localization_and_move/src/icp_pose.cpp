#include "icp_pose.h"



 IcpPose::IcpPose()
 {
    get_goal_= false;

    laser_sub_ = nh_.subscribe("scan", 1, &IcpPose::laserReceived, this);
    goal_sub_ = nh_.subscribe("goal_scan_amcl", 1, &IcpPose::goalReceived, this);
    arrive_sub_ = nh_.subscribe("arrive_goal", 1, &IcpPose::goalArrive, this);
    
    ref_goal_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("ref_pose_goal", 1, true);
   
    initIcpParams();
 }

 void IcpPose::goalArrive()
 {

 }

 void IcpPose::goalReceived(const charge_localization_and_move::posewithscanConstPtr& msg)
 {
     get_goal_= true;
     goal_ = *msg;

 }


 IcpPose::~IcpPose()
 {

 }

 void IcpPose::laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                            LDP& ldp)
{
  unsigned int n = scan_msg->ranges.size();
  ldp = ld_alloc_new(n);
 

  for (unsigned int i = 0; i < n; i++)
  {

 
    double r = scan_msg->ranges[i];
 
    if (r > scan_msg->range_min && r < scan_msg->range_max)
    {
 
 
      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else
    {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1; 
    }
   
    ldp->theta[i]    = scan_msg->angle_min + i * scan_msg->angle_increment;
 
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

 void IcpPose::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
 {
     if(!get_goal_)
     {
         return;
     }
    LDP scan_ref;
    LDP scan_cur;

    laserScanToLDP(laser_scan,scan_cur);
    laserScanToLDP(goal_laser_scan_,scan_ref);

    input_.laser_ref  = scan_ref;
    input_.laser_sens = scan_cur;

    input_.min_reading = laser_scan->range_min;
    input_.max_reading = laser_scan->range_max;
 
 
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

    if(output_.valid)
    {
        geometry_msgs::Pose2D ch;
        ch.x = output_.x[0];
        ch.y = output_.x[1];
        ch.theta = output_.x[2];
        ref_goal_pose_pub_.publish(ch);
    }

    ld_free(scan_ref);
    ld_free(scan_cur);


 }

 void IcpPose::initIcpParams()
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
 
    
    input_.max_angular_correction_deg = 90.0;
    input_.max_linear_correction = 2.80;
   
    input_.max_correspondence_dist = 4.0;
 

    input_.max_iterations = MAXITERATIONS;

    input_.epsilon_xy = 0.0001;

    input_.epsilon_theta = 0.0001;
    
 
 
  // Noise in the scan (m)
    input_.sigma = 0.010;
  // Use smart tricks for finding correspondences.
     input_.use_corr_tricks = 1;
  // Restart: Restart if error is over threshold
    input_.restart = 0;
  // Restart: Threshold for restarting
    input_.restart_threshold_mean_error = 0.01;
  // Restart: displacement for restarting. (m)
    input_.restart_dt = 1.0;
  // Restart: displacement for restarting. (rad)
    input_.restart_dtheta = 0.1;
  // Max distance for staying in the same clustering
     input_.clustering_threshold = 0.25;
  // Number of neighbour rays used to estimate the orientation
    input_.orientation_neighbourhood = 20;
  // If 0, it's vanilla ICP
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
