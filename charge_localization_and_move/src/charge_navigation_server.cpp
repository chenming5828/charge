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

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_broadcaster.h>
#include <mutex>     
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>




#include <csm/csm_all.h>   
#undef min
#undef max

typedef actionlib::SimpleActionServer<charge_localization_and_move::chargeAction> chargeServer;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient;

#define PREDOCK_FORWARD                 (2.0)
// #define KP_DIS_FAR                          (40)
// #define KP_DIS_NEAR                          (20)
// #define KD_DIS_FAR                          (10)
// #define KD_DIS_NEAR                          (20)
#define KI_DIS                          (0)
#define KP_DIS                         (100)
#define KD_DIS                          (0)
// #define KD_DIS                          (20)


#define KP_YAW                          (5)
#define KD_YAW                          (0)
#define KI_YAW                          (0)

#define STOP_DIS                         (0.003)
#define DIS_PID                         (0.02)


static double normalize(double z)
{
  return atan2(sin(z),cos(z));
}

class dock_server
{
    public:
        dock_server();
        ~dock_server() = default;

    private:
        sm_params                   m_input;
        sm_result                   m_output;
        std::mutex                  m_mtx;
        std::mutex                  m_mtx_pose;
        sensor_msgs::LaserScan      m_lastest_scan;
        bool                        m_get_laser;
        moveBaseClient*             m_move_base_client;
        chargeServer*               m_charge_server;

        ros::NodeHandle             m_nh;
        ros::Publisher              m_vel_pub;
        ros::Subscriber             m_laser_sub;

        message_filters::Subscriber<nav_msgs::Odometry>*    m_odom_sub;
        tf2_ros::MessageFilter<nav_msgs::Odometry>*         m_tf2_filter;

        std::shared_ptr<tf2_ros::Buffer> m_tf;
        std::shared_ptr<tf2_ros::TransformListener> m_tfl;

        geometry_msgs::PoseStamped     m_current_pose;
        bool                           has_robot_pose;

        //debug
        ros::Publisher   dis_pub ;
        ros::Publisher   yaw_pub ;

        double                      m_kp_yaw;          
        double                      m_kd_yaw;          
        double                      m_ki_yaw;          
        double                      m_kp_dis;          
        double                      m_kd_dis;          
        double                      m_ki_dis;          

        void initIcpParams();
        void laserScanToLDP(sensor_msgs::LaserScan& scan_msg,
                                            LDP& ldp);
        void dockGetPreDock(double bx,double by,double ba, 
            double& lx,double& ly,double& la, double forward);
        
        void executeCb(const charge_localization_and_move::chargeGoalConstPtr& goal);
        void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
        bool icpGetRelativePose(sensor_msgs::LaserScan& scan_cur,
                                    sensor_msgs::LaserScan& scan_ref);


        void odomTfCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);








};

  

dock_server::dock_server():
    m_move_base_client(NULL),
    m_charge_server(NULL),
    m_get_laser(false),
    has_robot_pose(false)
{

    m_tf.reset(new tf2_ros::Buffer());
    m_tfl.reset(new tf2_ros::TransformListener(*m_tf));

    m_odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(m_nh, "odom", 100);

    m_tf2_filter = new tf2_ros::MessageFilter<nav_msgs::Odometry>(*m_odom_sub,
                                                               *m_tf,
                                                               "map",
                                                                100,
                                                                m_nh);
    m_tf2_filter->registerCallback(boost::bind(&dock_server::odomTfCallback, this, _1));
    


    m_laser_sub = m_nh.subscribe("scan", 1, &dock_server::laserReceived,this);
    m_vel_pub = m_nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

    dis_pub = m_nh.advertise<std_msgs::Float64>("error_dis", 1);
    yaw_pub = m_nh.advertise<std_msgs::Float64>("error_yaw", 1);

    initIcpParams();

    m_charge_server = new chargeServer(m_nh, "dock_charge", boost::bind(&dock_server::executeCb, this, _1), false);
    m_move_base_client = new moveBaseClient("move_base",true);
    m_charge_server->start();

    // m_kp_yaw = 5;
    m_kp_yaw = KP_YAW;
    m_kd_yaw = KD_YAW;
    m_ki_yaw = KI_YAW;

    m_ki_dis = KI_DIS;
    m_kp_dis = KP_DIS;
    m_kd_dis = KD_DIS;

}

void dock_server::odomTfCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    // std::cout << "get a odom msg "<< std::endl;
    
    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header = odom_msg->header;
    odom_pose.pose = odom_msg->pose.pose;
    m_mtx_pose.lock();
    try 
    {
        
        m_tf->transform(odom_pose, m_current_pose, "map");
        has_robot_pose = true;
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("Failure %s\n", ex.what()); 
    }
    m_mtx_pose.unlock();

    
}

bool dock_server::icpGetRelativePose(sensor_msgs::LaserScan& scan_cur,sensor_msgs::LaserScan& scan_ref)
{
        LDP ldp_cur;
        LDP ldp_ref;
        laserScanToLDP(scan_ref,ldp_ref);
        laserScanToLDP(scan_cur,ldp_cur);

        m_input.laser_ref  = ldp_ref;
        m_input.laser_sens = ldp_cur;
        m_input.min_reading = scan_ref.range_min;
        m_input.max_reading = scan_ref.range_max;
 
        if (m_output.cov_x_m)
        {
          gsl_matrix_free(m_output.cov_x_m);
          m_output.cov_x_m = 0;
        }
        if (m_output.dx_dy1_m)
        {
          gsl_matrix_free(m_output.dx_dy1_m);
          m_output.dx_dy1_m = 0;
        }
        if (m_output.dx_dy2_m)
        {
          gsl_matrix_free(m_output.dx_dy2_m);
          m_output.dx_dy2_m = 0;
        }
               
        sm_icp(&m_input, &m_output);

        ld_free(ldp_cur);
        ld_free(ldp_ref);

        if (m_output.valid && (m_output.error/m_output.nvalid) < 0.05 )
        {
            return true;
        }
        return false;


}




void dock_server::executeCb(const charge_localization_and_move::chargeGoalConstPtr& goal)
{
    std::cout << "---start to go to charge dock" << std::endl;
    move_base_msgs::MoveBaseGoal pre_goal;
    double pre_x;
    double pre_y;
    double pre_a;
    double bx = goal->goal_pose.position.x;
    double by = goal->goal_pose.position.y;
    double ba = tf2::getYaw(goal->goal_pose.orientation);
    dockGetPreDock(bx,by,ba, pre_x,pre_y,pre_a,PREDOCK_FORWARD);

    pre_goal.target_pose.header.frame_id = "map";
    pre_goal.target_pose.header.stamp = ros::Time::now();
    pre_goal.target_pose.pose.position.x = pre_x;
    pre_goal.target_pose.pose.position.y = pre_y;
    pre_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pre_a);

    // tell move base to go to pre dock station
    m_move_base_client->sendGoal(pre_goal);
    
    std::cout << "go to pre dock" << std::endl;
    m_move_base_client->waitForResult();
    std::cout << "arrival in  pre dock" << std::endl;

    sensor_msgs::LaserScan goal_scan = goal->goal_scan;

   
    double e_yaw = 0;
    double e_yaw_pre = 0;

    double e_dis = 0;
    double e_dis_pre = 0;
    double e_dis_add = 0;


    double out_error = 0;
    // double out_error_pre = 0;
    // double out_error_add = 0;

    double in_error = 0;
    double in_error_pre = 0;
    // double in_error_add = 0;

    // const double out_max = 1.0;
    const double w_max = 0.3;

    bool flag_pid = false;
    double error_x ,error_y,error_yaw;
    ros::NodeHandle n;

    const double kp_out = 15;
    const double max_angle = 0.7;
    int cnt_flag = 0;
    const int cnt_max = 200;
    double angle_exp = 0;

    const double kp_in = 10;
    const double kd_in = 2;

    

    // ros::Rate r(100);
    //这个需要满足turtlebot的时间频率
    while(n.ok())
    {
        // r.sleep();
        if(!m_get_laser || !has_robot_pose)
        {
            std::cout <<" No laser or no Pose, please check laser and localization!!!" << std::endl;
            geometry_msgs::Twist vel;
            vel.linear.y= 0;
            vel.linear.z= 0;
            vel.linear.x= 0;
            
            vel.angular.x= 0;
            vel.angular.y= 0;
            vel.angular.z= 0;

            m_vel_pub.publish(vel);
            m_vel_pub.publish(vel);
            m_vel_pub.publish(vel);
            m_charge_server->setAborted();
            return;
        }
        if(cnt_flag == 0)
        {
            m_mtx.lock();
            sensor_msgs::LaserScan scan_copy = m_lastest_scan ;
            m_mtx.unlock();
            if(icpGetRelativePose(scan_copy,goal_scan))
            {

                out_error = m_output.x[1];
                angle_exp = kp_out * out_error;

                if(angle_exp > max_angle )
                {
                    angle_exp = max_angle;
                }
                else if(angle_exp < (-1.0 * max_angle))
                {
                    angle_exp = (-1.0 * max_angle);
                }    
            }
            else
            {
                // std::cout << "error no  icp,use amcl pose|| " 
                //     << m_output.valid << "  "<< m_output.error/m_output.nvalid << std::endl;
            
                m_mtx_pose.lock();
                geometry_msgs::PoseStamped pose_cur = m_current_pose;
                m_mtx_pose.unlock();
                double yaw_cur  = tf2::getYaw(pose_cur.pose.orientation);
                double offset_yaw = yaw_cur - ba;
                offset_yaw = normalize(offset_yaw);

                double m_x = pose_cur.pose.position.x - bx;
                double m_y = pose_cur.pose.position.y - by;
                // to goal cordinatory
                double offset_x = m_x * cos(ba) + m_y * sin(ba);
                double offset_y = m_y * cos(ba) - m_x * sin(ba);

        
               
                out_error = offset_y;
                angle_exp = kp_out * out_error;

                if(angle_exp > max_angle )
                {
                    angle_exp = max_angle;
                }
                else if(angle_exp < (-1.0 * max_angle))
                {
                    angle_exp = (-1.0 * max_angle);
                }
              
            }

            std::cout << "================================================================" << angle_exp << "   :   " << out_error << std::endl;
            
            in_error = 0;
            in_error_pre = 0;
            cnt_flag ++;
            

        }
        else
        {
            m_mtx.lock();
            sensor_msgs::LaserScan scan_copy = m_lastest_scan ;
            m_mtx.unlock();
            double x = 0;
            double y = 0;
            double zz = 0;
            if(icpGetRelativePose(scan_copy,goal_scan))
            {
                in_error = angle_exp - m_output.x[2];
                x = m_output.x[0];
                y = m_output.x[1];
                zz = m_output.x[2];
            }
            else
            {
                
                // std::cout << "error no  icp,use amcl pose|| " 
                //     << m_output.valid << "  "<< m_output.error/m_output.nvalid << std::endl;
            
                m_mtx_pose.lock();
                geometry_msgs::PoseStamped pose_cur = m_current_pose;
                m_mtx_pose.unlock();
                double yaw_cur  = tf2::getYaw(pose_cur.pose.orientation);
                double offset_yaw = yaw_cur - ba;
                offset_yaw = normalize(offset_yaw);

                double m_x = pose_cur.pose.position.x - bx;
                double m_y = pose_cur.pose.position.y - by;
                // to goal cordinatory
                double offset_x = m_x * cos(ba) + m_y * sin(ba);
                double offset_y = m_y * cos(ba) - m_x * sin(ba); 
                x = offset_x;
                y = offset_y;
                zz = offset_yaw;

                in_error = angle_exp - offset_yaw;


            }
            


            double z = in_error * kp_in + (in_error - in_error_pre) * kd_in;

            

            if(z > w_max)
            {
              z = w_max;
            }
            else if(z < -w_max)
            {
              z = -w_max;
            }

            std::cout << "yaw : "<< zz << "  in_error : " << in_error << " z :  " << z << " y: "<< y << std::endl;

            geometry_msgs::Twist vel;
            vel.linear.y = 0;
            vel.linear.z = 0;
            vel.linear.x = -0.15;
            vel.angular.x = 0;
            vel.angular.y = 0;
            vel.angular.z = 0;

            if(x < STOP_DIS )
            {
              vel.linear.x = 0;
              m_vel_pub.publish(vel);
            
              break;
            }

            if(x < 0.4)
            {
              vel.linear.x = -0.1;
            }
            vel.angular.z = z;
            m_vel_pub.publish(vel);
 
            in_error_pre = in_error;
            cnt_flag ++;
            cnt_flag = cnt_flag % cnt_max;

            
        }
    }
    geometry_msgs::Twist vel;
    vel.linear.y= 0;
    vel.linear.z= 0;
    vel.linear.x= 0;
    
    vel.angular.x= 0;
    vel.angular.y= 0;
    vel.angular.z= 0;

    m_vel_pub.publish(vel);
    m_vel_pub.publish(vel);
    m_vel_pub.publish(vel);

    m_mtx.lock();
    sensor_msgs::LaserScan scan_copy = m_lastest_scan ;
    m_mtx.unlock();
    icpGetRelativePose(scan_copy,goal_scan);
    std::cout << " end:  "<< m_output.x[0] << "  "<< m_output.x[1] << "  "<< m_output.x[2]  << std::endl;


    std::cout << "arrival in dock station" << std::endl;
    m_charge_server->setSucceeded();
}


void dock_server::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
    // std::cout << "get a laser " << std::endl;
    m_mtx.lock();
    m_lastest_scan = *laser_scan;
    m_get_laser = true;
    m_mtx.unlock();
}

void dock_server::initIcpParams()
{
    m_input.laser[0] = 0.0;
    m_input.laser[1] = 0.0;
    m_input.laser[2] = 0.0;
 
    m_input.first_guess[0] = 0;
    m_input.first_guess[1] = 0;
    m_input.first_guess[2] = 0;

    m_output.cov_x_m = 0;
    m_output.dx_dy1_m = 0;
    m_output.dx_dy2_m = 0;
    m_input.max_angular_correction_deg = 70.0;
    m_input.max_linear_correction = 0.7;
    m_input.max_correspondence_dist = 10;
    m_input.max_iterations = 40;
    m_input.epsilon_xy = 0.0001;
    m_input.epsilon_theta = 0.0001;
    m_input.sigma = 0.010;
    m_input.use_corr_tricks = 1;
    m_input.restart = 0;
    m_input.restart_threshold_mean_error = 0.01;
    m_input.restart_dt = 1.0;
    m_input.restart_dtheta = 0.1;
    m_input.clustering_threshold = 0.25;
    m_input.orientation_neighbourhood = 20;
    m_input.use_point_to_line_distance = 1;
    m_input.do_alpha_test = 0;
    m_input.do_alpha_test_thresholdDeg = 20.0; 
    m_input.outliers_maxPerc = 0.90;
    m_input.outliers_adaptive_order = 0.7;
    m_input.outliers_adaptive_mult = 2.0;
    m_input.do_visibility_test = 0; 
    m_input.outliers_remove_doubles = 1;
    m_input.do_compute_covariance = 0;
    m_input.debug_verify_tricks = 0;
    m_input.use_ml_weights = 0;
    m_input.use_sigma_weights = 0;
}

void dock_server::dockGetPreDock(double bx,double by,double ba, 
            double& lx,double& ly,double& la, double forward)
{
    lx = bx + forward * cos(ba) ;
    ly = by + forward * sin(ba) ;
    la = ba ;
}



void dock_server::laserScanToLDP(sensor_msgs::LaserScan& scan_msg,
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

boost::shared_ptr<dock_server> dock_node_ptr;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "charge_navigation_server");
   
    
    dock_node_ptr.reset(new dock_server());
     ros::spin();
    dock_node_ptr.reset();
    return 0;
}

