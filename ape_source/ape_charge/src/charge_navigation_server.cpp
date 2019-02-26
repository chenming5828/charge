#include "ros/ros.h"

#include "ape_msgs/PoseWithScan.h"
#include "ape_actions/ApeChargeAction.h"
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

typedef actionlib::SimpleActionServer<ape_actions::ApeChargeAction> chargeServer;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient;

#define PREDOCK_FORWARD                 (1.0)

#define KI_DIS                          (0.03)
#define KP_DIS                         (30)
#define KD_DIS                          (5)



#define KP_YAW                          (5)
#define KD_YAW                          (0)
#define KI_YAW                          (0)

#define STOP_DIS                         (0.003)
#define ERROR_ICP                         (0.03)


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
        bool                           m_has_robot_pose;

        //debug
        ros::Publisher   dis_pub ;
        ros::Publisher   yaw_pub ;
        ros::Publisher   y_pub ;

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
        
        void executeCb(const ape_actions::ApeChargeGoalConstPtr& goal);
        void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
        bool icpGetRelativePose(sensor_msgs::LaserScan& scan_cur,
                                    sensor_msgs::LaserScan& scan_ref);


        void odomTfCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

        void pubZeroVel();









};

  

dock_server::dock_server():
    m_move_base_client(NULL),
    m_charge_server(NULL),
    m_get_laser(false),
    m_has_robot_pose(false)
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
    y_pub = m_nh.advertise<std_msgs::Float64>("error_y", 1);

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
        m_has_robot_pose = true;
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
        // std::cout<<  "size " << scan_cur.ranges.size()<< std::endl;
        const double valid_size = scan_cur.ranges.size()/3.0;

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

        if (m_output.valid && (m_output.error/m_output.nvalid) < ERROR_ICP && (m_output.nvalid > valid_size))
        {
            return true;
        }
        return false;


}




void dock_server::executeCb(const ape_actions::ApeChargeGoalConstPtr& goal)
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
    

    if(m_move_base_client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        std::cout << "move base error " << std::endl;
        m_charge_server->setAborted();
        return ;
    }

    std::cout << "arrival in  pre dock" << std::endl;
  

    sensor_msgs::LaserScan goal_scan = goal->goal_scan;

   
    double e_yaw = 0;
    double e_yaw_pre = 0;
    double e_yaw_add = 0;

    double e_dis = 0;
    double e_dis_pre = 0;
    double e_dis_add = 0;

    const double w_max = 0.4;

    double error_x ,error_y,error_yaw;
    bool flag_init = true;
    ros::NodeHandle n;

     



    
#if 1
    // ros::Rate r(100);
    //这个需要满足turtlebot的时间频率
    while(n.ok())
    {
        // r.sleep();
        if(!m_get_laser || !m_has_robot_pose)
        {
            std::cout <<" No laser or no Pose, please check laser and localization!!!" << std::endl;
            pubZeroVel();
            m_charge_server->setAborted();
            return;
        }

        m_mtx.lock();
        sensor_msgs::LaserScan scan_copy = m_lastest_scan ;
        m_mtx.unlock();


        // set first guess 
        m_mtx_pose.lock();
        geometry_msgs::PoseStamped pose_cur = m_current_pose;
        m_mtx_pose.unlock();
        double yaw_cur  = tf2::getYaw(pose_cur.pose.orientation);
        double offset_yaw = yaw_cur - ba;
        offset_yaw = normalize(offset_yaw);

        double m_x = pose_cur.pose.position.x - bx;
        double m_y = pose_cur.pose.position.y - by;
        
        double offset_x = m_x * cos(ba) + m_y * sin(ba);
        double offset_y = m_y * cos(ba) - m_x * sin(ba); 

        m_input.first_guess[0] = offset_x;
        m_input.first_guess[1] = offset_y;
        m_input.first_guess[2] = offset_yaw;
 

        double w = 0;
        double x = 0;
        double y = 0;
        double theda = 0;
        if(icpGetRelativePose(scan_copy,goal_scan))
        {
            std::cout << "----------------use icp-------------------" << std::endl;
            std::cout << m_output.x[0] << "  " << m_output.x[1] << "  " << m_output.x[2]<< std::endl;
            std::cout << m_output.error/m_output.nvalid << " " << m_output.nvalid << std::endl;

            x = m_output.x[0];
            y = m_output.x[1];
            theda = m_output.x[2];

 

        }
        else
        {
            std::cout << "----------------use odom-------------------" << std::endl;
            std::cout << m_output.x[0] << "  " << m_output.x[1] << "  " << m_output.x[2]<< std::endl;
            std::cout << m_output.error/m_output.nvalid << " " << m_output.nvalid << std::endl;

            m_mtx_pose.lock();
            geometry_msgs::PoseStamped pose_cur = m_current_pose;
            m_mtx_pose.unlock();
            yaw_cur  = tf2::getYaw(pose_cur.pose.orientation);
            offset_yaw = yaw_cur - ba;
            offset_yaw = normalize(offset_yaw);

            m_x = pose_cur.pose.position.x - bx;
            m_y = pose_cur.pose.position.y - by;
            
            offset_x = m_x * cos(ba) + m_y * sin(ba);
            offset_y = m_y * cos(ba) - m_x * sin(ba); 


            x       = offset_x;
            y       = offset_y;
            theda   = offset_yaw;
          
        }
        if(flag_init)
        {
            flag_init = false;
            error_x   = x;
            error_y   = y;
            error_yaw = theda;

        }

        e_dis = y;
        e_yaw = 0 - theda;
        w = m_kp_dis * e_dis + m_kd_dis * (e_dis - e_dis_pre) + m_ki_dis * e_dis_add + m_kp_yaw * e_yaw ;
        if(fabs(y) < 0.01 && x < 0.5)
        {
            std::cout << "********************************************************" << std::endl;
            std::cout << m_kp_dis * e_dis << "   " << m_kd_dis * (e_dis - e_dis_pre) << "   "<< m_ki_dis * e_dis_add  << std::endl;
            std::cout << m_kp_yaw * e_yaw  << "   " <<  m_kd_yaw * (e_yaw - e_yaw_pre) << "   "<< m_ki_yaw * e_yaw_add << std::endl;
            std::cout << "********************************************************" << std::endl;

            e_dis_add += e_dis;
            e_yaw_add += e_yaw;

            w = m_kp_dis * e_dis + m_kd_dis * (e_dis - e_dis_pre) + m_ki_dis * e_dis_add + m_kp_yaw * e_yaw + m_kd_yaw * (e_yaw - e_yaw_pre) + m_ki_yaw * e_yaw_add;
           



        }
        e_yaw_pre = e_yaw;

        e_dis_pre = e_dis;



        if(w > w_max)
        {
            w = w_max;
        }
        else if(w < -w_max)
        {
            w = -w_max;
        }

        std_msgs::Float64 tmp_dis ;
        tmp_dis.data = y;
        dis_pub.publish(tmp_dis);

       
        tmp_dis.data = theda;
        y_pub.publish(tmp_dis);



        if(x < STOP_DIS )
        {
            pubZeroVel();
            m_mtx.lock();
            sensor_msgs::LaserScan scan_copy = m_lastest_scan ;
            m_mtx.unlock();
            icpGetRelativePose(scan_copy,goal_scan);
            std::cout << " end:  "<< m_output.valid << "  "<< m_output.x[0] << "  "<< m_output.x[1] << "  "<< m_output.x[2]  << std::endl;
            std::cout << " init error:  "<< error_x << "  "<< error_y << "  "<< error_yaw << std::endl;
            
            std::cout << "arrival in dock station" << std::endl;
            m_charge_server->setSucceeded();    
            return;
        }

        geometry_msgs::Twist vel;
        vel.linear.y = 0;
        vel.linear.z = 0;
        vel.linear.x = -0.1;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = w;
        if(x < 0.1)
        {
            vel.linear.x = -0.05;

        }
        m_vel_pub.publish(vel);
    
    }

#endif
 
}


void dock_server::pubZeroVel()
{
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
    m_input.max_angular_correction_deg = 45.0;
    m_input.max_linear_correction = 0.5;
    m_input.max_correspondence_dist = 1.0;
    m_input.max_iterations = 40;
    m_input.epsilon_xy = 0.000001;
    m_input.epsilon_theta = 0.000001;
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
    m_input.outliers_maxPerc = 0.75;
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

