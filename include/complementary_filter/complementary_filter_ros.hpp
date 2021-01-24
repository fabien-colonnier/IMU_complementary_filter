#pragma once

/* ROS standard includes */
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>


#include <nav_msgs/Odometry.h>

/* simulator includes */


/* Project related includes */
#include <complementary_filter/complementary_filter.hpp>
#include <complementary_filter/utilities.hpp>


namespace complementary_filter
{

/******* acquisition of parameter function ******************************/
// to get parameters from yaml files
template<typename T>
bool GetParam(const ros::NodeHandle& nh, const::std::string& where_str, const::std::string& key, T& val, const T& default_val)
{
  nh.param(key, val, default_val);
  ROS_INFO_STREAM("[" << where_str << "] Param " << key << " : " << val);
  return true;
};

class estimator_ros
{
  private:
    //ROS variables
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    //suscribers
    ros::Subscriber imu_sub_;   
  
    ros::Subscriber pose_sub_;
    ros::Subscriber reset_sub_;

    //publisher estimated states
    ros::Publisher odom_pub_;


    //variables
    std::unique_ptr<complementary_filter> cf_ptr;
    Eigen::Vector3d a_meas; //accelerometer measurement (x y z)
    Eigen::Vector3d omega_meas; //gyro measurement  (wx wy wz)

    //functions
    void Get_CF_param(ros::NodeHandle nh_private, gain_cf& p); 
    void GetVector(ros::NodeHandle nh, std::string param_name,  Eigen::Vector3d& Xout);
    
    void Publish_State_est();    

    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void pose_callback(const  nav_msgs::Odometry::ConstPtr& msg);
    //void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void reset_state_callback(const std_msgs::Bool::ConstPtr& msg);

  public:
    estimator_ros(ros::NodeHandle nh, ros::NodeHandle nh_private);
};


}
