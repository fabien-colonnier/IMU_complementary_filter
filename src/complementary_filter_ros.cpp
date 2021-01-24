#include "complementary_filter/complementary_filter_ros.hpp"

namespace complementary_filter
{


estimator_ros::estimator_ros(ros::NodeHandle nh, ros::NodeHandle nh_private):
nh_(nh),
nh_priv_(nh_private)
{
      ROS_INFO_STREAM("complementary_filter ROS init function"); 
      std::string topic_imu("imu_topic");
      std::string topic_pose("poseGT_topic");

      /// setup subscribers and publishers
      imu_sub_ = nh_.subscribe(topic_imu, 1, &estimator_ros::imu_callback, this);
      pose_sub_ = nh_.subscribe(topic_pose, 1, &estimator_ros::pose_callback, this);
      reset_sub_ = nh_.subscribe("/reset_state", 1, &estimator_ros::reset_state_callback, this);

      //publish states position
      odom_pub_ = nh_priv_.advertise<geometry_msgs::PoseStamped>("odom", 10);

      //get param
      gain_cf _gain;
      Get_CF_param(nh_private, _gain); 

      //create object
      cf_ptr = std::make_unique<complementary_filter>(_gain);

      /********************* TO modify ****************************/
      ROS_INFO_STREAM("complementary_filter ROS initialized"); 
}

void estimator_ros::GetVector(ros::NodeHandle nh, std::string param_name,  Eigen::Vector3d& Xout)
{
	std::vector<float> yaml_list;

	if( nh.hasParam(param_name) )
      {
            nh.getParam(param_name, yaml_list);
            
            #ifdef VERBOSE
                  printf("yaml_list : ");
                  for(int j=0; j<7; j++)
                        printf("%f, ", yaml_list[j]);
            #endif

            Xout = Eigen::Vector3d(yaml_list[0], yaml_list[1], yaml_list[2]);

            ROS_INFO_STREAM("[" << param_name << "]  vector = [" 
                                << Xout(0) << ", " << Xout(1) << ", " << Xout(2) << "]");
      }
      else
      {
            ROS_INFO_STREAM("[" << param_name << "] has not been detected. Default [0,0,0] is applied. ");
            Xout = Eigen::Vector3d::Zero();
      }
}

void estimator_ros::Get_CF_param(ros::NodeHandle nh_private, gain_cf& p) 
{
      //get gain parameters
	std::string module = "CF_PARAM";
   
      GetVector(nh_private, "k_m", p.km);
      GetVector(nh_private, "k_a", p.ka);
      GetVector(nh_private, "k_b", p.kb);

}




void estimator_ros::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
      /* msg content 
      geometry_msgs/Quaternion orientation
      float64[9] orientation_covariance # Row major about x, y, z axes

      geometry_msgs/Vector3 angular_velocity
      float64[9] angular_velocity_covariance # Row major about x, y, z axes

      geometry_msgs/Vector3 linear_acceleration
      float64[9] linear_acceleration_covariance # Row major x, y z 
      */
      
      // ROS_INFO_STREAM("IMU callback ");   

      Eigen::Vector3d angular_vel;
      angular_vel << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
      Eigen::Vector3d linear_acc;
      linear_acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

      Eigen::Vector3d mag_ref;  //fictious measurement
      mag_ref << 1.0, 0.0, 0.0;

      cf_ptr->process_cf(msg->header.stamp.toSec(), linear_acc, angular_vel, mag_ref);
      Publish_State_est();

      //ROS_INFO_STREAM("IMU callback ends, ax=" << linear_acc(0) << ", ay=" << linear_acc(1) << ", az=" << linear_acc(2) );
}





void estimator_ros::pose_callback(const  nav_msgs::Odometry::ConstPtr& msg)
{
      //get pose from message
      Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
      Eigen::Quaterniond q_temp(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

      //ROS_INFO_STREAM( "robot_pos: " << position(0) << ", " << position(1) << ", " << position(2));

}

void estimator_ros::Publish_State_est()
{
      /* publish estimated state */
      geometry_msgs::PoseStamped odom_msg;
      Eigen::Quaterniond q_temp = cf_ptr->q_est;
      odom_msg.header.frame_id = "odom";
      odom_msg.header.stamp = ros::Time::now(); 
      odom_msg.pose.position.x = 0.0;
      odom_msg.pose.position.y = 0.0;
      odom_msg.pose.position.z = 0.0;
      odom_msg.pose.orientation.x = q_temp.x();
      odom_msg.pose.orientation.y = q_temp.y();
      odom_msg.pose.orientation.z = q_temp.z();
      odom_msg.pose.orientation.w = q_temp.w();

      odom_pub_.publish(odom_msg);
}

void estimator_ros::reset_state_callback(const std_msgs::Bool::ConstPtr& msg)
{
      if(msg->data)
      {
            //Init state object
            cf_ptr->reset();
      }
}

}
