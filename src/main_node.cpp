#include "complementary_filter/complementary_filter_ros.hpp"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "complementary_filter");

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");


  ROS_INFO_STREAM("Start complementary_filter node");
	complementary_filter::estimator_ros node_obj(nh, nh_private);

	ros::spin();
	
	return 0;
}

