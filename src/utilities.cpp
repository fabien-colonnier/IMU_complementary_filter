
#include <complementary_filter/utilities.hpp>

namespace complementary_filter
{

void vec2quat(const Eigen::Vector3d& vec, Eigen::Quaterniond& quat)
{
  //test value approximation of the conversion
  double theta_squared = 0;
  for(int i=0; i<3;i++)
    theta_squared += vec(i)*vec(i);
  
  double theta = sqrt(theta_squared);	
  double C = cos(theta/2);
  double S;
  if(theta == 0)
    S = 1;
  else
    S = sin(theta/2)/theta;
    
  quat = Eigen::Quaterniond(C, S*vec(0), S*vec(1), S*vec(2));
}

//Convert a quaternion into a vector
void quat2vec(const Eigen::Quaterniond& quat, Eigen::Ref<Eigen::Vector3d>& vec)
{
  //logarithmic map

	double inv_mag;
	
	if(quat.w()==1.0)
	{
		vec(0) = 0;
		vec(1) = 0;
		vec(2) = 0;
	}
	else
	{
		inv_mag = 2*acos(quat.w())/sqrt(1-quat.w()*quat.w());
    vec(0) = inv_mag * quat.x();
		vec(1) = inv_mag * quat.y();
		vec(2) = inv_mag * quat.z();
	}
}


}