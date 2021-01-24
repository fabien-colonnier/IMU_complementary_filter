#pragma once

#include <iostream>   //cout screen output
#include <math.h>
#include <Eigen/Dense>

//debugging purposes
#define DEBUGGING // compute manually the vector cross product
#define DEBUG_NORM_ERROR 0.0005


// gravity values
#define G_EARTH 9.80
#define G_EARTH_DOUBLE 2*G_EARTH
#define G_EARTH_SQUARED G_EARTH*G_EARTH

// roll pitch and yaw
#define ROLL       0
#define PITCH      1
#define YAW        2

namespace complementary_filter
{

//Convert an error rotation vector into a quaternion 
void vec2quat(const Eigen::Vector3d& vec, Eigen::Quaterniond& quat);

//Convert a quaternion into an error rotation vector
void quat2vec(const Eigen::Quaterniond& quat, Eigen::Ref<Eigen::Vector3d>& vec);

}
