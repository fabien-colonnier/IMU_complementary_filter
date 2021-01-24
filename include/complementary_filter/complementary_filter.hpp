#pragma once

#include <complementary_filter/utilities.hpp>


namespace complementary_filter
{

struct gain_cf
{
  Eigen::Vector3d ka;    //gain for accelerometer
  Eigen::Vector3d kb;    //gain for gyro
  Eigen::Vector3d km;    //gain for magnetometer 
};



class complementary_filter{
  public:

    Eigen::Quaterniond q_est;  //estimated quaternion
    Eigen::Vector3d b_est;     //estimated bias
    
    /* constructor */
    complementary_filter(gain_cf gain_in);

    /* processing function */
    void process_cf(double t, 
               const Eigen::Vector3d& a_meas,
               const Eigen::Vector3d& omega_meas,
               const Eigen::Vector3d& magneto_meas);

    void reset();

  private:
    Eigen::Vector3d magneto_ref; //indication of the North,  TODO initialize properly if available 

    gain_cf gain;  //param of the complementary filter
    double t_old;  //time of last update
  

};



}