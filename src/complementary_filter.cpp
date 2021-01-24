#include "complementary_filter/complementary_filter.hpp"


namespace complementary_filter
{
complementary_filter::complementary_filter(gain_cf gain_in):
gain(gain_in)
{
  q_est = Eigen::Quaterniond(1,0,0,0);
  b_est = Eigen::Vector3d::Zero();

  magneto_ref = Eigen::Vector3d::Zero();
  t_old = -1;
}

void complementary_filter::process_cf(double t,
                                 const Eigen::Vector3d& a_meas,
                                 const Eigen::Vector3d& omega_meas,
                                 const Eigen::Vector3d& magneto_meas)
{  
  // std::cout << "run process" << std::endl;

  if(t_old==-1)
  {
    t_old = t;
    return;
  } 
  
  double dt = t - t_old;
  t_old = t;

  /* compute estimate accelerometer */
  // estimation of IMU vector using Q_hat (estimated quaternion): a_hat = Inv(Q_hat) * g
  Eigen::Vector3d g;  //gravity reference
  g << 0,0,G_EARTH;
  Eigen::Vector3d a_est = q_est.inverse() * g;

  #ifdef DEBUGGING
    /*
      Inv(Q_hat) = [q0 -q1 -q2 -q3]'
      Inv(Q_hat)*g = [1-2(q2^2+q3^2)     2(q1q2+q0q3)     2(q1q3-q0q2)]     [0]
                     [  2(q1q2-q0q3)   1-2(q1^2+q3^2))    2(q2q3+q0q1)]  *  [0]
                     [  2(q1q3+q0q2)     2(q2q3-q0q1)   1-2(q1^2+q2^2)]     [g]
    */
    Eigen::Vector3d a_hat;
    a_hat(0) = G_EARTH_DOUBLE*(q_est.x()*q_est.z() - q_est.w()*q_est.y() );
    a_hat(1) = G_EARTH_DOUBLE*(q_est.y()*q_est.z() + q_est.w()*q_est.x() );
    a_hat(2) = G_EARTH*(1-2.0*(q_est.x()*q_est.x() + q_est.y()*q_est.y()) );

    Eigen::Vector3d error_a;
    error_a = a_est - a_hat;
    if(error_a.norm()>DEBUG_NORM_ERROR)
    {
      std::cout << "[ERROR] a_est wrong estimation" << std::endl;
      std::cout << "\t a_est: \n" << a_est <<  std::endl;
      std::cout << "\t a_hat: \n" << a_hat <<  std::endl;
    }
  #endif

  /* compute estimate magnetometer */
  // estimation of magnetometer vector using Q_hat (estimated quaternion): m_hat = Inv(Q_hat) * m_ref
  Eigen::Vector3d magneto_ref;
  magneto_ref << 1,0,0;
  Eigen::Vector3d m_est = q_est.inverse() * magneto_ref;

  #ifdef DEBUGGING
    /*
      Inv(Q_hat) = [q0 -q1 -q2 -q3]'
      Inv(Q_hat)*m_ref = [1-2(q2^2+q3^2)     2(q1q2+q0q3)     2(q1q3-q0q2)]     [1]
                         [  2(q1q2-q0q3)   1-2(q1^2+q3^2))    2(q2q3+q0q1)]  *  [0]
                         [  2(q1q3+q0q2)     2(q2q3-q0q1)   1-2(q1^2+q2^2)]     [0]
    */
    Eigen::Vector3d m_hat;
    m_hat(0) = (1-2.0*(q_est.y()*q_est.y() + q_est.z()*q_est.z()));
    m_hat(1) = 2.0*(q_est.x()*q_est.y() - q_est.w()*q_est.z());
    m_hat(2) = 2.0*(q_est.x()*q_est.z() + q_est.w()*q_est.y());

    Eigen::Vector3d error_m;
    error_m = m_est - m_hat;
    if(error_m.norm()>DEBUG_NORM_ERROR)
    {
      std::cout << "[ERROR] m_est wrong estimation" << std::endl;
      std::cout << "\t m_est: \n" << m_est <<  std::endl;
      std::cout << "\t m_hat: \n" << m_hat <<  std::endl;
    }
  #endif

  // compute the error between a_est and a_meas
  // cross(a_hat, a_bar)
  Eigen::Vector3d a_err = a_est.cross(a_meas);

  #ifdef DEBUGGING
    Eigen::Vector3d a_tilde;
    a_tilde(0) = a_hat(1)*a_meas(2) - a_hat(2)*a_meas(1);
    a_tilde(1) = a_hat(2)*a_meas(0) - a_hat(0)*a_meas(2);
    a_tilde(2) = a_hat(0)*a_meas(1) - a_hat(1)*a_meas(0);

    error_a = a_err - a_tilde;
    if(error_a.norm()>DEBUG_NORM_ERROR)
    {
      std::cout << "[ERROR] a cross product wrong estimation" << std::endl;
      std::cout << "\t a_err: \n" << a_err <<  std::endl;
      std::cout << "\t a_tilde: \n" << a_tilde <<  std::endl;
    }
  #endif

  // compute the error between m_est and a_tilde
  // cross(m_hat, m_bar)
  Eigen::Vector3d m_err = m_est.cross(magneto_meas);

  #ifdef DEBUGGING
    Eigen::Vector3d m_tilde;
    m_tilde(0) = m_hat(1)*magneto_meas(2) - m_hat(2)*magneto_meas(1);
    m_tilde(1) = m_hat(2)*magneto_meas(0) - m_hat(0)*magneto_meas(2);
    m_tilde(2) = m_hat(0)*magneto_meas(1) - m_hat(1)*magneto_meas(0);

    error_m = m_err - m_tilde;
    if(error_m.norm()>DEBUG_NORM_ERROR)
    {
      std::cout << "[ERROR] m cross product wrong estimation" << std::endl;
      std::cout << "\t m_err: \n" << m_err <<  std::endl;
      std::cout << "\t m_tilde: \n" << m_tilde <<  std::endl;
    }
  #endif

  /* complementary filter equations */
  // Compute the debiased rotation speed
  Eigen::Vector3d omega_est = omega_meas - b_est;
  
  // calculate the correction to apply to the quaternion
  Eigen::Vector3d alpha;  // <=> -omega meas in eq 32 of Mahony et al. 2007
  for(int ii=0; ii<3; ++ii)
    alpha(ii) = gain.ka(ii) * a_err(ii)/(G_EARTH_SQUARED) + gain.km(ii) * m_err(ii);
  
  // Corrected pure rotation speed for integration
  Eigen::Vector3d omega_corr = omega_est - alpha;

  // Bias derivative
  Eigen::Vector3d bdot_est;
  for(int ii=0; ii<3; ++ii){
    bdot_est(ii) = gain.kb(ii) * alpha(ii);
  }
  
  // Bias integration
  b_est += dt * bdot_est;


  // Quaternion integration
  #ifdef DEBUGGING
    Eigen::Quaterniond q_corr = Eigen::Quaterniond(0, dt*omega_corr(0), dt*omega_corr(1), dt*omega_corr(2));
    Eigen::Quaterniond dq_est = q_est*q_corr;

    std::array<double, 4> dQ_hat;
    std::array<double, 4> Q_corr = {0, omega_corr(0), omega_corr(1), omega_corr(2)};

    // Quaternion derivative: dQ_hat = dt*(Q_hat*Q_corr)
    dQ_hat[0] = dt*(q_est.w()*Q_corr[0] - (q_est.x()*Q_corr[1] + q_est.y()*Q_corr[2] + q_est.z()*Q_corr[3]));
    dQ_hat[1] = dt*(q_est.w()*Q_corr[1] + Q_corr[0]*q_est.x() + q_est.y()*Q_corr[3] - q_est.z()*Q_corr[2]);
    dQ_hat[2] = dt*(q_est.w()*Q_corr[2] + Q_corr[0]*q_est.y() + q_est.z()*Q_corr[1] - q_est.x()*Q_corr[3]);
    dQ_hat[3] = dt*(q_est.w()*Q_corr[3] + Q_corr[0]*q_est.z() + q_est.x()*Q_corr[2] - q_est.y()*Q_corr[1]);

    if(dQ_hat[0]-dq_est.w() > DEBUG_NORM_ERROR || dQ_hat[1]-dq_est.x() > DEBUG_NORM_ERROR ||
       dQ_hat[2]-dq_est.y() > DEBUG_NORM_ERROR || dQ_hat[3]-dq_est.z() > DEBUG_NORM_ERROR )
    {
      std::cout << "[ERROR] m dQ wrong estimation" << std::endl;
      std::cout << "\t dQ_hat: " << dQ_hat[0] << ", " << dQ_hat[1] << ", " << dQ_hat[2] << ", " << dQ_hat[3] << std::endl;
      std::cout << "\t dq_est: \n" << dq_est.w() << ", " << dq_est.x() << ", " << dq_est.y() << ", " << dq_est.z() <<  std::endl;
    }
  #endif

  //  dq/dt = 1/2 . q . ω
  //  to integrate the estimated quaternion according to the angular speed corrected omega_corr
  //    q_(n+1) = q_n * q{ω_n .∆t} 
  //    where  q{ω_n . ∆t} = Exp(ω_n.∆t) 
  //                       = [ cos( ||ω|| ∆t/2 ), ω/||ω|| . sin( ||ω|| ∆t/2 )]^T

  Eigen::Quaterniond quat_delta;
	vec2quat(dt*omega_corr, quat_delta);
	q_est = q_est * quat_delta;
  q_est.normalize();

/*
  https://math.stackexchange.com/questions/1693067/differences-between-quaternion-integration-methods

  OLDER VERSION

  //     dQ_hat = 0.5*(Q_hat*Q_corr)
  //     Q_hat[i] += dQ_hat[i]*dt;: 
  Eigen::Quaterniond q_corr = Eigen::Quaterniond(0, omega_corr(0), omega_corr(1), omega_corr(2));

  Eigen::Quaterniond dQ = q_est*q_corr; 
  q_est.w() += dt*0.5*dQ.w();
  q_est.x() += dt*0.5*dQ.x();
  q_est.y() += dt*0.5*dQ.y();
  q_est.z() += dt*0.5*dQ.z();
  q_est.normalize();
*/

}

void complementary_filter::reset()
{
  q_est = Eigen::Quaterniond(1,0,0,0);
  //b_est = Eigen::Vector3d::Zero();
}

}