#include <kdemo_record/robot/robot.h>

Robot::Robot()
{
  mode_name.resize(2);
  mode_name[0] = "FREEDRIVE";
  mode_name[1] = "IDLE";
}

Robot::~Robot()
{

}


Robot::Mode Robot::getMode() const
{
  return this->mode;
}

std::string Robot::getModeName() const
{
  return mode_name[getMode()];
}

arma::vec Robot::getJointsLowerLimits() const
{
  return jpos_low_lim;
}

arma::vec Robot::getJointsUpperLimits() const
{
  return jpos_upper_lim;
}

std::vector<std::string> Robot::getJointNames() const
{
  return jnames;
}

Eigen::Vector4d Robot::rotm2quat(Eigen::Matrix3d rotm) const
{
    Eigen::Quaternion<double> temp_quat(rotm);
    Eigen::Vector4d quat;
    quat << temp_quat.w(), temp_quat.x(), temp_quat.y(), temp_quat.z();

    quat = quat * (2*(quat(0)>=0)-1); // to avoid discontinuities

    return quat;
}

arma::vec Robot::rotm2quat(const arma::mat &rotm) const
{
  arma::vec quat(4);

  Eigen::Map<const Eigen::Matrix3d> rotm_wrapper(rotm.memptr());
  Eigen::Map<Eigen::Vector4d> quat_wrapper(quat.memptr());
  quat_wrapper = rotm2quat(rotm_wrapper);

  return quat;
}
