#ifndef ROBOT_H
#define ROBOT_H

#include <cstdlib>
#include <exception>
#include <vector>
#include <cstring>
#include <armadillo>
#include <Eigen/Dense>

class Robot
{
public:
  /** The control modes that can be applied to the robot. */
  enum Mode
  {
    FREEDRIVE, // freedrive mode (or gravity compensation)
    IDLE, // robot is idle and doesn't move
  };

  Robot();
  ~Robot();

  virtual int getNumOfJoints() const = 0;

  virtual arma::vec getTaskPosition() const = 0;
  virtual arma::vec getTaskOrientation() const = 0;
  arma::vec getTaskForce() const { return getTaskWrench().subvec(0,2); }
  arma::vec getTaskTorque() const { return getTaskWrench().subvec(3,5); }
  virtual arma::vec getTaskWrench() const = 0;

  virtual arma::vec getJointsPosition() const = 0;
  virtual arma::vec getJointsTorque() const = 0;
  virtual arma::mat getJacobian() const = 0;

  /** Updates the robot state (position, forces, velocities etc.) by reading them
   *  from the actual hardware. Must be called once in each control cycle.
   */
  virtual void update() = 0;

  /** Sends a command to the robot according to the current control mode.
   *  Must be called once in each control cycle.
   */
  virtual void command() = 0;

  virtual void stop() = 0;

  virtual void setMode(const Robot::Mode &mode) = 0;

  virtual double getCtrlCycle() const = 0;
  virtual bool isOk() const = 0;

  /** Moves the robot to the specified joint positions within the specified duration.
   *  @param[in] qT Desired joint position.
   *  @param[in] duration Desired movement duration.
   */
  virtual bool setJointsTrajectory(const arma::vec &qT, double duration) = 0;

  Robot::Mode getMode() const;
  std::string getModeName() const;

  arma::vec getJointsLowerLimits() const;
  arma::vec getJointsUpperLimits() const;

  std::vector<std::string> getJointNames() const;

  virtual void setExternalStop(bool set) = 0;
protected:

  Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm) const;
  arma::vec rotm2quat(const arma::mat &rotm) const ;

  Mode mode; ///< robot's control mode
  std::vector<std::string> mode_name; ///< robot's control mode name

  int N_JOINTS;

  arma::vec jpos_low_lim;
  arma::vec jpos_upper_lim;

  std::vector<std::string> jnames;
};

#endif // ROBOT_H
