#ifndef DUMMY_ROBOT_H
#define DUMMY_ROBOT_H

#include <kdemo_record/robot/robot.h>
#include <armadillo>
#include <mutex>

class DummyRobot: public Robot
{
public:
  DummyRobot();
  ~DummyRobot();

  int getNumOfJoints() const
  { return N_JOINTS; }

  arma::vec getTaskPosition() const
  { return pos; }

  arma::vec getTaskOrientation() const
  { return quat; }

  arma::vec getTaskWrench() const
  { return arma::join_vert(force,torque); }

  arma::vec getJointsPosition() const
  { return jpos; }

  arma::vec getJointsTorque() const
  { return jtorque; }

  arma::mat getJacobian() const
  { return jacob; }

  void update();

  void command();

  void stop();

  void setMode(const Robot::Mode &mode);

  double getCtrlCycle() const
  { return 0.005; }

  bool isOk() const
  { return !externalStop(); }

  bool setJointsTrajectory(const arma::vec &qT, double duration);

  void setExternalStop(bool set) { ext_stop = set; }

  bool externalStop() const { return ext_stop; }

private:

  bool ext_stop;

  double t;
  double Ts;

  arma::vec jpos;
  arma::vec jtorque;
  arma::mat jacob;
  arma::vec pos;
  arma::vec quat;
  arma::vec force;
  arma::vec torque;

  arma::vec jpos_target;
  arma::uvec target_reached;

};

#endif // DUMMY_ROBOT_H
