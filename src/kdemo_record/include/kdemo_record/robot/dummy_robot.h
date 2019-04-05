#ifndef DUMMY_ROBOT_H
#define DUMMY_ROBOT_H

#include <kdemo_record/robot/robot.h>
#include <armadillo>
#include <mutex>
#include <thread>
#include <chrono>

class DummyRobot: public Robot
{
public:
  DummyRobot();
  ~DummyRobot();

  void commandThread();

  int getNumOfJoints() const
  { return N_JOINTS; }

  std::string getErrMsg() const
  {
    return "N/A";
  }

  arma::vec getTaskPosition() const
  { return pos; }

  arma::vec getTaskOrientation() const
  { return quat; }

  arma::vec getTaskForce() const
  { return getTaskWrench().subvec(0,2); }

  arma::vec getTaskTorque() const
  { return getTaskWrench().subvec(3,5); }

  arma::vec getTaskWrench() const
  { return arma::join_vert(force,torque); }

  arma::vec getJointsPosition() const
  { return jpos; }

  arma::vec getJointsTorque() const
  { return jtorque; }

  arma::mat getJacobian() const
  { return jacob; }

  void update()
  {
    // std::cerr << "[DummyRobot::update]: <KRC_tick>: Waiting for notification...\n";;
    KRC_tick.wait();
    // std::cerr << "[DummyRobot::update]: <KRC_tick>: Got notification!\n";
}

  arma::vec getJointsLowerLimits() const
  { return jpos_low_lim; }

  arma::vec getJointsUpperLimits() const
  { return jpos_upper_lim; }

  void stop();

  void setMode(const Robot::Mode &mode);

  double getCtrlCycle() const
  { return Ts; }

  bool isOk() const
  { return !ext_stop; }

  bool setJointsTrajectory(const arma::vec &qT, double duration);

  void setExternalStop(bool set) { ext_stop = set; }

  std::vector<std::string> getJointNames() const
  { return jnames; }

private:

  void setJointTorque(const arma::vec &jtorque);
  void setJointPosition(const arma::vec &jpos);

  double t;
  double Ts;

  bool ext_stop;

  MtxVar<Mode> cmd_mode;
  Semaphore mode_change;

  int N_JOINTS;

  arma::vec jpos;
  arma::vec jtorque;
  arma::mat jacob;
  arma::vec pos;
  arma::vec quat;
  arma::vec force;
  arma::vec torque;

  arma::vec jpos_target;
  arma::uvec target_reached;

  arma::vec jpos_low_lim;
  arma::vec jpos_upper_lim;

  std::vector<std::string> jnames;

};

#endif // DUMMY_ROBOT_H
