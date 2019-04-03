#ifndef LWR4p_ROBOT_H
#define LWR4p_ROBOT_H

#include <kdemo_record/robot/robot.h>

#include <lwr4p/Robot.h>
#include <ati_sensor/ft_sensor.h>

class LWR4p_Robot: public Robot
{
public:
  LWR4p_Robot();
  ~LWR4p_Robot();

  int getNumOfJoints() const
  { return N_JOINTS; }

  arma::vec getTaskPosition() const
  { return robot->getTaskPosition(); }

  arma::vec getTaskOrientation() const
  {
    arma::vec task_orient(4);
    arma::mat R = robot->getTaskOrientation();
    task_orient = rotm2quat(R);
    return task_orient;
  }

  arma::vec getTaskWrench() const
  {
    static double measurements[6];
    uint32_t rdt(0),ft(0);
    (const_cast<ati::FTSensor *>(&ftsensor))->getMeasurements(measurements,rdt,ft);
    //ftsensor.getMeasurements(measurements,rdt,ft);

    arma::vec Fext(6);
    Fext(0) = measurements[0];
    Fext(1) = measurements[1];
    Fext(2) = measurements[2];
    Fext(3) = measurements[3];
    Fext(4) = measurements[4];
    Fext(5) = measurements[5];

    arma::mat R = robot->getTaskOrientation();
    Fext.subvec(0,2) = R*Fext.subvec(0,2);
    Fext.subvec(3,5) = R*Fext.subvec(3,5);

    // Fext = robot->getExternalWrench();
    // Fext = -Fext;

    return Fext;
  }

  arma::vec getJointsPosition() const
  { return robot->getJointPosition(); }

  arma::vec getJointsTorque() const
  { return robot->getJointExternalTorque(); }

  arma::mat getJacobian() const
  { return robot->getRobotJacobian(); }

  void update()
  { robot->waitNextCycle(); }

  void command();

  void stop();

  void setMode(const Robot::Mode &mode);

  double getCtrlCycle() const
  { return robot->getControlCycle(); }

  bool isOk() const
  { return (robot->isOk()); }

  bool setJointsTrajectory(const arma::vec &qT, double duration)
  { return robot->setJointsTrajectory(qT, duration); }

  void setExternalStop(bool set) { robot->setExternalStop(set); }

private:
  std::shared_ptr<lwr4p::Robot> robot;
  ati::FTSensor ftsensor;
};

#endif // LWR4p_ROBOT_H
