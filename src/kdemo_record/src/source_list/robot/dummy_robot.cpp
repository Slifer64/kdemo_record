#include <kdemo_record/robot/dummy_robot.h>

#include <QString>
#include <thread>
#include <chrono>

DummyRobot::DummyRobot()
{
  stop_signal = false;

  N_JOINTS = 7;

  jpos = arma::vec().zeros(N_JOINTS);
  jtorque = arma::vec().zeros(N_JOINTS);
  jacob = arma::mat().zeros(6,N_JOINTS);
  jpos_low_lim = -arma::vec({170, 120, 170, 120, 170, 120, 170});
  jpos_upper_lim = arma::vec({170, 120, 170, 120, 170, 120, 170});

  pos = arma::vec({0.2, 0.5, 0.3});
  quat = arma::vec({0.2, 0.5, 0.3, 0.5});
  quat = quat/arma::norm(quat);
  force = arma::vec({0.0, 0.0, 0.0});
  torque = arma::vec({0.0, 0.0, 0.0});

  jnames.resize(N_JOINTS);
  for (int i=0;i<N_JOINTS;i++) jnames[i] = QString("lwr_joint_" + QString::number(i+1)).toStdString();

  jpos_target = (2*arma::vec().randu(N_JOINTS)-1) % jpos_upper_lim;
  target_reached = arma::uvec().zeros(N_JOINTS);

  t = 0;
  Ts = 0.005;
}

DummyRobot::~DummyRobot()
{

}

void DummyRobot::update()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(Ts*1000)));
}

void DummyRobot::command()
{
  if (getMode() == Robot::FREEDRIVE)
  {
    t += Ts;

    for (int i=0;i<N_JOINTS;i++)
    {
      if (target_reached(i))
      {
        jpos_target(i) = arma::as_scalar(2*arma::vec().randu(1)-1) * jpos_upper_lim(i);
      }
    }

    double pi = 3.14;

    double djpos = 0.15;
    jpos = jpos + djpos * arma::sign(jpos_target - jpos);
    target_reached = (arma::abs(jpos - jpos_target) < djpos);

    pos(0) = 0.33 * std::cos(jpos(0)) + 0.5 * std::sin(jpos(1)) + 0.17 * std::cos(jpos(2));
    pos(1) = 0.4 * std::cos(jpos(3)) + 0.4 * std::sin(jpos(4)) + 0.2 * std::cos(jpos(5));
    pos(2) = 0.2 * std::cos(jpos(0)) + 0.3 * std::sin(jpos(4)) + 0.5 * std::cos(jpos(6));

    quat(0) = 0.33 * std::cos(jpos(1)) + 0.5 * std::sin(jpos(2)) + 0.17 * std::cos(jpos(3));
    quat(1) = 0.4 * std::cos(jpos(4)) + 0.4 * std::sin(jpos(2)) + 0.2 * std::cos(jpos(6));
    quat(2) = 0.2 * std::cos(jpos(2)) + 0.3 * std::sin(jpos(3)) + 0.5 * std::cos(jpos(4));
    quat(3) = 0.35 * std::cos(jpos(6)) + 0.3 * std::sin(jpos(5)) + 0.35 * std::cos(jpos(2));
    quat = quat / (arma::norm(quat) + 1e-16);

    force(0) = 2 * std::sin(2 * pi * 1.5 * t);
    force(1) = 3.5 * std::cos(2 * pi * 1 * 1.2 * t);
    force(2) = 1.5 * std::sin(2 * pi * 1.8 * t);

    torque(0) = 0.8 * std::cos(2 * pi * 2.5 * t);
    torque(1) = 1.1 * std::sin(2 * pi * 1 * 2.2 * t);
    torque(2) = 0.6 * std::cos(2 * pi * 2.8 * t);

    jtorque = arma::vec({0.5, 0.4, 0.33, 0.25, 0.6, 0.45, 0.38})*std::sin(2 * pi * 1.5 * t);
  }
}

void DummyRobot::setMode(const Robot::Mode &mode)
{
  if (mode == this->getMode()) return;
  this->mode = mode;
  t = 0;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void DummyRobot::stop()
{
  this->mode = Robot::IDLE;
}

bool DummyRobot::setJointsTrajectory(const arma::vec &qT, double duration)
{
  bool err_occured = false;
  while (arma::norm(jpos-qT)<0.1)
  {
    jpos = jpos + 0.01*(qT-jpos);
    update();
    if (!isOk() || stopSignalSent())
    {
      err_occured = true;
      break;
    }
  }

  return !err_occured;
}