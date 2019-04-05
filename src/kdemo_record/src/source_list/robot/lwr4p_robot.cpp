#include <kdemo_record/robot/lwr4p_robot.h>

LWR4p_Robot::LWR4p_Robot()
{
  N_JOINTS = 7;

  jpos_low_lim = -arma::vec({170, 120, 170, 120, 170, 120, 170});
  jpos_upper_lim = arma::vec({170, 120, 170, 120, 170, 120, 170});

  jnames.resize(N_JOINTS);
  jnames[0] = "lwr_joint_1";
  for (int i=1;i<N_JOINTS;i++)
  {
    jnames[i] = jnames[i-1];
    jnames[i].back()++;
  }

  // Initialize generic robot with the kuka-lwr model
  // std::cerr << "=======> Creating robot...\n";
  robot.reset(new lwr4p::Robot());
  // std::cerr << "=======> Robot created successfully!\n";

  std::string ft_sensor_ip = "192.168.2.1";
  // std::cerr << "Initializing F/T sensor at ip: " << ft_sensor_ip << "\n";
  ftsensor.init(ft_sensor_ip.c_str());
  ftsensor.setTimeout(1.0);
  ftsensor.setBias();

  mode.set(Robot::STOPPED);
  cmd_mode.set(mode.get());
  jpos_cmd.set(robot->getJointPosition());

  std::thread(&LWR4p_Robot::commandThread,this).detach();
}

LWR4p_Robot::~LWR4p_Robot()
{

}

void LWR4p_Robot::setMode(const Robot::Mode &mode)
{
  if (mode == cmd_mode.get()) return;

  cmd_mode.set(mode);
  mode_change.wait(); // wait to get notification from commandThread
}

void LWR4p_Robot::commandThread()
{
  while (isOk())
  {
    Mode new_mode = cmd_mode.get();
    // check if we have to switch mode
    if (new_mode != mode.get())
    {
      switch (new_mode)
      {
        case FREEDRIVE:
          robot->setMode(lwr4p::Mode::TORQUE_CONTROL);
          jtorque_cmd.set(arma::vec().zeros(N_JOINTS));
          break;
        case IDLE:
          robot->setMode(lwr4p::Mode::POSITION_CONTROL);
          jpos_cmd.set(robot->getJointPosition());
          break;
        case STOPPED:
          robot->setMode(lwr4p::Mode::POSITION_CONTROL);
          robot->setMode(lwr4p::Mode::STOPPED);
          robot->setExternalStop(true);
          mode.set(new_mode);
          mode_change.notify(); // unblock in case wait was called from another thread
          KRC_tick.notify(); // unblock in case wait was called from another thread
          return;
      }
      mode.set(new_mode);
      mode_change.notify();
    }

    // send command according to current mode
    switch (mode.get())
    {
      case Robot::Mode::FREEDRIVE:
        robot->setJointTorque(jtorque_cmd.get());
        break;
      case Robot::Mode::IDLE:
        robot->setJointPosition(jpos_cmd.get());
        break;
    }

    // sync with KRC
    robot->waitNextCycle();
    KRC_tick.notify();
  }

}

arma::mat get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime)
{
  arma::mat retTemp = arma::zeros<arma::mat>(p0.n_rows, 3);

  if (t < 0)
  {
    // before start
    retTemp.col(0) = p0;
  }
  else if (t > totalTime)
  {
    // after the end
    retTemp.col(0) = pT;
  }
  else
  {
    // somewhere betweeen ...
    // position
    retTemp.col(0) = p0 +
                     (pT - p0) * (10 * pow(t / totalTime, 3) -
                     15 * pow(t / totalTime, 4) +
                     6 * pow(t / totalTime, 5));
    // vecolity
    retTemp.col(1) = (pT - p0) * (30 * pow(t, 2) / pow(totalTime, 3) -
                     60 * pow(t, 3) / pow(totalTime, 4) +
                     30 * pow(t, 4) / pow(totalTime, 5));
    // acceleration
    retTemp.col(2) = (pT - p0) * (60 * t / pow(totalTime, 3) -
                     180 * pow(t, 2) / pow(totalTime, 4) +
                     120 * pow(t, 3) / pow(totalTime, 5));
  }

  // return vector
  return retTemp;
}

bool LWR4p_Robot::setJointsTrajectory(const arma::vec &qT, double duration)
{
  // setJntPosTrajTemplate(input, duration)
  // inital joint position values
  arma::vec q0 = arma::zeros<arma::vec>(N_JOINTS);
  arma::vec temp = arma::zeros<arma::vec>(N_JOINTS);
  for (int i = 0; i < N_JOINTS; i++) {
    temp(i) = qT(i);
  }
  q0 = robot->getJointPosition();
  // keep last known robot mode
  Robot::Mode prev_mode = this->getMode();
  arma::vec qref = q0;
  // start controller
  this->setMode(Robot::IDLE);
  // robot->setMode(lwr4p::Mode::POSITION_CONTROL);
  // initalize time
  double t = 0.0;
  // the main while
  while (t < duration)
  {
    if (!robot->isOk()) return false;

    // compute time now
    t += getCtrlCycle();
    // update trajectory
    qref = get5thOrder(t, q0, temp, duration).col(0);
    // set joint positions
    jpos_cmd.set(qref);
    //setJointPosition(qref);

    // waits for the next tick
    KRC_tick.wait();
  }
  // reset last known robot mode
  this->setMode(prev_mode);

  return true;
}

void LWR4p_Robot::stop()
{
  setMode(Robot::STOPPED);
}
