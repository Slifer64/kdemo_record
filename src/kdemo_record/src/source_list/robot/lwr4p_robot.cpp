#include <kdemo_record/robot/lwr4p_robot.h>

LWR4p_Robot::LWR4p_Robot()
{
  N_JOINTS = 7;

  jpos_low_lim = -arma::vec({170, 120, 170, 120, 170, 120, 170});
  jpos_upper_lim = arma::vec({170, 120, 170, 120, 170, 120, 170});

  // Initialize generic robot with the kuka-lwr model
  robot.reset(new lwr4p::Robot());
  // std::cout << "Robot created successfully!\n";

  std::string ft_sensor_ip = "192.168.2.1";
  // std::cout << "Initializing F/T sensor at ip: " << ft_sensor_ip << "\n";
  ftsensor.init(ft_sensor_ip.c_str());
  ftsensor.setTimeout(1.0);
  ftsensor.setBias();
}

LWR4p_Robot::~LWR4p_Robot()
{

}

void LWR4p_Robot::command()
{
  switch (this->getMode())
  {
    case Robot::Mode::FREEDRIVE:
      robot->setJointTorque(arma::vec().zeros(N_JOINTS));
      break;
    case Robot::Mode::IDLE:
      // do nothing for kuka
      break;
  }
}

void LWR4p_Robot::setMode(const Robot::Mode &mode)
{
  if (mode == this->getMode()) return;

  switch (mode){
    case FREEDRIVE:
      robot->setMode(lwr4p::Mode::TORQUE_CONTROL);
      break;
    case IDLE:
      robot->setMode(lwr4p::Mode::STOPPED);
      break;
  }
  this->mode = mode;
}

void LWR4p_Robot::stop()
{
  robot->setMode(lwr4p::Mode::STOPPED);
}
