/**
 * Copyright (C) 2016 AUTH-lwr
 */

#include <lwr4p/Robot.h>
#include <iostream>
#include <string>

#include <ros/package.h>

namespace lwr4p
{

  Robot::Robot(const char *path_to_FRI_init): N_JOINTS(7)
  {
    ext_stop = false;

    if (path_to_FRI_init == NULL)
    {
      std::string path = ros::package::getPath("lwr4p") + "/FRILibrary/980500-FRI-Driver.init";
      FRI.reset(new FastResearchInterface(path.c_str()));
    }
    else
      FRI.reset(new FastResearchInterface(path_to_FRI_init));

    cycle = FRI->GetFRICycleTime();
    this->mode = lwr4p::Mode::UNDEFINED;  // Dummy initialization before stopping controller
    stopController();  // Initially the robot is stopped
    startJointPositionController();

    // preallocate memory for jacobian
    jacob_temp = reinterpret_cast<float**>(malloc(sizeof(float *) * 6));
    for (size_t i = 0; i < 6; i++) {
      jacob_temp[i] = reinterpret_cast<float*>(malloc(sizeof(float) * 7));
    }
  }

  Robot::~Robot()
  {
    for (size_t i = 0; i < 6; i++) free(jacob_temp[i]);
    free(jacob_temp);
  }

  void Robot::setMode(lwr4p::Mode mode)
  {
    if (this->mode == mode) return;

    switch (mode)
    {
      case lwr4p::Mode::STOPPED:
        stopController();
        this->mode = lwr4p::Mode::STOPPED;
        break;
      case lwr4p::Mode::POSITION_CONTROL:
        if (this->mode != lwr4p::Mode::STOPPED) stopController();
        startJointPositionController();
        this->mode = lwr4p::Mode::POSITION_CONTROL;
        break;
      case lwr4p::Mode::TORQUE_CONTROL:
        if (this->mode != lwr4p::Mode::STOPPED) stopController();
        startJointTorqueController();
        this->mode = lwr4p::Mode::TORQUE_CONTROL;
        break;
      default: std::cerr << "Mode " << mode << " Not available" << std::endl;
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

  bool Robot::setJointsTrajectory(const arma::vec &input, double duration)
  {
    // setJntPosTrajTemplate(input, duration)
    // inital joint position values
    arma::vec q0 = arma::zeros<arma::vec>(N_JOINTS);
    arma::vec temp = arma::zeros<arma::vec>(N_JOINTS);
    for (int i = 0; i < N_JOINTS; i++) {
      temp(i) = input(i);
    }
    q0 = getJointPosition();
    // keep last known robot mode
    lwr4p::Mode prev_mode = mode;
    arma::vec qref = q0;
    // start conttroller
    setMode(lwr4p::Mode::POSITION_CONTROL);
    // initalize time
    double t = 0.0;
    // the main while
    while (t < duration)
    {
      if (!isOk()) return false;

      // waits for the next tick also
      FRI->WaitForKRCTick();
      // compute time now
      t += cycle;
      // update trajectory
      qref = get5thOrder(t, q0, temp, duration).col(0);
      // set joint positions
      setJointPosition(qref);
    }
    // reset last known robot mode
    setMode(prev_mode);

    return true;
  }

  void Robot::startJointPositionController()
  {
    // wait one tick
    FRI->WaitForKRCTick();
    std::cout << "[JointPosController::startController] Starting joint position control." << std::endl;
    int ResultValue = FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
    this->mode = lwr4p::Mode::POSITION_CONTROL;
    // if there s a problem
    if ((ResultValue != 0) && (ResultValue != EALREADY)) {
      std::cout << "[JointPosController::startController] "
                << "An error occurred during starting up the robot. I will stop controller." << std::endl;
      stopController();
      return;
    }
    std::cout << "[JointPosController::startController] " << "Finished" << std::endl;
  }

  void Robot::startJointTorqueController()
  {
    // wait one tick
    FRI->WaitForKRCTick();
    // temp variables
    static float stiffness[7];
    static float stiffnessCart[6];
    static float damping[7];
    static float dampingCart[6];
    static float torques[7];
    static float q[7];
    // put zeros everywhere
    for (int i = 0; i < 7; i++) {
      stiffness[i] = 0;
      damping[i] = 0;
      torques[i] = 0;
    }
    for (int i = 0; i < 6; i++) {
      stiffnessCart[i] = 0;
      dampingCart[i] = 0;
    }

    // set stiffness to zero
    FRI->SetCommandedJointStiffness(stiffness);
    // set stiffness to zero
    FRI->SetCommandedCartStiffness(stiffnessCart);
    // set damping to zero
    FRI->SetCommandedJointDamping(damping);
    // set damping to zero
    FRI->SetCommandedCartDamping(dampingCart);
    // set additional torques to zero
    FRI->SetCommandedJointTorques(torques);
    // set commanded joint positions to current
    FRI->GetCommandedJointPositions(q);
    FRI->SetCommandedJointPositions(q);
    std::cout << "[KukaTorqueController::startController] Starting torque control." << std::endl;
    int ResultValue = FRI->StartRobot(FastResearchInterface::JOINT_IMPEDANCE_CONTROL);
    // if there s a problem
    this->mode = lwr4p::Mode::TORQUE_CONTROL;
    if ((ResultValue != 0) && (ResultValue != EALREADY))
    {
      std::cout << "[KukaTorqueController::startController] "
                << "An error occurred during starting up the robot. I will stop controller." << std::endl;
      stopController();
      return;
    }

    std::cout << "[KukaTorqueController::startController] " << "Finished" << std::endl;
  }

  void Robot::stopController()
  {
    if (this->mode != lwr4p::Mode::STOPPED)
    {
      FRI->WaitForKRCTick();
      // printouts
      std::cout << "[KukaController::stopController] Stopping  control." << std::endl;

      static float pose[12];
      static float poseoff[12];

      static float q[7];
      static float qoff[7];
      static float torques[7];

      // set commanded joint positions to current commanded
      FRI->GetCommandedJointPositions(q);
      FRI->GetCommandedJointPositionOffsets(qoff);

      for (int i = 0; i < 7; i++)
      {
        q[i] += qoff[i];
        torques[i] = 0.0;
      }

      FRI->SetCommandedJointPositions(q);

      // set commanded pose  to current commanded
      FRI->GetCommandedCartPose(pose);
      FRI->GetCommandedCartPoseOffsets(poseoff);
      for (int i = 0; i < 12; i++)
      {
        pose[i] += poseoff[i];
      }
      FRI->SetCommandedCartPose(pose);

      // set joint torques to zero
      FRI->SetCommandedJointTorques(torques);

      std::cout << "StopRobot...\n";
      // call stanford command
      FRI->StopRobot();
      this->mode = lwr4p::Mode::STOPPED;
      std::cout << "Done!\n";
    }
    // lower the flag
    std::cout << "[KukaController::stopController] " << "Finished" << std::endl;
  }


}  // namespace lwr
