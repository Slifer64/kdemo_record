#include <kdemo_record/kdemo_record.h>


#include <QApplication>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

#include <kdemo_record/robot/lwr4p_robot.h>
#include <kdemo_record/robot/dummy_robot.h>

KDemoRecord::KDemoRecord()
{
  std::string robot_type="";

  ros::NodeHandle("~").getParam("robot_type",robot_type);
  // std::cerr << "=============> robot_type = " << robot_type << "\n";

  if (robot_type.compare("lwr4p")==0) robot.reset(new LWR4p_Robot);
  else if (robot_type.compare("dummy")==0) robot.reset(new DummyRobot);
  else
  {
    std::cerr << "Unsupported robot type...\n";
    exit(-1);
  }

  rec_data.reset(new RecData);

  // std::cerr << "=============> Launching GUI...\n";
  launchGUI();
  // std::cerr << "=============> GUI started!!!\n";
}

KDemoRecord::~KDemoRecord()
{

}

void KDemoRecord::launchGUI()
{
  std::thread([this]()
              {
                int argc = 0;
                char **argv = 0;
                QApplication app(argc, argv);
                this->gui = new MainWindow(this->robot.get(), this->rec_data.get());
                this->gui->show();
                this->start_sem.notify();
                app.exec();
                std::cerr << "[KDemoRecord::launchGUI]: Notifying!\n";
                this->finish_sem.notify();
                delete (this->gui); // must be destructed in this thread!
              }).detach();

  start_sem.wait(); // wait for gui to be initialized
}

void KDemoRecord::run()
{
  q_start = robot->getJointsPosition();

  while (gui->isRunning())
  {
    // =======> Check mode
    if (gui->getMode()==MainWindow::FREEDRIVE && robot->getMode()!=Robot::FREEDRIVE)
    {
      robot->setMode(Robot::FREEDRIVE);
      // std::cerr << "[KDemoRecord::run::FREEDRIVE]: gui->modeChangedSignal()\n";
      if (robot->isOk()) gui->modeChangedSignal(); // inform the gui that the robot's mode changed
    }
    else if (gui->getMode()==MainWindow::IDLE && robot->getMode()!=Robot::IDLE)
    {
      robot->setMode(Robot::IDLE);
      // std::cerr << "[KDemoRecord::run::IDLE]: gui->modeChangedSignal()\n";
      if (robot->isOk()) gui->modeChangedSignal(); // inform the gui that the robot's mode changed
    }

    // =======> Check if robot is ok
    if (!robot->isOk())
    {
      gui->terminateAppSignal("An error occured on the robot.\nThe application will terminate.");
      break;
    }

    if (gui->record()) record();

    if (gui->gotoStartPose())
    {
      if (!gotoStartPose()) gui->sendGotoStartPoseAck(false, err_msg.c_str());
      else gui->sendGotoStartPoseAck(true, "Reached start pose!");
    }

    if (gui->saveData())
    {
      if (! rec_data->saveData(gui->getSaveDataPath()) ) gui->sendSaveAck(false, rec_data->getErrMsg().c_str());
      else gui->sendSaveAck(true, "The recorded data were successfully saved!");
    }

    if (gui->clearData())
    {
      rec_data->clearData();
      gui->sendClearAck(true, "All recorded data were deleted!");
    }

    if (gui->currentPoseAsStart())
    {
      q_start = robot->getJointsPosition();
      gui->sendSetStartPoseAck(true, "Registered current pose as start!");
    }

    robot->update();
    // robot->command();
  }

  if (robot->isOk()) robot->stop();

  std::cerr << "[KDemoRecord::run]: Waiting to be notified...\n";
  finish_sem.wait(); // wait for gui to finish
  std::cerr << "[KDemoRecord::launchGUI]: Got notification!\n";

}

bool KDemoRecord::gotoStartPose()
{
  arma::vec q = robot->getJointsPosition();
  double duration = arma::max(arma::abs(q-q_start))*8.0/3.14159;
  return robot->setJointsTrajectory(q_start, duration);
}

void KDemoRecord::record()
{
  if (gui->record(DataType::TOOL_POS)) rec_data->addData(DataType::TOOL_POS, robot->getTaskPosition());
  if (gui->record(DataType::TOOL_ORIENT)) rec_data->addData(DataType::TOOL_ORIENT, robot->getTaskOrientation());
  if (gui->record(DataType::TOOL_FORCE)) rec_data->addData(DataType::TOOL_FORCE, robot->getTaskForce());
  if (gui->record(DataType::TOOL_TORQUE)) rec_data->addData(DataType::TOOL_TORQUE, robot->getTaskTorque());
  if (gui->record(DataType::JOINT_POS)) rec_data->addData(DataType::JOINT_POS, robot->getJointsPosition());
  if (gui->record(DataType::JOINT_TORQUE)) rec_data->addData(DataType::JOINT_TORQUE, robot->getJointsTorque());
  if (gui->record(DataType::JACOBIAN)) rec_data->addData(DataType::JACOBIAN, robot->getJacobian());
}
