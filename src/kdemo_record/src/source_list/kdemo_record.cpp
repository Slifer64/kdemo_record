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
    robot->update();

    // =======> Check mode
    if (gui->getMode()==MainWindow::FREEDRIVE && robot->getMode()!=Robot::FREEDRIVE)
    {
      robot->setMode(Robot::FREEDRIVE);
      if (robot->isOk()) gui->modeChangedSignal(); // inform the gui that the robot's mode changed
    }
    else if (gui->getMode()==MainWindow::IDLE && robot->getMode()!=Robot::IDLE)
    {
      robot->setMode(Robot::IDLE);
      if (robot->isOk()) gui->modeChangedSignal(); // inform the gui that the robot's mode changed
    }

    // =======> Check if robot is ok
    if (!robot->isOk())
    {
      gui->terminationSignal("An error occured on the robot.\nThe application will terminate.");
      break;
    }

    if (gui->record()) record();

    if (gui->gotoStartPose())
    {
      if (!gotoStartPose()) gui->errMsgSignal(err_msg.c_str());
      else gui->infoMsgSignal("Reached start pose!");
      gui->resetGotoStartPose();
    }

    if (gui->saveData())
    {
      if (! rec_data->saveData(gui->getSaveDataPath()) ) gui->errMsgSignal(rec_data->getErrMsg().c_str());
      else gui->infoMsgSignal("The recorded data were successfully saved!");
      gui->resetSaveData();
    }

    if (gui->clearData())
    {
      rec_data->clearData();
      gui->resetClearData();
      gui->infoMsgSignal("All recorded data were deleted!");
    }

    if (gui->currentPoseAsStart())
    {
      q_start = robot->getJointsPosition();
      gui->resetCurrentPoseAsStart();
      gui->infoMsgSignal("Registered current pose as start!");
    }

    robot->command();
  }

  if (robot->isOk()) robot->stop();

  std::cerr << "[KDemoRecord::run]: Waiting to be notified...\n";
  finish_sem.wait(); // wait for gui to finish
  std::cerr << "[KDemoRecord::launchGUI]: Got notification!\n";

}

bool KDemoRecord::gotoStartPose()
{
  arma::vec q = robot->getJointsPosition();
  double duration = arma::max(arma::abs(q-q_start))*7.0/3.14;
  robot->setJointsTrajectory(q_start, duration);

  return true;
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
