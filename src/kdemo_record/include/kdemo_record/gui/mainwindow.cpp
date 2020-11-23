#include "mainwindow.h"
#include "utils.h"

#include <ros/package.h>

#include <QDebug>

MainWindow::MainWindow(const Robot *robot, const RecData *rec_data, QWidget *parent): QMainWindow(parent)
{
  this->robot = robot;
  this->rec_data = rec_data;

  mode_name[FREEDRIVE] = "FREEDRIVE";
  mode_name[IDLE] = "IDLE";

  //this->resize(400,350);
  this->setWindowTitle("Kinesthetic Demo Recorder");

  //QToolBar *tool_bar = new QToolBar(this);
  //this->addToolBar(tool_bar);
  status_bar = new QStatusBar(this);
  this->setStatusBar(status_bar);

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  // ====================================

  createWidgets();

  createLayouts();

  createActions();

  createMenus();

  createConnections();

  rec = false;
  goto_start_pose = false;
  current_pose_as_start = false;
  save_data = false;
  clear_data = false;
  is_running = true;

  project_path = ros::package::getPath("kdemo_record") + "/";

  default_save_data_path = project_path + "../../kdemo_matlab/data/";

  mode = FREEDRIVE;
  setMode(IDLE);
}

MainWindow::~MainWindow()
{

}

MainWindow::Mode MainWindow::getMode() const
{
  return mode;
}

QString MainWindow::getModeName() const
{
  return (mode_name.find(getMode()))->second;
}

void MainWindow::setMode(const Mode &m)
{
  if (getMode() == m) return;

  mode = m;

  idle_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
  freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");

  this->setEnabled(false);

  // updateInterface();
}

void MainWindow::updateInterface()
{
  switch (getMode())
  {
    case FREEDRIVE:
      idle_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
      freedrive_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");

      start_btn->setEnabled(true);
      stop_btn->setEnabled(true);

      break;

    case IDLE:
      stopRecordingPressed();
      freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
      idle_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0) }");

      start_btn->setEnabled(false);
      stop_btn->setEnabled(false);

      break;
  }
}

void MainWindow::createActions()
{
  save_act = new QAction(tr("&Save"), this);
  save_act->setShortcut(QKeySequence("Ctrl+S"));
  save_act->setStatusTip(tr("Saves the recorded data to default location."));

  save_as_act = new QAction(tr("Save As..."), this);
  save_as_act->setShortcut(QKeySequence("Shift+Ctrl+S"));
  save_as_act->setStatusTip(tr("Saves the recorded data to a user specified path."));

  clear_act = new QAction(tr("&Clear"), this);
  clear_act->setShortcut(QKeySequence("Ctrl+C"));
  clear_act->setStatusTip(tr("Clears the recorded data."));

  set_rec_data_act = new QAction(tr("Set recorded data"), this);
  // set_rec_data_act->setShortcut(QKeySequence("Ctrl+C"));
  set_rec_data_act->setStatusTip(tr("Sets which data to record."));

  view_pose_act = new QAction(tr("View pose"), this);
  view_pose_act->setStatusTip(tr("Opens a window displaying the robot's end-effector pose."));

  view_joints_act = new QAction(tr("View joints"), this);
  view_joints_act->setStatusTip(tr("Opens a window with sliders displaying the robot's joints position."));

  plot_act = new QAction(tr("plot"), this);
  plot_act->setStatusTip(tr("Opens a window where the user can choose to plot recorded data."));
}

void MainWindow::createConnections()
{
  QObject::connect( save_btn, &QPushButton::clicked, this, &MainWindow::saveTriggered );
  QObject::connect( save_act, &QAction::triggered, this, &MainWindow::saveTriggered );

  QObject::connect( save_as_btn, &QPushButton::clicked, this, &MainWindow::saveAsTriggered );
  QObject::connect( save_as_act, &QAction::triggered, this, &MainWindow::saveAsTriggered );

  QObject::connect( clear_btn, &QPushButton::clicked, this, &MainWindow::clearTriggered );
  QObject::connect( clear_act, &QAction::triggered, this, &MainWindow::clearTriggered );

  QObject::connect( set_rec_data_btn, &QPushButton::clicked, this, &MainWindow::setRecordedDataPressed );
  QObject::connect( set_rec_data_act, &QAction::triggered, this, &MainWindow::setRecordedDataPressed );

  QObject::connect( view_pose_btn, &QPushButton::clicked, this, &MainWindow::viewPoseTriggered );
  QObject::connect( view_pose_act, &QAction::triggered, this, &MainWindow::viewPoseTriggered );

  QObject::connect( view_jpos_btn, &QPushButton::clicked, this, &MainWindow::viewJointsTriggered );
  QObject::connect( view_joints_act, &QAction::triggered, this, &MainWindow::viewJointsTriggered );

  QObject::connect( plot_btn, &QPushButton::clicked, this, &MainWindow::plotTriggered );
  QObject::connect( plot_act, &QAction::triggered, this, &MainWindow::plotTriggered );

  QObject::connect( freedrive_btn, &QPushButton::clicked, [this](){ this->setMode(FREEDRIVE);} );
  QObject::connect( idle_btn, &QPushButton::clicked, [this](){ this->setMode(IDLE);} );

  QObject::connect( start_btn, &QPushButton::clicked, this, &MainWindow::startRecordingPressed );
  QObject::connect( stop_btn, &QPushButton::clicked, this, &MainWindow::stopRecordingPressed );

  QObject::connect( set_start_pose_btn, &QPushButton::clicked, this, &MainWindow::setStartPosePressed );
  QObject::connect( goto_start_pose_btn, &QPushButton::clicked, this, &MainWindow::gotoStartPosePressed );

  QObject::connect( emergency_stop_btn, &QPushButton::clicked, [this](){ const_cast<Robot *>(this->robot)->setExternalStop(true); this->setEnabled(false); } );

  QObject::connect( this, SIGNAL(terminateAppSignal(const QString &)), this, SLOT(terminateAppSlot(const QString &)) );
  QObject::connect( this, SIGNAL(modeChangedSignal()), this, SLOT(modeChangedSlot()) );
  QObject::connect( this, SIGNAL(gotoStartPoseAckSignal(bool , const QString &)), this, SLOT(gotoStartPoseAckSlot(bool , const QString &)) );
  QObject::connect( this, SIGNAL(saveAckSignal(bool , const QString &)), this, SLOT(saveAckSlot(bool , const QString &)) );
  QObject::connect( this, SIGNAL(clearAckSignal(bool , const QString &)), this, SLOT(clearAckSlot(bool , const QString &)) );
  QObject::connect( this, SIGNAL(setStartPoseAckSignal(bool , const QString &)), this, SLOT(startPoseAckSlot(bool , const QString &)) );
}

void MainWindow::createMenus()
{
  menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  file_menu = menu_bar->addMenu(tr("&File"));
  file_menu->addAction(save_act);
  file_menu->addAction(save_as_act);
  file_menu->addAction(clear_act);

  edit_menu = menu_bar->addMenu(tr("&Edit"));
  edit_menu->addAction(set_rec_data_act);

  view_menu = menu_bar->addMenu(tr("&View"));
  view_menu->addAction(view_pose_act);
  view_menu->addAction(view_joints_act);
  view_menu->addSeparator();
  view_menu->addAction(plot_act);
}

void MainWindow::createWidgets()
{
  QFont font1("Ubuntu", 13, QFont::DemiBold);
  QFont font2("Ubuntu", 15, QFont::DemiBold);

  data_rec_dialog = new DataRecDialog(this);
  data_plot_dialog = new DataPlotDialog(rec_data, this);
  view_pose_dialog = new ViewPoseDialog(std::bind(&Robot::getTaskPosition, robot), std::bind(&Robot::getTaskOrientation, robot), this);
  view_jpos_dialog = new ViewJPosDialog(robot->getJointsLowerLimits(), robot->getJointsUpperLimits(), std::bind(&Robot::getJointsPosition, robot), this);
  view_jpos_dialog->setJointNames(robot->getJointNames());

  mode_label = new QLabel;
  mode_label->setText("Robot mode");
  mode_label->setFont(font2);
  mode_label->setStyleSheet("background-color: rgb(250,250,250); color: rgb(0,0,200); font: 75 15pt \"FreeSans\";");
  mode_label->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Fixed);
  mode_label->setAlignment(Qt::AlignCenter);

  freedrive_btn = new QPushButton;
  freedrive_btn->setText("FREEDRIVE");
  freedrive_btn->setFont(QFont("Ubuntu", 13, QFont::DemiBold));

  idle_btn = new QPushButton;
  idle_btn->setText("IDLE");
  idle_btn->setFont(QFont("Ubuntu", 13, QFont::DemiBold));

  // ===============================

  save_btn = new QPushButton;
  save_btn->setText("save data...");
  save_btn->setFont(font1);

  save_as_btn = new QPushButton;
  save_as_btn->setText("save data as...");
  save_as_btn->setFont(font1);

  clear_btn = new QPushButton;
  clear_btn->setText("clear data");
  clear_btn->setFont(font1);

  // ===============================

  rec_label = new QLabel;
  rec_label->setText("Recording");
  rec_label->setStyleSheet("background-color: rgb(250,250,250); color: rgb(0,0,200);");
  rec_label->setFont(QFont("Ubuntu", 16, QFont::Bold));
  rec_label->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Fixed);
  rec_label->setAlignment(Qt::AlignCenter);

  start_btn = new QPushButton;
  start_btn->setText("  start");
  start_btn->setFont(font2);
  start_btn->setIcon(QIcon(":/start_rec_icon"));
  start_btn->setIconSize(QSize(50,50));

  stop_btn = new QPushButton;
  stop_btn->setText("  stop");
  stop_btn->setFont(QFont("Ubuntu", 15, QFont::DemiBold));
  stop_btn->setIcon(QIcon(":/stop_rec_icon"));
  stop_btn->setIconSize(QSize(50,50));

  // ===============================

  emergency_stop_btn = new QPushButton;
  emergency_stop_btn->setText("EMERGENCY\n     STOP");
  emergency_stop_btn->setMinimumSize(80,80);
  emergency_stop_btn->setIcon(QIcon(":/panic_button_icon"));
  emergency_stop_btn->setIconSize(QSize(50,50));
  emergency_stop_btn->setStyleSheet("color:rgb(255,0,0); background-color:rgba(200, 200, 200, 100);");
  emergency_stop_btn->setFont(QFont("Ubuntu",13,QFont::DemiBold));

  set_start_pose_btn = new QPushButton;
  set_start_pose_btn->setText("set start pose");
  set_start_pose_btn->setFont(font1);

  goto_start_pose_btn = new QPushButton;
  goto_start_pose_btn->setText("goto start pose");
  goto_start_pose_btn->setFont(font1);

  view_jpos_btn = new QPushButton;
  view_jpos_btn->setText("view joint pos");
  view_jpos_btn->setFont(font1);

  view_pose_btn = new QPushButton;
  view_pose_btn->setText("view cart pose");
  view_pose_btn->setFont(font1);

  plot_btn = new QPushButton;
  plot_btn->setText("plot");
  plot_btn->setFont(font1);

  set_rec_data_btn = new QPushButton;
  set_rec_data_btn->setText("set recorded data");
  set_rec_data_btn->setFont(font1);
}

void MainWindow::createLayouts()
{
  mode_layout = new QVBoxLayout;
  mode_layout->addWidget(mode_label);
  mode_layout->addWidget(freedrive_btn);
  mode_layout->addWidget(idle_btn);
  mode_layout->addStretch();

  save_layout = new QVBoxLayout;
  save_layout->addWidget(save_btn);
  save_layout->addWidget(save_as_btn);
  save_layout->addWidget(clear_btn);
  save_layout->addStretch();

  btns_layout = new QVBoxLayout;
  btns_layout->addWidget(emergency_stop_btn);
  btns_layout->addWidget(set_start_pose_btn);
  btns_layout->addWidget(goto_start_pose_btn);
  btns_layout->addWidget(view_jpos_btn);
  btns_layout->addWidget(view_pose_btn);
  btns_layout->addWidget(plot_btn);
  btns_layout->addWidget(set_rec_data_btn);
  btns_layout->addStretch();

  rec_layout = new QGridLayout;
  rec_layout->addWidget(rec_label,0,0,1,2);
  rec_layout->addWidget(start_btn,1,0);
  rec_layout->addWidget(stop_btn,1,1);
  rec_layout->addItem(new QSpacerItem(0,50,QSizePolicy::Preferred,QSizePolicy::Fixed),2,0,1,2);

  main_layout = new QGridLayout(central_widget);
  main_layout->setSizeConstraint(QLayout::SetFixedSize);
  main_layout->addLayout(mode_layout,0,0);
  main_layout->addItem(new QSpacerItem(30,0,QSizePolicy::Fixed,QSizePolicy::Preferred),0,1);
  main_layout->addLayout(save_layout,0,2);
  main_layout->addItem(new QSpacerItem(20,0,QSizePolicy::Fixed,QSizePolicy::Preferred),0,3);
  main_layout->addLayout(btns_layout,0,4,3,1);
  main_layout->addItem(new QSpacerItem(20,10,QSizePolicy::Preferred,QSizePolicy::Fixed),1,0);
  main_layout->addLayout(rec_layout,2,0,1,3);
//    //main_layout->addItem(new QSpacerItem(30,30,QSizePolicy::Fixed,QSizePolicy::Preferred),2,0);
}

void MainWindow::saveTriggered()
{
  // if (getMode() != IDLE)
  // {
  //     showWarningMsg("Mode must be set to \"IDLE\" to save the recorded data.");
  //     return;
  // }

  save_data_path = default_save_data_path + "/recorded_data.bin";
  save_data = true;
  updateInterfaceOnSaveData();
}

void MainWindow::saveAsTriggered()
{
  QString save_as_data_path = QFileDialog::getSaveFileName(this, tr("Save Recorded Data"), default_save_data_path.c_str(), "Binary files (*.bin)");
  if (save_as_data_path.isEmpty()) return;

  save_data_path = save_as_data_path.toStdString();
  save_data = true;
  updateInterfaceOnSaveData();
}

void MainWindow::saveAckSlot(bool success, const QString &msg)
{
  save_data = false;

  success ? showInfoMsg(msg) : showErrorMsg(msg);

  update_gui_sem.notify();

  updateInterfaceOnSaveData();
}

void MainWindow::updateInterfaceOnSaveData()
{
  bool set = !save_data();

  start_btn->setEnabled(set);
  stop_btn->setEnabled(set);

  save_btn->setEnabled(set);
  save_act->setEnabled(set);

  save_as_btn->setEnabled(set);
  save_as_act->setEnabled(set);

  clear_btn->setEnabled(set);
  clear_act->setEnabled(set);
}

void MainWindow::clearTriggered()
{
  clear_data = true;
  updateInterfaceOnClearData();
}

void MainWindow::clearAckSlot(bool success, const QString &msg)
{
  clear_data = false;

  success ? showInfoMsg(msg) : showErrorMsg(msg);

  update_gui_sem.notify();

  updateInterfaceOnClearData();
}

void MainWindow::updateInterfaceOnClearData()
{
  bool set = !clear_data();

  start_btn->setEnabled(set);
  stop_btn->setEnabled(set);

  save_btn->setEnabled(set);
  save_act->setEnabled(set);

  save_as_btn->setEnabled(set);
  save_as_act->setEnabled(set);

  clear_btn->setEnabled(set);
  clear_act->setEnabled(set);

  plot_btn->setEnabled(set);
  plot_act->setEnabled(set);
}

void MainWindow::setRecordedDataPressed()
{
  data_rec_dialog->launch();
}

void MainWindow::viewPoseTriggered()
{
  view_pose_dialog->launch();
}

void MainWindow::viewJointsTriggered()
{
  view_jpos_dialog->launch();
}

void MainWindow::plotTriggered()
{
  // if (getMode() != IDLE)
  // {
  //   showWarningMsg("The mode must be \"IDLE\" to plot recorded data.");
  //   return;
  // }

  data_plot_dialog->launch();
}

void MainWindow::startRecordingPressed()
{
  if (rec) return;

  // if (getMode() != FREEDRIVE)
  // {
  //     if (showQuestionMsg("The mode must be changed to \"FREEDRIVE\".\nDo you wish to change it?") == QMessageBox::Yes) setMode(FREEDRIVE);
  //     return;
  // }

  if (!rec_data->isEmpty())
  {
    if (showQuestionMsg("All previously recorded data will be erased.\nDo you wish to continue?") != QMessageBox::Yes) return;
    clear_data = true;
  }

  rec = true;

  start_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");

  goto_start_pose_btn->setEnabled(false);

  plot_btn->setEnabled(false);
  plot_act->setEnabled(false);

  set_rec_data_btn->setEnabled(false);
  set_rec_data_act->setEnabled(false);

  save_btn->setEnabled(false);
  save_act->setEnabled(false);

  save_as_btn->setEnabled(false);
  save_as_act->setEnabled(false);

  clear_btn->setEnabled(false);
  clear_act->setEnabled(false);
}

void MainWindow::stopRecordingPressed()
{
  rec = false;
  start_btn->setStyleSheet(stop_btn->styleSheet());

  goto_start_pose_btn->setEnabled(true);

  plot_btn->setEnabled(true);
  plot_act->setEnabled(true);

  set_rec_data_btn->setEnabled(true);
  set_rec_data_act->setEnabled(true);

  save_btn->setEnabled(true);
  save_act->setEnabled(true);

  save_as_btn->setEnabled(true);
  save_as_act->setEnabled(true);

  clear_btn->setEnabled(true);
  clear_act->setEnabled(true);
}

void MainWindow::gotoStartPosePressed()
{
  // if (record())
  // {
  //     if (showQuestionMsg("Recording is on! Do you want to stop it?") != QMessageBox::Yes) return;
  //     else setMode(IDLE);
  // }

  // if (getMode()==FREEDRIVE) setMode(IDLE);

  goto_start_pose = true;
  updateInterfaceOnGotoStartPose();
}

void MainWindow::gotoStartPoseAckSlot(bool success, const QString &msg)
{
  goto_start_pose = false;

  success ? showInfoMsg(msg) : showErrorMsg(msg);

  update_gui_sem.notify();

  updateInterfaceOnGotoStartPose();
}

void MainWindow::updateInterfaceOnGotoStartPose()
{
  bool set = !goto_start_pose();

  freedrive_btn->setEnabled(set);
  idle_btn->setEnabled(set);
  if (set)
  {
    if (getMode()==FREEDRIVE) freedrive_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
    else if (getMode()==IDLE) idle_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
  }
  else
  {
    freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
    idle_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
  }

  start_btn->setEnabled(set);
  stop_btn->setEnabled(set);

  goto_start_pose_btn->setEnabled(set);
  set_start_pose_btn->setEnabled(set);

  plot_btn->setEnabled(set);
  plot_act->setEnabled(set);

  set_rec_data_btn->setEnabled(set);
  set_rec_data_act->setEnabled(set);

  save_btn->setEnabled(set);
  save_act->setEnabled(set);

  save_as_btn->setEnabled(set);
  save_as_act->setEnabled(set);

  clear_btn->setEnabled(set);
  clear_act->setEnabled(set);
}

void MainWindow::setStartPosePressed()
{
  current_pose_as_start = true;
  set_start_pose_btn->setEnabled(false);
}

void MainWindow::startPoseAckSlot(bool success, const QString &msg)
{
  current_pose_as_start = false;

  success ? showInfoMsg(msg) : showErrorMsg(msg);

  update_gui_sem.notify();

  set_start_pose_btn->setEnabled(true);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  is_running = false;
  const_cast<Robot *>(robot)->setExternalStop(true);
  update_gui_sem.notify(); // unlock possible waits...
  QMainWindow::closeEvent(event);
}

void MainWindow::terminateAppSlot(const QString &msg)
{
  showErrorMsg(msg);
  this->close();
}

void MainWindow::modeChangedSlot()
{
  this->setEnabled(true);
  updateInterface();
  showInfoMsg("Mode changed to \"" + getModeName() + "\"\n");
}
