#ifndef KINESTHETIC_DEMO_RECORD_H
#define KINESTHETIC_DEMO_RECORD_H

#include <vector>
#include <string>
#include <memory>
#include <armadillo>
#include <fstream>
#include <iomanip>

#include <kdemo_record/robot/robot.h>

#include "rec_data.h"
#include "mainwindow.h"

class KDemoRecord
{
public:
    KDemoRecord();
    ~KDemoRecord();

    void run();

private:
    std::shared_ptr<Robot> robot;
    std::shared_ptr<RecData> rec_data;
    MainWindow *gui;

    void launchGUI();
    void record();
    bool gotoStartPose();

    arma::vec q_start;
    std::string err_msg;

    Semaphore start_sem;
    Semaphore finish_sem;
};

#endif // KINESTHETIC_DEMO_RECORD_H
