#ifndef KINESTHETIC_DEMO_RECORD_H
#define KINESTHETIC_DEMO_RECORD_H

#include <vector>
#include <string>
#include <memory>
#include <armadillo>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <condition_variable>

#include <kdemo_record/robot/robot.h>

#include "rec_data.h"
#include "mainwindow.h"

class Semaphore
{
private:
  std::mutex mutex_;
  std::condition_variable condition_;
  unsigned long count_ = 0; // Initialized as locked.

public:
  void notify()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    ++count_;
    condition_.notify_one();
  }

  void wait()
  {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    while(!count_) // Handle spurious wake-ups.
      condition_.wait(lock);
    --count_;
  }

  bool try_wait()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    if(count_)
    {
      --count_;
      return true;
    }
    return false;
  }
};

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
