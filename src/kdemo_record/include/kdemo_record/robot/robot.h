#ifndef ROBOT_H
#define ROBOT_H

#include <cstdlib>
#include <exception>
#include <vector>
#include <cstring>
#include <armadillo>
#include <Eigen/Dense>

#include <mutex>
#include <condition_variable>
#include <thread>

class Semaphore
{
private:
  std::mutex mutex_;
  std::condition_variable condition_;
  // unsigned long count_ = 0; // Initialized as locked.
  bool count_ = false;  // Initialized as locked.

public:
  void notify()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    // ++count_;
    count_ = true;
    condition_.notify_one();
  }

  void wait()
  {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    while(!count_) // Handle spurious wake-ups.
      condition_.wait(lock);
    // --count_;
    count_ = false;
  }

  bool try_wait()
  {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    if(count_)
    {
      // --count_;
      count_ = false;
      return true;
    }
    return false;
  }
};

template<typename T>
class MtxVar
{
public:
  T get() const { std::unique_lock<std::mutex> lck(*(const_cast<std::mutex *>(&cmd_mtx))); return cmd; }
  void set(const T &new_cmd) { std::unique_lock<std::mutex> lck(cmd_mtx); cmd=new_cmd; }
private:
  std::mutex cmd_mtx;
  T cmd;
};

class Robot
{
public:
  /** The control modes that can be applied to the robot. */
  enum Mode
  {
    FREEDRIVE, // freedrive mode (or gravity compensation)
    IDLE, // robot is idle and doesn't move
    STOPPED, // the robot stops
  };

  Robot();
  ~Robot();

  Robot::Mode getMode() const;
  std::string getModeName() const;

  virtual int getNumOfJoints() const = 0;

  virtual arma::vec getTaskPosition() const = 0;
  virtual arma::vec getTaskOrientation() const = 0;
  virtual arma::vec getTaskForce() const = 0;
  virtual arma::vec getTaskTorque() const = 0;
  virtual arma::vec getTaskWrench() const = 0;

  virtual arma::vec getJointsPosition() const = 0;
  virtual arma::vec getJointsTorque() const = 0;
  virtual arma::mat getJacobian() const = 0;

  /** Updates the robot state (position, forces, velocities etc.) by reading them
   *  from the actual hardware. Must be called once in each control cycle.
   */
  virtual void update() = 0;

  virtual void stop() = 0;

  virtual void setMode(const Robot::Mode &mode) = 0;

  virtual double getCtrlCycle() const = 0;
  virtual bool isOk() const = 0;

  virtual void commandThread() = 0;

  virtual bool setJointsTrajectory(const arma::vec &qT, double duration) = 0;

  virtual arma::vec getJointsLowerLimits() const = 0;
  virtual arma::vec getJointsUpperLimits() const = 0;

  virtual std::vector<std::string> getJointNames() const = 0;

  virtual void setExternalStop(bool set) = 0;

protected:

  Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm) const;
  arma::vec rotm2quat(const arma::mat &rotm) const ;

  MtxVar<Mode> mode; // current mode
  std::vector<std::string> mode_name; ///< robot's control mode name

  MtxVar<arma::vec> jpos_cmd;
  MtxVar<arma::vec> jtorque_cmd;

  Semaphore KRC_tick;
};

#endif // ROBOT_H
