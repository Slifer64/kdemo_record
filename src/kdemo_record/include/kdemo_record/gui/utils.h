#ifndef UTILS_H
#define UTILS_H

#include <QString>
#include <QMessageBox>
#include <QLineEdit>
#include <QPushButton>

#include <mutex>
#include <condition_variable>

class DataType
{
public:
    enum ID
    {
        TOOL_POS = 0,
        TOOL_ORIENT = 1,
        TOOL_FORCE = 2,
        TOOL_TORQUE = 3,
        JOINT_POS = 4,
        JOINT_TORQUE = 5,
        JACOBIAN = 6,
        // NEW_DATA_TYPE = ..., and increase below the total number of types
        NUMBER_OF_DATA_TYPES = 7 // stores the total number of data types
    };

    static int getNumberOfDataTypes() { return DataType::NUMBER_OF_DATA_TYPES; }
    static std::string getDataTypeName(DataType::ID id) { return DataType::name[id]; }

private:
    constexpr static const char *name[] = {"TOOL_POS", "TOOL_ORIENT", "TOOL_FORCE", "TOOL_TORQUE", "JOINT_POS", "JOINT_TORQUE", "JACOBIAN"};
};

void showErrorMsg(const QString &msg);
void showWarningMsg(const QString &msg);
int showQuestionMsg(const QString &msg);
void showInfoMsg(const QString &msg);

class MyLineEdit: public QLineEdit
{
public:
    MyLineEdit(QWidget *parent=0):QLineEdit(parent) { size_hint = QSize(60,30); }

    void setSizeHint(int w, int h) { size_hint.setWidth(w); size_hint.setHeight(h); }
    QSize sizeHint() const override { return size_hint; }
private:
    QSize size_hint;
};

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
  MtxVar() { }
  MtxVar& operator=(const T &val) { set(val); return *this; }
  bool operator()() const { return get(); }
  T get() const { std::unique_lock<std::mutex> lck(*(const_cast<std::mutex *>(&var_mtx))); return var; }
  void set(const T &val) { std::unique_lock<std::mutex> lck(var_mtx); var=val; }
private:
  std::mutex var_mtx;
  T var;
};

#endif // UTILS_H
