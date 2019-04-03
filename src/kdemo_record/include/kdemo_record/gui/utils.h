#ifndef UTILS_H
#define UTILS_H

#include <QString>
#include <QMessageBox>
#include <QLineEdit>
#include <QPushButton>

#include <mutex>

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

class MtxFlag
{
public:
    MtxFlag() { flag = false; }
    MtxFlag& operator=(bool s) { set(s); return *this; }
    bool operator()() const { return get(); }

    bool get() const { return flag; }
    void set(bool s) { std::unique_lock<std::mutex> lck(mtx); flag=s; }
private:
    bool flag;
    std::mutex mtx;
};

#endif // UTILS_H
