#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QMenu>
#include <QPushButton>
#include <QLabel>
#include <QAction>
#include <QWidget>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPalette>
#include <QColor>
#include <QMessageBox>
#include <QFileDialog>

#include <map>

#include "data_rec_dialog.h"
#include "data_plot_dialog.h"
#include "view_pose_dialog.h"
#include "view_jpos_dialog.h"
#include <kdemo_record/robot/robot.h>
#include "rec_data.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    enum Mode
    {
        FREEDRIVE,
        IDLE
    };

    MainWindow(const Robot *robot, const RecData *rec_data, QWidget *parent = 0);
    ~MainWindow();

    void setMode(const Mode &m);
    MainWindow::Mode getMode() const;
    QString getModeName() const;

    bool isRunning() const { return is_running; }
    bool record() const { return rec; }

    bool currentPoseAsStart() const { return current_pose_as_start(); }
    void resetCurrentPoseAsStart() { current_pose_as_start = false; }

    bool gotoStartPose() const { return goto_start_pose(); }
    void resetGotoStartPose() { goto_start_pose=false; }

    bool saveData() const { return save_data(); }
    void resetSaveData() { save_data=false; }
    std::string getSaveDataPath() const { return save_data_path; }

    bool clearData() const { return clear_data(); }
    void resetClearData() { clear_data=false; }

    bool record(DataType::ID id) const { return data_rec_dialog->rec[id]; }

signals:
    void errMsgSignal(const QString &msg);
    void infoMsgSignal(const QString &msg);
    void terminationSignal(const QString &msg);
    void modeChangedSignal();

private slots:
    void saveTriggered();
    void saveAsTriggered();
    void clearTriggered();
    void setRecordedData();
    void viewPoseTriggered();
    void viewJointsTriggered();
    void plotTriggered();
    void startRecording();
    void stopRecording();
    void gotoStartPosePressed();
    void setStartPosePressed();

    void showSentErrMsg(const QString &msg);
    void showSentInfoMsg(const QString &msg);
    void terminateApp(const QString &msg);
    void modeChanged();

private:
    QWidget *central_widget;
    QStatusBar *status_bar;

    std::map<Mode, QString> mode_name;

    Mode mode;

    bool rec;
    bool is_running;

    MtxFlag goto_start_pose;
    MtxFlag current_pose_as_start;
    MtxFlag save_data;
    MtxFlag clear_data;
    std::string save_data_path;
    std::string default_save_data_path;

    const Robot *robot;
    const RecData *rec_data;

    DataRecDialog *data_rec_dialog;
    DataPlotDialog *data_plot_dialog;
    ViewPoseDialog *view_pose_dialog;
    ViewJPosDialog *view_jpos_dialog;

    // ======  menus  ========
    QMenuBar *menu_bar;
    QMenu *file_menu;
    QMenu *edit_menu;
    QMenu *view_menu;

    // ======  actions  ========

    QAction *save_act;
    QAction *save_as_act;
    QAction *clear_act;
    QAction *set_rec_data_act;

    QAction *view_pose_act;
    QAction *view_joints_act;
    QAction *plot_act;

    // ======  widgets  ========
    QLabel *mode_label;
    QPushButton *freedrive_btn;
    QPushButton *idle_btn;

    QPushButton *save_btn;
    QPushButton *save_as_btn;
    QPushButton *clear_btn;

    QLabel *rec_label;
    QPushButton *start_btn;
    QPushButton *stop_btn;

    QPushButton *emergency_stop_btn;
    QPushButton *set_start_pose_btn;
    QPushButton *goto_start_pose_btn;
    QPushButton *view_jpos_btn;
    QPushButton *view_pose_btn;
    QPushButton *plot_btn;
    QPushButton *set_rec_data_btn;

    // ======  layouts  ========
    QVBoxLayout *mode_layout;
    QVBoxLayout *save_layout;
    QVBoxLayout *btns_layout;
    QGridLayout *rec_layout;
    QGridLayout *main_layout;

    // ======  functions  ========
    void createWidgets();
    void createLayouts();
    void createActions();
    void createMenus();
    void updateInterface();

    void closeEvent(QCloseEvent *event) override;
};

#endif // MAINWINDOW_H
