#ifndef DATA_RECORDING_DIALOG_H
#define DATA_RECORDING_DIALOG_H

#include <QDialog>
#include <QCheckBox>
#include <QPushButton>
#include <QLabel>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>

#include "utils.h"

class DataRecDialog : public QDialog
{
    Q_OBJECT

public:
    DataRecDialog(QWidget *parent = 0);
    ~DataRecDialog();

    void launch();

    std::vector<bool> rec;

private slots:
    void okBtnPressed();
    void cancelBtnPressed();

private:
    std::vector<QCheckBox *> chkbox;

    void addCheckBox(const char *label);

    void closeEvent(QCloseEvent *event) override;
};

#endif // DATA_RECORDING_DIALOG_H
