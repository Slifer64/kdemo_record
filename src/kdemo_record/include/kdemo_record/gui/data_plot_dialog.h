#ifndef DATA_PLOT_DIALOG_H
#define DATA_PLOT_DIALOG_H

#include <QDialog>
#include <QCheckBox>
#include <QPushButton>
#include <QLabel>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>

#include <vector>

#include "utils.h"
#include "rec_data.h"
#include "qcustomplot.h"

class PlotDialog : public QDialog
{
public:
    enum Color
    {
        BLUE = 0,        //   0   0   255
        GREEN = 1,       //   0  255    0
        BROWN = 2,       // 153   51    0
        MAGENTA = 3,     // 255    0  255
        CYAN = 4,        //   0  255  255
        RED = 5,         // 255    0    0
        YELLOW = 6,      // 230  230    0
        LIGHT_BROWN = 7, // 217   84   26
        PURPLE = 8,      // 125   46  143
        MUSTARD = 9,     // 237  176   33
        PINK = 10,        // 255  153  199
        BLACK = 11,       //   0    0    0
        GREY = 12,        // 200  200  200
    };

    PlotDialog(QWidget *parent=0);
    ~PlotDialog();

    void hold(bool set);
    void grid(bool set);

    void plot(const arma::vec &data);
    void setTitle(const std::string &title);
    void setXLabel(const std::string &label);
    void setYLabel(const std::string &label);
    void setLegend(const std::vector<std::string> &legend_labels);

private:

    std::map<PlotDialog::Color, QColor> color;
    QGridLayout *main_layout;
    QCustomPlot *cplot;
    bool hold_on;

    int color_ind;
    const int N_COLORS = 13;
};

class DataPlotDialog : public QDialog
{
    Q_OBJECT

public:
    DataPlotDialog(const RecData *rec_data, QWidget *parent = 0);
    ~DataPlotDialog();

    void launch();

signals:
    void closePlots();

private slots:
    void plotBtnPressed();
    void cancelBtnPressed();

private:
    std::vector<QCheckBox *> chkbox;

    const RecData *rec_data;

    void addCheckBox(const char *label);

    void plotData(DataType::ID id);

    void closeEvent(QCloseEvent *event) override;
};

#endif // DATA_PLOT_DIALOG_H
