#include "data_plot_dialog.h"

#include <QDebug>

DataPlotDialog::DataPlotDialog(const RecData *rec_data, QWidget *parent): QDialog(parent)
{
    this->rec_data = rec_data;

    addCheckBox("Position"); // TOOL_POS
    addCheckBox("Orientation"); // TOOL_ORIENT
    addCheckBox("Force"); // TOOL_FORCE
    addCheckBox("Torque"); // TOOL_TORQUE
    addCheckBox("Position"); // JOINT_POS
    addCheckBox("Torque"); // JOINT_TORQUE
    // addCheckBox("Jacobian"); // JACOBIAN
    // **To add extra checkboxes:
    // addCheckBox("New type"); // NEW_DATA_TYPE
    // main_layout->addWidget(chkbox[DataType::NEW_DATA_TYPE],x,y);

    this->setWindowTitle("Data to plot");

    QLabel *cart_label = new QLabel("Cartesian:");
    cart_label->setStyleSheet("background-color: rgb(250,250,250); color: rgb(0,0,0); font: 75 16pt \"FreeSans\";");
    QLabel *joint_label = new QLabel("Joint:");
    joint_label->setStyleSheet("background-color: rgb(250,250,250); color: rgb(0,0,0); font: 75 16pt \"FreeSans\";");

    QPushButton *plot_btn = new QPushButton("Plot");
    QPushButton *cancel_btn = new QPushButton("Cancel");

    QHBoxLayout *btn_layout = new QHBoxLayout;
    btn_layout->addWidget(plot_btn);
    btn_layout->addWidget(cancel_btn);

    QGridLayout *main_layout = new QGridLayout(this);
    main_layout->setSizeConstraint(QLayout::SetFixedSize);

    main_layout->addWidget(cart_label,0,0, Qt::AlignLeft);
    main_layout->addWidget(chkbox[DataType::TOOL_POS],1,0);
    main_layout->addWidget(chkbox[DataType::TOOL_ORIENT],2,0);
    main_layout->addWidget(chkbox[DataType::TOOL_FORCE],3,0);
    main_layout->addWidget(chkbox[DataType::TOOL_TORQUE],4,0);

    main_layout->addItem(new QSpacerItem(20,0),0,1);

    main_layout->addWidget(joint_label,0,2);
    main_layout->addWidget(chkbox[DataType::JOINT_POS],1,2);
    main_layout->addWidget(chkbox[DataType::JOINT_TORQUE],2,2);
    // main_layout->addWidget(chkbox[DataType::JACOBIAN],3,2);

    main_layout->addItem(new QSpacerItem(0,20),5,1);

    main_layout->addLayout(btn_layout,6,0,1,3);

    QObject::connect(plot_btn, &QPushButton::clicked, this, &DataPlotDialog::plotBtnPressed);
    QObject::connect(cancel_btn, &QPushButton::clicked, this, &DataPlotDialog::cancelBtnPressed);
}

DataPlotDialog::~DataPlotDialog()
{

}

void DataPlotDialog::addCheckBox(const char *label)
{
    QCheckBox *box = new QCheckBox(label);
    box->setStyleSheet("font: 75 14pt \"FreeSans\";");
    chkbox.push_back(box);
}

void DataPlotDialog::launch()
{
//    QCheckBox *box = chkbox[0];
//    QPalette pal = box->palette();
//    QColor bg_color = pal.color(QPalette::Background);
//    qDebug() << "bg color: (" << bg_color.red() << ", " << bg_color.green() << ", " << bg_color.blue() << ")\n";
//    qDebug() << "stylesheet: " << box->styleSheet() << "\n";

    for (int i=0;i<chkbox.size();i++)
    {
        chkbox[i]->setCheckable(!rec_data->isEmpty(static_cast<DataType::ID>(i)));
        if (chkbox[i]->isCheckable()) chkbox[i]->setStyleSheet("background-color: rgb(242,241,240);");
        else chkbox[i]->setStyleSheet("background-color: rgb(215,215,215);");
    }
    this->show();
}

void DataPlotDialog::plotBtnPressed()
{
    for (int i=0; i<chkbox.size(); i++)
    {
        if (chkbox[i]->isChecked()) plotData(static_cast<DataType::ID>(i));
    }
}

void DataPlotDialog::cancelBtnPressed()
{
    emit closePlots();
    this->hide();
}

void DataPlotDialog::plotData(DataType::ID id)
{
    PlotDialog *fig = new PlotDialog(this);
    fig->hold(true);
    fig->setXLabel("timestep");
    QObject::connect( this, SIGNAL(closePlots()), fig, SLOT(close()) );

    const arma::mat *data = rec_data->getData(id);
    int n_graphs = data->n_rows;
    for (int i=0;i<n_graphs;i++) fig->plot(data->row(i).t());

    std::vector<std::string> legend_labels;
    std::string title;
    std::string y_label;
    legend_labels.resize(n_graphs);

    switch (id)
    {
        case DataType::TOOL_POS:
            title = "Tool Position";
            y_label = "[m]";
            legend_labels = {"x","y","z"};
            break;
        case DataType::TOOL_ORIENT:
            title = "Tool Orientation";
            y_label = "Unit Quaternion";
            legend_labels = {"qw", "qx", "qy", "qz"};
            break;
        case DataType::TOOL_FORCE:
            title = "Tool Force";
            y_label = "[N]";
            legend_labels = {"fx","fy","fz"};
            break;
        case DataType::TOOL_TORQUE:
            title = "Tool Torque";
            y_label = "[Nm]";
            legend_labels = {"tx","ty","tz"};
            break;
        case DataType::JOINT_POS:
            title = "Joints Position";
            y_label = "[rad]";
            for (int i=0;i<legend_labels.size();i++) legend_labels[i] = QString("j" + QString::number(i+1)).toStdString();
            break;
        case DataType::JOINT_TORQUE:
            title = "Joints Torque";
            y_label = "[Nm]";
            for (int i=0;i<legend_labels.size();i++) legend_labels[i] = QString("j" + QString::number(i+1)).toStdString();
            break;
    }

    fig->setTitle(title);
    fig->setYLabel(y_label);
    fig->setLegend(legend_labels);

    fig->show();
}

void DataPlotDialog::closeEvent(QCloseEvent *event)
{
    cancelBtnPressed();
}

// ======================================================
// ======================================================

PlotDialog::PlotDialog(QWidget *parent) : QDialog(parent)
{
    color[PlotDialog::BLUE] = QColor(0, 0, 255);
    color[PlotDialog::GREEN] = QColor(0, 255, 0);
    color[PlotDialog::BROWN] = QColor(153, 51, 0);
    color[PlotDialog::MAGENTA] = QColor(255, 0, 255);
    color[PlotDialog::CYAN] = QColor(0, 255, 255);
    color[PlotDialog::RED] = QColor(255, 0, 0);
    color[PlotDialog::YELLOW] = QColor(230, 230, 0);
    color[PlotDialog::LIGHT_BROWN] = QColor(217, 84, 26);
    color[PlotDialog::PURPLE] = QColor(125, 46, 143);
    color[PlotDialog::MUSTARD] = QColor(237, 176, 33);
    color[PlotDialog::PINK] = QColor(255, 153, 199);
    color[PlotDialog::BLACK] = QColor(0, 0, 0);
    color[PlotDialog::GREY] = QColor(200, 200, 200);

    this->resize(667, 452);

    this->setAttribute(Qt::WA_DeleteOnClose);

    cplot = new QCustomPlot;
    // set locale to english, so we get english decimal separator:
    cplot->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
    // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
    cplot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    color_ind = 0;

    main_layout = new QGridLayout(this);
    main_layout->addWidget(cplot, 0, 0, 1, 1);

    hold(false);
    grid(false);
}

PlotDialog::~PlotDialog()
{
    // delete this;
}

void PlotDialog::plot(const arma::vec &data)
{
    if (!hold_on)
    {
        cplot->clearGraphs();
        color_ind = 0;
    }

    int n_data = data.size();
    QVector<double> x_data(n_data);
    QVector<double> y_data(n_data);
    for (int i=0; i<n_data; i++)
    {
        x_data[i] = i;
        y_data[i] = data(i);
    }

    QCPGraph *graph = new QCPGraph(cplot->xAxis, cplot->yAxis);
    graph->setData(x_data, y_data);
    QColor c = (color.find(static_cast<PlotDialog::Color>(color_ind)))->second;
    graph->setPen(QPen(QBrush(c), 2.0, Qt::SolidLine)); // line color blue for first graph
    color_ind = (color_ind+1)%N_COLORS;
    graph->setLineStyle(QCPGraph::lsLine); // connect points with straight lines
    graph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot, 4)); // set marker type and size. The width is determined by the linewidth

    cplot->rescaleAxes();
}

void PlotDialog::hold(bool set)
{
    hold_on = set;
}

void PlotDialog::grid(bool set)
{
    cplot->xAxis->grid()->setVisible(set);
    cplot->yAxis->grid()->setVisible(set);
}

void PlotDialog::setTitle(const std::string &title)
{
    int fontsize = 16;
    QString font_family = "Arial";

    cplot->plotLayout()->insertRow(0);
    cplot->plotLayout()->addElement(0, 0, new QCPTextElement(cplot, title.c_str(), QFont(font_family, fontsize, QFont::Normal)));
}

void PlotDialog::setXLabel(const std::string &label)
{
    int fontsize = 14;
    QString font_family = "Arial";

    QCPAxis *x_axis = cplot->xAxis;
    x_axis->setLabel(label.c_str());
    x_axis->setLabelColor(QColor(0,0,0));
    x_axis->setLabelFont(QFont(font_family, fontsize, QFont::Normal));
}

void PlotDialog::setYLabel(const std::string &label)
{
    int fontsize = 14;
    QString font_family = "Arial";

    QCPAxis *y_axis = cplot->yAxis;
    y_axis->setLabel(label.c_str());
    y_axis->setLabelColor(QColor(0,0,0));
    y_axis->setLabelFont(QFont(font_family, fontsize, QFont::Normal));
}

void PlotDialog::setLegend(const std::vector<std::string> &legend_labels)
{
    int fontsize = 14;
    QString font_family = "Arial";

    int n_graphs = cplot->graphCount();
    int n_labels = legend_labels.size();

    if (n_labels > n_graphs)
    {
        qDebug() << "Extra legend labels will be ignored.";
        n_labels = n_graphs;
    }

    for (int i=0; i<n_labels; i++) cplot->graph(i)->setName(legend_labels[i].c_str());

    cplot->legend->setVisible(true);
    cplot->legend->setFont(QFont(font_family, fontsize, QFont::Normal));
    cplot->legend->setBrush(QBrush(QColor(255,255,255,230)));
    cplot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom|Qt::AlignRight);
}
