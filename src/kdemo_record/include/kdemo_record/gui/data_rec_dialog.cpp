#include "data_rec_dialog.h"

DataRecDialog::DataRecDialog(QWidget *parent): QDialog(parent)
{
    addCheckBox("Position"); // TOOL_POS
    addCheckBox("Orientation"); // TOOL_ORIENT
    addCheckBox("Force"); // TOOL_FORCE
    addCheckBox("Torque"); // TOOL_TORQUE
    addCheckBox("Position"); // JOINT_POS
    addCheckBox("Torque"); // JOINT_TORQUE
    addCheckBox("Jacobian"); // JACOBIAN
    // **To add extra checkboxes:
    // addCheckBox("New type"); // NEW_DATA_TYPE
    // main_layout->addWidget(chkbox[DataType::NEW_DATA_TYPE],x,y);

    this->setWindowTitle("Data to record");

    QLabel *cart_label = new QLabel("Cartesian:");
    cart_label->setStyleSheet("background-color: rgb(250,250,250); color: rgb(0,0,0); font: 75 16pt \"FreeSans\";");
    QLabel *joint_label = new QLabel("Joint:");
    joint_label->setStyleSheet("background-color: rgb(250,250,250); color: rgb(0,0,0); font: 75 16pt \"FreeSans\";");

    QPushButton *ok_btn = new QPushButton("Ok");
    QPushButton *cancel_btn = new QPushButton("Cancel");

    QHBoxLayout *btn_layout = new QHBoxLayout;
    btn_layout->addWidget(ok_btn);
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
    main_layout->addWidget(chkbox[DataType::JACOBIAN],3,2);

    main_layout->addItem(new QSpacerItem(0,20),5,1);

    main_layout->addLayout(btn_layout,6,0,1,3);

    QObject::connect(ok_btn, &QPushButton::clicked, this, &DataRecDialog::okBtnPressed);
    QObject::connect(cancel_btn, &QPushButton::clicked, this, &DataRecDialog::cancelBtnPressed);
}

DataRecDialog::~DataRecDialog()
{

}

void DataRecDialog::addCheckBox(const char *label)
{
    rec.push_back(false);
    QCheckBox *box = new QCheckBox(label);
    box->setStyleSheet("font: 75 14pt \"FreeSans\";");
    chkbox.push_back(box);
}

void DataRecDialog::launch()
{
    this->show();
}

void DataRecDialog::okBtnPressed()
{
    for (int i=0; i<rec.size(); i++) rec[i] = chkbox[i]->isChecked();
    this->hide();
}

void DataRecDialog::cancelBtnPressed()
{
    for (int i=0; i<rec.size(); i++) chkbox[i]->setChecked(rec[i]);
    this->hide();
}

void DataRecDialog::closeEvent(QCloseEvent *event)
{
    cancelBtnPressed();
}
