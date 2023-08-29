#include "test.h"
#include "ui_test.h"
#include <QWidget>
#include <QObject>

test::test(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::test)
{
    ui->setupUi(this);
    //QObject::connect(ui->horizontalSlider,&QSlider::valueChanged,&test::setT);
}

test::~test()
{
    delete ui;
}

void test::on_horizontalSlider_sliderMoved(int position)
{
    int i=ui->horizontalSlider->value();
    ui->lineEdit->setText(QString::number(i));
    ui->spinBox->setValue(i);
}

void test::on_spinBox_valueChanged(int arg1)
{
    int i=ui->spinBox->value();
    ui->horizontalSlider->setValue(i);
    ui->lineEdit->setText(QString::number(i));
}
