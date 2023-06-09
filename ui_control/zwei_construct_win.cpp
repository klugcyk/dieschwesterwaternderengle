/*
    文件等级：密一
    author:klug
    献给不喜欢我的弗雷德里希冯海因洛特
    start:230510
    last:230515
*/

#include "zwei_construct_win.h"
#include "ui_zwei_construct_win.h"

zwei_construct_win::zwei_construct_win(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::zwei_construct_win)
{
    ui->setupUi(this);
    setWindowTitle("zwei_construct");
}

zwei_construct_win::~zwei_construct_win()
{
    delete ui;
}

void zwei_construct_win::on_read_param_link_clicked()
{

}

void zwei_construct_win::on_read_param_richt_clicked()
{

}

void zwei_construct_win::on_set_param_link_clicked()
{

}

void zwei_construct_win::on_set_param_richt_clicked()
{

}

