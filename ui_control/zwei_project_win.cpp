/*
    文件等级：密一
    author:klug
    献给杜尔西内娅德尔托博索
    start:230706
    last:230706
*/

#include "zwei_project_win.h"

zwei_project_win::zwei_project_win(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::zwei_project_win)
{
    ui->setupUi(this);
    setWindowTitle("zwei_pro_construct");
}

zwei_project_win::~zwei_project_win()
{
    delete ui;
}

