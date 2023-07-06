/*
    文件等级：密一
    author:klug
    献给杜尔西内娅德尔托博索
    start:230706
    last:230706
*/

#include "zwei_project.h"
#include "ui_zwei_project.h"

zwei_project::zwei_project(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::zwei_project)
{
    ui->setupUi(this);
    setWindowTitle("zwei_project");
}

zwei_project::~zwei_project()
{
    delete ui;
}
