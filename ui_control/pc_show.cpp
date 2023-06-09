/*
    文件等级：密一
    author:klug
    献给我亲爱的师弟长发的阿开奥斯人
    start:230515
    last:230515
*/

#include "pc_show.h"
#include "ui_pc_show.h"

pc_show::pc_show(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::pc_show)
{
    ui->setupUi(this);
}

pc_show::~pc_show()
{
    delete ui;
}
