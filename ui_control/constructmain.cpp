/*
    文件等级：密一
    author:klug
    献给我的恩师阿尔瓦罗赛佩达萨姆迪奥
    start:230825
    last:230825
*/

#include "constructmain.h"
#include "ui_constructmain.h"

constructMain::constructMain(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::constructMain)
{
    ui->setupUi(this);
    setWindowTitle("constructMain");
    //imgTransThread=std::thread();
}

constructMain::~constructMain()
{
    //imgTransThread.join();
    delete ui;
}
