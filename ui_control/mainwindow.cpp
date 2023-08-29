/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:230828
    last:230828
*/

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("constructMain");

    //ui->paramBox->setFrameStyle(QFrame::Box);
}

MainWindow::~MainWindow()
{
    delete ui;
}
