/*
    文件等级：密一
    author:klug
    献给我的好友脚踏西瓜皮的胡安帕诺麦克
    start:230829
    last:230829
*/

#include "toolpage.h"
#include "ui_toolpage.h"
#include <QDebug>
#include <QFormLayout>
#include <QDebug>
#include <QHBoxLayout>
#include <QLabel>
#include <QFile>

toolPage::toolPage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::toolPage)
{
    ui->setupUi(this);
    QImage img("/home/klug/dieschwesterwaternderengle/source/9.png");
    QImage img1=img.scaled(10,10,Qt::KeepAspectRatio);
    ui->labelImg->setPixmap(QPixmap::fromImage(img1));
}

toolPage::~toolPage()
{
    delete ui;
}

void toolPage::on_openTree_clicked()
{
    if(status)
    {
        QImage img("/home/klug/dieschwesterwaternderengle/source/1.jpeg");
        QImage img1=img.scaled(10,10,Qt::KeepAspectRatio);
        ui->labelImg->setPixmap(QPixmap::fromImage(img1));
        status=0;
    }
    else
    {
        QImage img("/home/klug/dieschwesterwaternderengle/source/9.png");
        QImage img1=img.scaled(10,10,Qt::KeepAspectRatio);
        ui->labelImg->setPixmap(QPixmap::fromImage(img1));
        status=1;
    }
}

