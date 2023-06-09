/*
    文件等级：密一
    author:klug
    献给杜尔西内娅德尔托博索
    start:230510
    last:230511
*/

#include "cam_richt.h"
#include "ui_cam_richt.h"
#include <unistd.h>
#include "source.hpp"

cam_richt::cam_richt(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::cam_richt)
{
    ui->setupUi(this);
    richt_update=1;
    setWindowTitle("cam_richt");
    img_richt=std::thread(&cam_richt::img_show,this);
    this->setFixedSize(cam_width/4,cam_height/4);
    this->setAttribute(Qt::WA_DeleteOnClose);
}

cam_richt::~cam_richt()
{
    richt_update=0;
    img_richt.join();
    delete ui;
}

void cam_richt::img_show()
{
    while(richt_update)
    {
        QImage img;
        img.load("/home/klug/img/zwei_construct/2.png");
        QImage qimg=img.scaled(img.width()/4,img.height()/4).scaled(img.width()/4,img.height()/4,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
        ui->img_richt->setPixmap(QPixmap::fromImage(qimg));
        ui->img_richt->resize(qimg.size());
        ui->img_richt->show();
        printf("show richt img\n");
        usleep(500000);
    }
}
