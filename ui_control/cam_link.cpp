/*
    文件等级：密一
    author:klug
    献给杜尔西内娅德尔托博索
    start:230510
    last:230511
*/

#include "cam_link.h"
#include "ui_cam_link.h"
#include <unistd.h>
#include "source.hpp"

cam_link::cam_link(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::cam_link)
{
    link_update=1;
    ui->setupUi(this);
    setWindowTitle("cam_link");
    img_link=std::thread(&cam_link::img_show,this);
    this->setFixedSize(cam_width/4,cam_height/4);
    this->setAttribute(Qt::WA_DeleteOnClose);
}

cam_link::~cam_link()
{
    link_update=0;
    img_link.join();
    //img_link.
    delete ui;
}

void cam_link::img_show()
{
    while(link_update)
    {
        QImage img;
        img.load("/home/klug/img/zwei_construct/1.png");
        QImage qimg=img.scaled(img.width()/4,img.height()/4).scaled(img.width()/4,img.height()/4,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
        ui->img_link->setPixmap(QPixmap::fromImage(qimg));
        ui->img_link->resize(qimg.size());
        ui->img_link->show();
        printf("show link img\n");
        usleep(500000);
    }
}
