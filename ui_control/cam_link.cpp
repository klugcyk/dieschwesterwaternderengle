/*
    文件等级：密一
    author:klug
    献给杜尔西内娅德尔托博索
    start:230510
    last:230625
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

    cv::Mat img=imread("/home/klug/img/zwei_construct/cam_link.png");

    this->setFixedSize(img.cols/2,img.rows/2);
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
        img.load("/home/klug/img/zwei_construct/cam_link.png");
        QImage qimg=img.scaled(img.width()/2,img.height()/2).scaled(img.width()/2,img.height()/2,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
        ui->img_link->setPixmap(QPixmap::fromImage(qimg));
        ui->img_link->resize(qimg.size());
        ui->img_link->show();
        printf("show link img\n");
        usleep(500000);
    }
}
