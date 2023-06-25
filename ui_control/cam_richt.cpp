/*
    文件等级：密一
    author:klug
    献给杜尔西内娅德尔托博索
    start:230510
    last:230625
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

    cv::Mat img=imread("/home/klug/img/zwei_construct/cam_richt.png");

    this->setFixedSize(img.cols/2,img.rows/2);
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
        img.load("/home/klug/img/zwei_construct/cam_richt.png");
        QImage qimg=img.scaled(img.width()/2,img.height()/2).scaled(img.width()/2,img.height()/2,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
        ui->img_richt->setPixmap(QPixmap::fromImage(qimg));
        ui->img_richt->resize(qimg.size());
        ui->img_richt->show();
        printf("show richt img\n");
        usleep(500000);
    }
}
