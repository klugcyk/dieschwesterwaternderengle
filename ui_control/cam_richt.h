/*
    文件等级：密一
    author:klug
    献给杜尔西内娅德尔托博索
    start:230510
    last:230510
*/

#ifndef CAM_RICHT_H
#define CAM_RICHT_H

#include <QWidget>
#include <iostream>
#include <thread>
#include "camera/camera.hpp"

namespace Ui {
class cam_richt;
}

class cam_richt : public QWidget,public basler_camera
{
    Q_OBJECT

public:
    explicit cam_richt(QWidget *parent = nullptr);
    ~cam_richt();

private:
    Ui::cam_richt *ui;
    std::thread img_richt; //刷新图像线程
    void img_show();

};

#endif // CAM_RICHT_H
