/*
    文件等级：密一
    author:klug
    献给杜尔西内娅德尔托博索
    start:230510
    last:230510
*/

#ifndef CAM_LINK_H
#define CAM_LINK_H

#include <QWidget>
#include <iostream>
#include <thread>
#include "camera/camera.hpp"

namespace Ui {
class cam_link;
}

class cam_link : public QWidget,public basler_camera
{
    Q_OBJECT

public:
    explicit cam_link(QWidget *parent = nullptr);
    ~cam_link();

private:
    Ui::cam_link *ui;
    void img_show();
    std::thread img_link;

};

#endif // CAM_LINK_H
