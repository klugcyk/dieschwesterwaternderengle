/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹，她的身旁常伴月光
    start:221129
    last:230608
*/

#include "ui_control/mainwindow.h"
#include "ui_control/img1.h"
#include <QApplication>
#include "source.hpp"
#include <sys/time.h> //系统时间

bool link_update=0;
bool richt_update=0;
std::string read_path;
std::string write_path;
bool recal_flag=0; //重新标定标志位,1:重新标定，0:读取标定参数
bool cal_done=~recal_flag; //标定完成标志位
bool communication_open=0; //开启通讯标志

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow w;
    w.show();

    return a.exec();
}
