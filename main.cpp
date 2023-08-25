/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹，她是美丽的女神阿尔忒弥斯的神女
    start:230825
    last:230825
*/

#include <QApplication>
#include "ui_control/constructmain.h"
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include "com/imgCode.hpp"

int main(int argc, char *argv[])
{

    cv::Mat img=cv::imread("/home/klug/img/construct/cal/1.png");
    imgcode ic(img);

    return 0;
/*
    QApplication a(argc, argv);
    constructMain m;

    m.show();

    return a.exec();
*/
}
