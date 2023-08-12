/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹，她是美丽女神阿尔忒弥斯的神女
    start:230825
    last:230825
*/

#include <QApplication>
#include "ui_control/constructmain.h"
#include <sys/time.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    constructMain m;

    m.show();

    return a.exec();
}
