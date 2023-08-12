/*
    文件等级：密一
    author:klug
    献给我的恩师阿尔瓦罗赛佩达萨姆迪奥
    start:230825
    last:230825
*/

#ifndef CONSTRUCTMAIN_H
#define CONSTRUCTMAIN_H

#include <QWidget>
#include <iostream>
#include <thread>

namespace Ui {
class constructMain;
}

class constructMain : public QWidget
{
    Q_OBJECT

public:
    explicit constructMain(QWidget *parent = nullptr);
    ~constructMain();

private:
    Ui::constructMain *ui;
    std::thread imgTransThread; //图像传输线程

};

#endif // CONSTRUCTMAIN_H
