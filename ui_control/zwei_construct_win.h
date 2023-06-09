/*
    文件等级：密一
    author:klug
    献给不喜欢我的弗雷德里希冯海因洛特
    start:230510
    last:230515
*/

#ifndef ZWEI_CONSTRUCT_WIN_H
#define ZWEI_CONSTRUCT_WIN_H

#include <QWidget>
#include "StructureLight/zwei_construct.hpp"
#include "camera/camera.hpp"

namespace Ui {
class zwei_construct_win;
}

class zwei_construct_win : public QWidget,public zwei_construct::construct,public basler_camera
{
    Q_OBJECT

public:
    explicit zwei_construct_win(QWidget *parent = nullptr);
    ~zwei_construct_win();

private slots:
    void on_read_param_link_clicked();
    void on_read_param_richt_clicked();
    void on_set_param_link_clicked();
    void on_set_param_richt_clicked();

private:
    Ui::zwei_construct_win *ui;
};

#endif // ZWEI_CONSTRUCT_WIN_H
