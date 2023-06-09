/*
    文件等级：密一
    author:klug
    献给我亲爱的师弟长发的阿开奥斯人
    start:230515
    last:230515
*/

#ifndef PC_SHOW_H
#define PC_SHOW_H

#include <QWidget>
//#include <vtkRenderWindow.h>//注意添加这个头文件，不添加会报错，但是这个头文件会带来编译时间大大增加
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace Ui {
class pc_show;
}

class pc_show : public QWidget
{
    Q_OBJECT

public:
    explicit pc_show(QWidget *parent = nullptr);
    ~pc_show();

private:
    Ui::pc_show *ui;
};

#endif // PC_SHOW_H
