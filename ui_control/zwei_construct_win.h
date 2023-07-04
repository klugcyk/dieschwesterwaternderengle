/*
    文件等级：密一
    author:klug
    献给不喜欢我的弗雷德里希冯海因洛特
    start:230510
    last:230626
*/

#ifndef ZWEI_CONSTRUCT_WIN_H
#define ZWEI_CONSTRUCT_WIN_H

#include <QWidget>
#include "StructureLight/zwei_construct.hpp"
#include "camera/camera.hpp"

#define zwei_win_print_error_info
#define zwei_win_print_msg_info
#define zwei_win_print_data_info
#define zwei_win_save_process

namespace Ui {
class zwei_construct_win;
}

class zwei_construct_win : public QWidget,public ZweiConstruct::construct,public basler_camera
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
    void on_grab_zwei_clicked();
    void on_sys_cal_clicked();
    void on_save_clicked();
    void on_set_param_clicked();
    void on_load_param_clicked();

private:
    Ui::zwei_construct_win *ui;
    std::string img_read_path;
    std::string img_write_path;

};

#endif // ZWEI_CONSTRUCT_WIN_H
