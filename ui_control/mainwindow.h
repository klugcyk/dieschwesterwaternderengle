/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:221129
    last:230609
*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "camera/camera.hpp"
#include "StructureLight/construct.hpp"
#include "StructureLight/zwei_construct_cal.hpp"
#include "ui_control/img1.h"
#include "ui_control/zwei_construct_win.h"
#include "CameraPose/cam_pose.hpp"
#include "communication/communication.hpp"

// 使用相机类型
#define camera_rgb
//#define camera_gray

#define for_3d_construct //此时用于三维重建
//#define for_zweilight //此时用于双目结构光

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow,
#ifdef for_3d_construct
        public structlight_construct::construct,
        public cal_cam_pose::cam_pose,
        public communication_up::communication
#endif

#ifdef for_zweilight
        public zwei_light_cal
#endif
        ,public basler_camera
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();  

private:
    std::vector<cv::Mat> img_vector;
    cv::Mat cam_img;
    void img_show();
    void img_show_continue();
    void res_show(cv::Mat &res_img);

private slots:
    void on_sys_cal_clicked();
    void on_func_test_clicked();
    void on_img_1_clicked();
    void on_read_param_clicked();
    void on_grab_clicked();
    void on_save_clicked();
    void on_set_param_clicked();
    void on_zwei_clicked();
    void on_cam_link_clicked();
    void on_cam_richt_clicked();
    void on_construct_clicked();
    void on_save_cal_param_clicked();
    void on_undistort_clicked();
    void on_read_cal_param_clicked();

private:
    Ui::MainWindow *ui;
    std::thread camera_show_thread;//相机连续采集显示图像线程 img_show_continue()

};

#endif // MAINWINDOW_H
