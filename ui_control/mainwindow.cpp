/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:221129
    last:230609
*/

#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "StructureLight/construct_cal.hpp"
#include "pointcloud/pointcloud.hpp"
#include "cam_link.h"
#include "cam_richt.h"
#include "source.hpp"
#include <dlfcn.h>//显式调用动态链接库使用
#include "gene_operate/json_operate.hpp"

#ifdef depend_outside
#include "camera/camera_.hpp"
#endif
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("3d_construct");
    if(camera_continue_switch)
    {
        camera_show_thread=std::thread(&MainWindow::img_show_continue,this);
    }
}

MainWindow::~MainWindow()
{
    if(camera_continue_switch)
    {
        camera_show_thread.join();
    }
    delete ui;
}

#ifdef for_3d_construct

void MainWindow::img_show()
{
    QImage img;
    img.load("/home/klug/img/construct/cam_img.png");
    QImage qimg=img.scaled(img.width()/4,img.height()/4).scaled(img.width()/4,img.height()/4,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
    ui->img_continue->setPixmap(QPixmap::fromImage(qimg));
    ui->img_continue->resize(qimg.size());
    ui->img_continue->show();
}

void MainWindow::img_show_continue()
{
    while(1)
    {
        std::unique_lock<std::mutex> show_lock(continue_lock);

        if(!cam_img_continue.empty())
        {
            imshow("g",cam_img_continue);
            show_lock.unlock();
            sleep(1);
        }
    }
}

void MainWindow::on_save_clicked()
{
    int i=1;

    i=ui->img_name->text().toInt();
    std::string num=std::to_string(i);
    std::string n;
    QString name;
    QString num_;
    num_=QString::fromStdString(num);

    n="/home/klug/img/construct/";
    n+=std::to_string(i);
    n+=".png";
    name=QString::fromStdString(n);

    img_show();
    QImage img;
    img.load("/home/klug/img/construct/cam_img.png");
    img.save(name);
    ui->img_name->setText(num_);
}

void MainWindow::on_sys_cal_clicked()
{
    std::vector<cv::Mat> cal_img;
    std::vector<cv::Mat> laser_img;

    // load the image from the path for the camera calibrate
    int cal_img_cnt=30; //加载图片的数量
    for(int i=1;i<=cal_img_cnt;i++)
    {
        std::string path="/home/klug/img/construct/";
        path+=std::to_string(i);
        path+=".png";
        cv::Mat temp_img=imread(path);
        if(!temp_img.empty())
        {
            //cv::Mat gray_img;
            //cv::cvtColor(temp_img,gray_img,cv::COLOR_RGB2GRAY);
            cal_img.push_back(temp_img);
        }
    }

    for(int i=126;i<=130;i++)
    {
        std::string path="/home/klug/img/construct/";
        path+=std::to_string(i);
        path+=".png";
        cv::Mat temp_img=imread(path);
        if(!temp_img.empty())
        {
            laser_img.push_back(temp_img);
        }
    }

    system_calibrate(cal_img,laser_img,cv::Size(6,9),cv::Size2f(6.95,6.95));
    //system_calibrate(cal_img,laser_img,cv::Size(6,9),cv::Size2f(3.95,3.95));
}

void MainWindow::on_func_test_clicked()
{
/*    cv::Mat img=imread("/home/klug/img/construct/undistort/126.png");
    if(img.empty())
    {
        return ;
    }
    cv::Mat res(img.rows,img.cols,CV_8UC1);
    laser_zenturm_line_zwei(img,res);

    cv::imwrite("/home/klug/img/construct/res_functest.jpg",res);
    res_show(res);
    cal_test();*/
    //img_show_continue();
    cv::Mat i=cv::imread("/home/klug/img/construct/1.png");
    img_encode(i);
}

#endif

#ifdef camera_gray

void MainWindow::on_grab_clicked()
{
    camera_grab_gray();
    img_show();
}

#endif

#ifdef camera_rgb

void MainWindow::on_grab_clicked()
{
    camera_grab_rgb();
    img_show();
}

#endif

void MainWindow::on_set_param_clicked()
{
    set_exposure_time=ui->exposure_time_set->text().toInt();
    camera_set_parameter();
    printf("set the parameter...\n");
}

void MainWindow::on_read_param_clicked()
{
    camera_read_parameter();

    std::string temp=std::to_string(read_exposure_time);
    QString et=QString::fromStdString(temp);
    ui->exposure_time_set->setText(et);
}

void MainWindow::res_show(cv::Mat &res_img)
{
    cvtColor(res_img,res_img,cv::COLOR_BGR2RGB);
    QImage img((const unsigned char*)res_img.data,res_img.cols,res_img.rows,res_img.step,QImage::Format_RGB888);
    QImage qimg=img.scaled(800,600).scaled(640,512,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
    ui->img_continue->setPixmap(QPixmap::fromImage(qimg));
    ui->img_continue->resize(qimg.size());
    ui->img_continue->show();
}

void MainWindow::on_img_1_clicked()
{
    img1 *new_win = new img1;
    new_win->show();
}

void MainWindow::on_zwei_clicked()
{
    zwei_construct_win *new_win = new zwei_construct_win;
    new_win->show();
}


void MainWindow::on_cam_link_clicked()
{
    if(!link_update)
    {
        cam_link *new_win = new cam_link;
        new_win->show();
    }
}


void MainWindow::on_cam_richt_clicked()
{
    if(!richt_update)
    {
        cam_richt *new_win = new cam_richt;
        new_win->show();
    }
}

//三维重建测试
void MainWindow::on_construct_clicked()
{
    int i=1;

    i=ui->img_name->text().toInt();
    if(i<=200)
    {
        i=201;
    }

    std::string num=std::to_string(i);
    std::string n;
    QString name;
    QString num_;
    num_=QString::fromStdString(num);

    read_path=read_img_path;
    read_path+=std::to_string(i);
    read_path+=".png";
    cv::Mat construct_img=cv::imread(read_path);
    cv::Mat res_img,undistort_img;

    cv::undistort(construct_img,undistort_img,cameraMatrix,distCoeffs);

    if(!undistort_img.empty())
    {
        write_path=write_img_path;
        write_path+="undistort_img_test.png";
        cv::imwrite(write_path,undistort_img);
        std::vector<cv::Point2f> get_points;
        std::vector<math_geometry::point3> get_gp;
        construct_img_test(undistort_img,res_img,get_points);
        get_gp=construct_point_ein(get_points);
        construct_test(get_gp);
    }
}

/*
    保存参数标定数据到文件
*/
void MainWindow::on_save_cal_param_clicked()
{
    if(cal_done)
    {
        double fx=cameraMatrix.at<double>(0,0);
        double fy=cameraMatrix.at<double>(1,1);
        double u0=cameraMatrix.at<double>(0,2);
        double v0=cameraMatrix.at<double>(1,2);
        double pa=light_plane_ein.A;
        double pb=light_plane_ein.B;
        double pc=light_plane_ein.C;
        double pd=light_plane_ein.D;

        std::string string_temp;
        //char param_name[9][5]={{"camera_param"},{"fx"},{"fy"},{"u0"},{"v0"},{"A"},{"B"},{"C"},{"D"}}; //相机参数
        std::string param_name[9]={"camera_param","fx","fy","u0","v0","A","B","C","D"}; //相机参数
        double param_data[9]={2,fx,fy,u0,v0,pa,pb,pc,pd};
        //mein_json::json_write_construct("/home/klug/system_param.json",&param_name[0][0],param_data,5,9);
        write_path=write_json_path;
        write_path+="system_param.json";
        mein_json::json_write(write_path,param_name,param_data,9);
    }
    else
    {
        printf("bitte calibrate camera Sie first...\n");
    }
    mein_json::json_read("/home/klug/system_param.json");
}

//去除图像畸变
void MainWindow::on_undistort_clicked()
{
    if(cal_done)
    {
        cv::Mat src_img,undistort_img;
        for(int i=126;i<=130;i++)
        {
            read_path=read_img_path;
            read_path+=std::to_string(i);
            read_path+=".png";
            write_path=undistort_img_path;
            write_path+=std::to_string(i);
            write_path+=".png";
            src_img=cv::imread(read_path);
            if(!src_img.empty())
            {
                cv::undistort(src_img,undistort_img,cameraMatrix,distCoeffs);
                cv::imwrite(write_path,undistort_img);
            }
            else
            {
                printf("img empty...\n");
            }
        }
    }
    else
    {
        printf("bitte calibrate camera Sie first...\n");
    }
}

void MainWindow::on_read_cal_param_clicked()
{
    std::string get_name[9]={"camera_param","fx","fy","u0","v0","A","B","C","D"};
    double get_value[9];

    read_path=read_json_path;
    read_path+="system_param.json";

    mein_json::json_read(read_path,get_name,get_value);
    std::cout<<"read achieve..."
            <<std::endl;
}
