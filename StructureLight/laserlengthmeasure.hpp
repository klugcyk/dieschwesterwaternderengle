/*
    文件等级：密一
    author:klug
    献给我亲爱的好友何塞阿尔卡迪奥
    start:230222
    last:230426
*/

#ifndef laser_length_measure_HPP
#define laser_length_measure_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include "construct_cal.hpp"
#include "ander/laser_zenturm_extract.hpp"
#include "geometry.hpp"

#define laser_length_measure_print_msg_info
#define laser_length_measure_print_data_info
#define laser_length_measure_print_error_info
#define laser_length_measure_save_process
//#define laser_length_measure_cheak

class laser_length_measure:public construct_cal,public galvo,public laser_zenturm_extract
{
public:
    laser_length_measure();
    ~laser_length_measure();

protected:
    int system_calibrate(std::vector<cv::Mat> cal_img);    
    void laser_zenturm_caltest(cv::Mat &src_img,cv::Point2f &zenturm);    
    //cv::Point3f laser_inject; //激光出射点在相机坐标系中的坐标
    double length_measure(cv::Mat src_img,double angle_ein,double angle_zwei); // 根据振镜的角度计算距离
    double length_measure_base(cv::Mat src_img);

protected:
    cv::Point3f zenturm_coordinate; //激光中心坐标
    math_geometry::geo_line_param laser_line; // 标定出射激光线在相机坐标系中的方程参数

private:
    // param for the laser length measure  

private:
    void calculate_line(double point_array[10][3]);
    void calculate_line(double point_array[10][3],math_geometry::geo_line_param &line);

};

#endif // laser_length_measure_HPP
