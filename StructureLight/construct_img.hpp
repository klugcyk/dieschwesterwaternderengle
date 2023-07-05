/*
    文件等级：密一
    author:klug
    献给不喜欢我的弗雷德里希冯海因洛特
    start:230215
    last:230705
*/

#ifndef contrruct_img_HPP
#define contrruct_img_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "math/geometry.hpp"
#include "math/least_sqaure.hpp"

#define construct_img_print_msg_info
#define construct_img_print_data_info
#define construct_img_print_error_info
#define construct_img_save_img
#define construct_img_mark

namespace structlight_construct
{

class construct_img
{
public:
    construct_img();
    construct_img(int r,int g,int b);
    ~construct_img();

public:

protected:
    //useless
    cv::Mat laser_zenturm_line_zwei(cv::Mat src_img,std::vector<cv::Point> &zenturm,bool type); //type=0,1
    void construct_img_test(cv::Mat src_img,cv::Mat &res_img,std::vector<cv::Point2f> &points);
    //useful
    void grid_extract_preprocess(cv::Mat src_img,cv::Mat &res_img,int p1,int p2);
    void grid_extract(cv::Mat src_img,cv::Mat &res_img);
    void convolution_grid_extract(cv::Mat src_img,cv::Mat &res_img,int kernel_size,int threshold);
    cv::Mat laser_zenturm_line(cv::Mat src_img,cv::Mat &res_img);
    int laserZenturmLineMultiCal(cv::Mat src_img,cv::Mat &res_img); //标定时，激光中心线提取
    int laserZenturmLineMulti(cv::Mat src_img,cv::Mat &res_img); //三维重建时，激光中心线提取
    int laser_zenturm_line_zwei(cv::Mat src_img,cv::Mat &res_img);

protected:
    std::vector<cv::Point2f> zenturm_line; //激光中心线点集合(单条)
    std::vector<std::vector<cv::Point2f>> zenturm_line_array; //激光中心线点集合(所有激光中心线)
    cv::Mat img_from_memory;//从内存中读取的图片

private:
    void savePointArray(std::vector<cv::Point2f> zenturm,int offset_x,int offset_y);
    void point_array(std::vector<cv::Point2f> points,std::vector<std::vector<cv::Point2f>> &points_array); //滤除激光中心线中不需要的点
    void point_array(std::vector<cv::Point2f> points,std::vector<std::vector<cv::Point2f>> *points_array); //滤除激光中心线中不需要的点
    void point_filter(std::vector<cv::Point2f> &points); //滤除激光中心线中不需要的点
    void draw_point(cv::Mat &src_img,std::vector<cv::Point2f> zenturm,int offset_x,int offset_y);
    void save_point(std::vector<cv::Point2f> zenturm,int offset_x,int offset_y);
    void draw_point(cv::Mat &src_img,std::vector<cv::Point2f> zenturm, cv::Scalar color);
    int circle_detect(std::vector<cv::Point> contour,int threshold,int step);
    cv::Mat laser_zenturm_line_ein(cv::Mat src_img,std::vector<cv::Point2f> &zenturm,bool type); //type=0,1
    cv::Mat zenturm_abandon(cv::Mat img1,cv::Mat img2,int length); //去除图像上结构光中心过亮的区域
    void getimgfrommeony(uchar *pixel_add,int row,int col,int channels);
    void getimgfrommeony(double *pixel_add,int row,int col,int channels);

private:
    int blue_threshold=250;
    int red_threshold=250;
    int green_threshold=250;

};

};

#endif // construct_img_HPP
