/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:230330
    last:230417
*/

#ifndef linelaser_hpp
#define linelaser_hpp

#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

//#define linelaser_print_msg_info
#define linelaser_print_error_info
//#define linelaser_print_data_info
#define linelaser_print_res_info
#define linelaser_save_end_img
#define linelaser_markcircle_on_img
#define linelaser_save_process_img

#define test

namespace xjg_image
{

struct distortion
{
    float k1;//径向畸变
    float k2;
    float k3;
    float p1;//切向畸变
    float p2;
};

//useless
void find_homo(std::vector<cv::Point2f> real,std::vector<cv::Point2f> extract);

//useful
distortion calibrate_iterate(cv::Mat src_img,std::vector<cv::Point2f> real,std::vector<cv::Point2f> extract,distortion first_dis,float step);
std::vector<cv::Point2f> point_reinject(std::vector<cv::Point2f> points,cv::Mat homo);
float error_calculate(std::vector<cv::Point2f> points);
void camera_calibrate(cv::Mat src_img);
void distortion_eliminate(cv::Mat src_img,cv::Mat &res_img,distortion dis);
void distortion_eliminate(std::vector<cv::Point2f> points_array,std::vector<cv::Point2f> &res_array,distortion dis);
void calibrate_initial(std::vector<cv::Point2f> points);
std::vector<cv::Point2f> array_point(std::vector<cv::Point2f> point_array,bool type);
void draw_point(cv::Mat &src_img,std::vector<cv::Point> zenturm,int offset_x,int offset_y);
cv::Mat laser_zenturm_line_ein(cv::Mat src_img,std::vector<cv::Point2f> &zenturm,bool type);
cv::Mat laser_zenturm_line_cal(cv::Mat src_img);
cv::Mat laser_zenturm_line(cv::Mat src_img);
void laser_extract(cv::Mat src_img,cv::Mat &res_img,std::vector<cv::Point2f> &zenturm);
void laser_endpoint(std::vector<cv::Point2f> zenturm,std::vector<cv::Point2f> &ep,int row,int offset_y);
void laser_endpoint(std::vector<cv::Point2f> zenturm,std::vector<cv::Point2f> &ep,bool type);
void laser_endpoint(std::vector<cv::Point2f> zenturm,std::vector<cv::Point2f> &ep,int offset_x,int offset_y,bool type);
void array_extract_points(std::vector<cv::Point2f> points_array,std::vector<cv::Point2f> &res_array,bool type);
void array_extract_points(std::vector<cv::Point2f> &points_array,bool type);

};

// xjg_image
extern std::vector<cv::Point2f> real_points;
extern std::vector<cv::Point2f> extract_points;//端点坐标，数组大小由标定物决定
extern std::vector<cv::Point2f> res_array_point;

// xjg_line_extract


#endif //linelaser_hpp
