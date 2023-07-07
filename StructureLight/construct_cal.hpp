/*
    文件等级：密一
    author:klug
    献给我亲爱的好友路易斯恩里克
    start:230220
    last:230705
*/

#ifndef construct_cal_HPP
#define construct_cal_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include "StructureLight/construct_img.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "math/geometry.hpp"
#include "math/least_sqaure.hpp"

#define construct_cal_print_msg_info
//#define construct_cal_print_data_info
#define construct_cal_save_process
#define construct_cal_print_error_info

namespace structlight_construct
{

class construct_cal:public structlight_construct::construct_img
{
public:
    construct_cal();
    ~construct_cal();

public:

protected:
    int system_calibrate(std::vector<cv::Mat> src_img,std::vector<cv::Mat> laser_img,cv::Size chess_size,cv::Size2f chess_length);
    int system_calibrate(std::vector<cv::Mat> src_img,std::vector<cv::Mat> laser_img);
    void cal_test();
    std::vector<cv::Mat> camera_calibrate(std::vector<cv::Mat> img_vector,cv::Size chess_size,cv::Size2f chess_length); //相机内外参标定

protected:
    cv::Mat cameraMatrix=cv::Mat(3,3,CV_32FC1,cv::Scalar::all(0)); //相机内参
    cv::Mat distCoeffs=cv::Mat(1,5,CV_32FC1,cv::Scalar::all(0)); //畸变系数
    std::vector<math_geometry::geo_plane_param> light_plane; //所有激光平面参数
    math_geometry::geo_plane_param light_plane_ein; //单个激光平面参数
    std::vector<cv::Mat> undistort_img_vector;
    //读取得到的数据
    cv::Mat read_cameraMatrix=cv::Mat(3,3,CV_32FC1,cv::Scalar::all(0)); //相机内参
    cv::Mat read_distCoeffs=cv::Mat(1,5,CV_32FC1,cv::Scalar::all(0)); //畸变系数
    std::vector<math_geometry::geo_plane_param> read_light_plane; //所有激光平面参数
    math_geometry::geo_plane_param read_light_plane_ein; //单个激光平面参数

private:
    void assign_points(std::vector<cv::Point2f> all_points,std::vector<std::vector<cv::Point2f>> &assigned_points); //对获取到的中线点按激光线分类
    void lightsource_calibrate(std::vector<cv::Mat> img_chess,std::vector<cv::Mat> img_laser); //激光平面位置标定相对于相机
    void lightsourceCalibrate(std::vector<cv::Mat> img_chess,std::vector<cv::Mat> img_laser); //激光平面位置标定相对于相机
    void lightsource_calibrate_ein(int img_cnt,std::vector<cv::Mat> img_laser); //单条激光平面位置标定相对于相机
    math_geometry::geo_plane_param extrinsic2plane(cv::Mat extrinsic); //外参转平面方程
    std::vector<cv::Mat> camera_calibrate(std::vector<cv::Mat> img_vector); //相机内外参标定
    //std::vector<cv::Mat> camera_calibrate(std::vector<cv::Mat> img_vector,cv::Size chess_size,cv::Size2f chess_length); //相机内外参标定
    void line_point_array(std::vector<Eigen::Vector3d> points,std::vector<std::vector<Eigen::Vector3d>> &array_points);

private:
    std::vector<std::vector<cv::Point2f>> assign_zenturm_points; //按激光线分类的中心线点
    std::vector<cv::Point2f> all_zenturm_points; //提取的所有激光线中心点
    std::vector<cv::Mat> extrinsic_matrix; //相机外参
    cv::Size board_size=cv::Size(6,9); //标定板的尺度
    cv::Size2f square_size=cv::Size2f(6.96,6.96);//标定板的格子大小 3.95
    std::vector<cv::Mat> rvecsMat;
    std::vector<cv::Mat> rotation_matrix;
    std::vector<cv::Mat> tvecsMat;
    //Eigen::Matrix3d cameraMatrix_inverse; //相机内参的逆
    std::vector<Eigen::Vector3d> zenturm_camera; //相机坐标系中，激光线的点
    std::vector<std::vector<Eigen::Vector3d>> zenturm_camera_array; //相机坐标系中，多条激光线的点，

};

};

#endif // construct_cal_HPP
