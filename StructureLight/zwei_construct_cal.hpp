/*
    文件等级：密一
    author:klug
    献给我亲爱的好友脚踏西瓜皮的胡安帕诺麦克
    start:2300423
    last:230625
*/

#ifndef zwei_construct_cal_HPP
#define zwei_construct_cal_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include "StructureLight/construct_img.hpp"
#include "StructureLight/zwei_construct_img.hpp"
#include "StructureLight/construct_cal.hpp"
#include "source.hpp"

#define zwei_construct_cal_print_msg_info
#define zwei_construct_cal_print_data_info
#define zwei_construct_cal_save_process
#define zwei_construct_cal_print_error_info

namespace calibrate_zwei_construct
{

class zwei_construct_cal:public img_zwei_construct::zwei_construct_img
{
public:
    zwei_construct_cal();
    ~zwei_construct_cal();

public:

protected:
    int system_calibrate();

protected:
    cv::Mat cameraMatrixLink=cv::Mat(3,3,CV_32FC1,cv::Scalar::all(0)); //相机内参
    cv::Mat cameraMatrixRicht=cv::Mat(3,3,CV_32FC1,cv::Scalar::all(0));
    cv::Mat distCoeffsLink=cv::Mat(1,5,CV_32FC1,cv::Scalar::all(0)); //畸变系数
    cv::Mat distCoeffsRicht=cv::Mat(1,5,CV_32FC1,cv::Scalar::all(0)); //畸变系数
    std::vector<cv::Mat> calibrateImgLink; //左相机标定图片
    std::vector<cv::Mat> calibrateImgRicht; //右相机标定图片
    cv::Mat zwei_rotate; //双目相机旋转关系
    cv::Mat zwei_transform; //双目相机平移关系
    cv::Mat essential; //本征矩阵
    cv::Mat fundamental; //基本矩阵

private:
    void relationship_calibrate(std::vector<cv::Mat> img_link,std::vector<cv::Mat> img_richt);
    int zwei_camera_calibrate(std::vector<cv::Mat> calibrate_img_link,std::vector<cv::Mat> calibrate_img_richt);
    int camera_calibrate(std::vector<cv::Mat> img_vector,cv::Mat &cameraMatrix,std::vector<cv::Mat> &extrinsic_matrix,cv::Mat &distCoeffs); //单相机内外参标定
    int camera_calibrate(std::vector<cv::Mat> img_vector,cv::Mat &cameraMatrix,std::vector<cv::Mat> &extrinsic_matrix,std::vector<std::vector<cv::Point2f>> &targetPoint,cv::Mat &distCoeffs); //单相机内外参标定
    int camera_calibrate(std::vector<cv::Mat> img_vector,cv::Mat &cameraMatrix,std::vector<std::vector<cv::Point2f>> &targetPoint,cv::Mat &distCoeffs); //单相机内外参标定

private:
    std::vector<std::vector<cv::Point3f>> objectPoints; //真实点坐标
    std::vector<cv::Point3f> objectPointsLink; //左，真实点坐标
    std::vector<cv::Point3f> objectPointsRicht; //右，真实点坐标
    std::vector<std::vector<cv::Point2f>> targetPointsLink; //左，角点在图像坐标系中的坐标
    std::vector<std::vector<cv::Point2f>> targetPointsRicht; //右，角点在图像坐标系中的坐标
    std::vector<cv::Mat> extrinsicMatrixLink; //左，相机外参
    std::vector<cv::Mat> extrinsicMatrixRicht; //右，相机外参
    std::vector<cv::Mat> extrinsicMatrix; //相机外参
    cv::Size board_size=cv::Size(6,9); //标定板的尺度
    cv::Size2f square_size=cv::Size2f(3.95,3.95);//标定板的格子大小
    std::vector<cv::Mat> rvecsMat;
    std::vector<cv::Mat> rotation_matrix;
    std::vector<cv::Mat> tvecsMat;
    std::string img_path;

};

};

#endif // zwei_construct_cal_HPP
