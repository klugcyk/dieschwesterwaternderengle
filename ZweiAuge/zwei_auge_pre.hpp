/*
    文件等级：密一
    author:klug
    献给不喜欢我的弗里德里希冯海因洛特
    start:230627
    last:230628
*/

#ifndef zwei_auge_pre_H
#define zwei_auge_pre_H

#include <opencv2/opencv.hpp>
#include "camera/cameraGene.hpp"

namespace zweiauge
{

class zwei_auge_pre:protected cameraGene
{
public:
    zwei_auge_pre();
    ~zwei_auge_pre();

public:

protected:
    void zweiAugeCal(); //双目标定

protected:
    cv::Mat cameraMatrixLink=cv::Mat(3,3,CV_32FC1,cv::Scalar::all(0)); //相机内参
    cv::Mat cameraMatrixRicht=cv::Mat(3,3,CV_32FC1,cv::Scalar::all(0));
    cv::Mat distCoeffsLink=cv::Mat(1,5,CV_32FC1,cv::Scalar::all(0)); //畸变系数
    cv::Mat distCoeffsRicht=cv::Mat(1,5,CV_32FC1,cv::Scalar::all(0)); //畸变系数
    std::vector<cv::Mat> calibrateImgLink; //左相机标定图片
    std::vector<cv::Mat> calibrateImgRicht; //右相机标定图片
    cv::Mat zweiRotate; //双目相机旋转关系
    cv::Mat zweiTransform; //双目相机平移关系
    cv::Mat essential; //本征矩阵
    cv::Mat fundamental; //基本矩阵
    std::vector<cv::Point2f> pointsLink;//左相机中匹配到的特征点
    std::vector<cv::Point2f> pointsricht;//右相机中匹配到的特征点

private:

private:

};

};

#endif // zwei_auge_pre_H
