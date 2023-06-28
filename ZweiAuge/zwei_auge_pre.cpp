/*
    文件等级：密一
    author:klug
    献给不喜欢我的弗里德里希冯海因洛特
    start:230627
    last:230628
*/

#include "ZweiAuge/zwei_auge_pre.hpp"

namespace zweiauge
{

zwei_auge_pre::zwei_auge_pre()
{

}

zwei_auge_pre::~zwei_auge_pre()
{

}

void zwei_auge_pre::zweiAugeCal()
{
    std::vector<cv::Mat> linkImg,richtImg;
    cameraCalibrate(linkImg,cameraMatrixLink,distCoeffsLink);
    cameraCalibrate(richtImg,cameraMatrixRicht,distCoeffsRicht);

}

};
