/*
    文件等级：密一
    author:klug
    献给不喜欢我的弗雷德里希冯海因洛特
    start:230825
    last:230825
*/

#include "imgProcess/imgCode.hpp"

imgcode::imgcode()
{

}

imgcode::~imgcode()
{

}

/*
    灰度图编码
    @img:编码的图片
    @type:图像文件后缀
*/
void imgcode::Encode(cv::Mat srcImg,std::string type)
{
    imgCode.clear();

    if(type=="jpg")
    {
        cv::imencode(".jpg",srcImg,imgCode);
    }
    else if(type=="bmp")
    {
        cv::imencode(".bmp",srcImg,imgCode);
    }
    else if(type=="png")
    {
        cv::imencode(".png",srcImg,imgCode);
    }
    else
    {
#ifdef imgEncodePrintError
        printf("ERROR: img type not set correct\n");
#endif
        return;
    }
}

/*
    灰度图解码
    @img:编码的图片
    @type:图像文件后缀
*/
void imgcode::Decode(std::vector<uchar> imgCode,cv::Mat &resImg)
{
    resImg=cv::imdecode(imgCode,CV_LOAD_IMAGE_COLOR);
}
