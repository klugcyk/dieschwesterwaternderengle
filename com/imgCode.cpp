/*
    文件等级：密一
    author:klug
    献给不喜欢我的弗雷德里希冯海因洛特
    start:230825
    last:230825
*/

#include "com/imgCode.hpp"

imgcode::imgcode(cv::Mat srcImg)
{
    Encode(srcImg,"png");
    codeEnDivide();
}

imgcode::imgcode()
{

}

imgcode::~imgcode()
{

}

imgcode::imgcode(const imgcode &ic)
{

}

/*
    灰度图编码
    @img:编码的图片
    @type:图像文件后缀
*/
void imgcode::Encode(cv::Mat srcImg,std::string type)
{
    sendImgCode.clear();

    if(type=="jpg")
    {
        cv::imencode(".jpg",srcImg,sendImgCode);
    }
    else if(type=="bmp")
    {
        cv::imencode(".bmp",srcImg,sendImgCode);
    }
    else if(type=="png")
    {
        cv::imencode(".png",srcImg,sendImgCode);
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
    数组无法开出很大的内存，做编码分割
*/
void imgcode::codeEnDivide()
{
    int readCnt=0;//取出编码计数
    std::vector<uchar> codeTemp;//缓存
    while(readCnt<sendImgCode.size())
    {
        for(size_t i=0;i<imgCodeDivideNum;i++)
        {
            uchar code=sendImgCode[readCnt];
            codeTemp.push_back(code);
            readCnt++;
        }
        recvImgCodeDivide.push_back(codeTemp);
    }
}

/*
    分多组编码传送的图像转换为一组
*/
void imgcode::codeDeDivide()
{
    recvImgCode.clear();
    for(size_t i=0;i<recvImgCodeDivide.size();i++)
    {
        for(size_t j=0;j<recvImgCodeDivide[i].size();j++)
        {
            recvImgCode.push_back(recvImgCodeDivide[i][j]);
        }
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
