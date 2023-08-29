/*
    文件等级：密一
    author:klug
    献给不喜欢我的弗雷德里希冯海因洛特
    start:230825
    last:230825
*/

#ifndef imgencode_H
#define imgencode_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include "socket/socket.hpp"
#include <iostream>
#include <thread>



#define imgEncodePrintError
#define imgEncodePrintDate
#define imgEncodePrintMsg

class imgcode
{
public:
    imgcode();
    imgcode(cv::Mat srcImg);
    imgcode(const imgcode &ic);
    ~imgcode();

public:

protected:
    void Encode(cv::Mat srcImg,std::string type);
    void Decode(std::vector<uchar> imgCode,cv::Mat &resImg);
    void codeEnDivide();
    void codeDeDivide();

protected:
    std::vector<uchar> sendImgCode; //图像编码字符串
    std::vector<uchar> recvImgCode; //图像编码字符串
    std::vector<std::vector<uchar>> recvImgCodeDivide; //分多次发送的图像编码字符串
    const int imgCodeDivideNum=5000;

private:

private:

};

#endif // imgencode_H
