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

class imgcode:public buendia::socket_
{
public:
    imgcode();
    ~imgcode();

public:

protected:
    void Encode(cv::Mat srcImg,std::string type);
    void Decode(std::vector<uchar> imgCode,cv::Mat &resImg);

protected:
    std::vector<uchar> imgCode;

private:

private:

};

#endif // imgencode_H
