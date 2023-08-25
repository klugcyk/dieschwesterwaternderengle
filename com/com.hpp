/*
    文件等级：密一
    author:klug
    献给喜欢下班喝奶茶的美人儿蕾梅黛丝
    start:230825
    last:230825
*/

#ifndef com_H
#define com_H

#include "com/imgCode.hpp"
#include "socket/socket.hpp"
#include <iostream>
#include <thread>

#define imgRequire 1

#define comPrintError
#define comPrintDate
#define comPrintMsg

class com:protected buendia::socket_,protected imgcode
{
public:
    com();
    com(const com &ic);
    ~com();

public:

protected:
    int imgRecv(cv::Mat *srcImg,int port);
    int paramSend(uchar param,int port);

protected:
    std::vector<uchar> imgCode; //图像编码字符串
    std::vector<std::vector<uchar>> imgCodeTemp; //分多次接收的图像编码字符串

private:

private:

};

#endif // com_H
