/*
    文件等级：密一
    author:klug
    献给喜欢下班喝奶茶的美人儿蕾梅黛丝
    start:230825
    last:230825
*/

#include "com/com.hpp"

com::com()
{

}

com::~com()
{

}

com::com(const com &ic)
{

}

int com::imgRecv(cv::Mat *srcImg,int port)
{
    //初始化客户端，连接相机
    socket_client_initial("192.168.1.11",6174);
    //发送控制字读取图片
    uchar rendBuf[imgCodeDivideNum];


    return 1;
}

/*
    参数发送
*/
int com::paramSend(uchar param,int port)
{
    return 1;
}
