/*
    文件等级：密一
    author:klug
    献给喜欢下班喝奶茶的美人儿蕾梅黛丝
    start:230825
    last:230828
*/

#include "com/com.hpp"

com::com()
{

}

com::com(const com &ic)
{

}

com::~com()
{

}
/*
    接收图片
    @ip:下位机IP地址
    @port:下位机发送数据端口
*/
void com::imgRecv(const char *ip,int port)
{
    recvTimes=imgRow*imgCol/imgCodeDivideNum;
    //初始化客户端，连接相机
    int socket_fd=socket_client_initial(ip,port);
    //发送控制字读取图片
    uchar readBuf[imgCodeDivideNum][recvTimes];

    while(1)
    {
        for(int recvCnt=0;recvCnt<recvTimes;recvCnt++)
        {
            int ret=send(socket_fd,"imgRequired",strlen("imgRequired"),0);
            ret=recv(socket_fd,&readBuf[0][recvCnt],imgCodeDivideNum,0);

            /*std::vector<uchar> codeTemp;
            codeTemp.clear();
            for(int codeCnt=0;codeCnt<imgCodeDivideNum;codeCnt++)
            {
                codeTemp.push_back(readBuf[codeCnt]);
            }
            imgCodeTemp.push_back(codeTemp);*/
        }

        for(int i=0;i<imgCodeDivideNum;i++)
        {
            for(int j=0;j<recvTimes;j++)
            {
                imgCode.push_back(readBuf[i][j]);
            }
        }
        usleep(10000);
    }
}

/*
    参数发送
*/
int com::paramSend(uchar param,int port)
{
    return 1;
}
