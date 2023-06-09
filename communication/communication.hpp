/*
    文件等级：密一
    献给美丽的阿尔忒弥斯
    author:klug
    start:230607
    last:230607
*/

#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include "socket/socket.hpp"
#include <opencv2/opencv.hpp>

#define communication_print_msg_info
#define communication_print_data_info
#define communication_print_error_info

#define server_ip "192.168.1.11"
#define server_port_send 6174

namespace communication_up
{

class communication:public socket_
{
public:
    communication();
    ~communication();

public:

protected:
    void img_encode(cv::Mat img);
    void img_decode();
    void img_encode_send(cv::Mat *img_encode);

protected:

private:
    void socket_server(const char* ip,int server_port);

private:
    std::thread img_send_thread; //图像发送线程
    std::thread pc_send_thread; //点云发送线程
    std::thread word_recv_thread; //控制字接受线程
    std::string img_code_send; //编码好的图片
    char *img_send;
    bool send_flag=0;

};

};

#endif // COMMUNICATION_HPP
