/*
    文件等级：密一
    献给美丽的阿尔忒弥斯
    author:klug
    start:230607
    last:230608
*/

#include "communication/communication.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>
#include <thread>
#include "source.hpp"

namespace communication_up
{

communication::communication()
{
    if(communication_open)
    {
        img_send_thread=std::thread(&communication::socket_server,this,server_ip,server_port_send);
    }
#ifdef communication_print_msg_info
    printf("open the server...\n");
#endif
}

communication::~communication()
{
    if(communication_open)
    {
        img_send_thread.join();
    }
#ifdef communication_print_msg_info
    printf("close the server...\n");
#endif
}

void communication::socket_server(const char* ip,int server_port)
{
#ifdef socket_print_msg_info
    printf("start the socker server...\n");
#endif
    int listen_sock;
    listen_sock=socket(AF_INET, SOCK_STREAM, 0);
    if(listen_sock==-1)
    {
#ifdef socket_printf_error_info
        perror("get listen socket error");
#endif
        exit(1);
    }

    int ret;
    if (setsockopt(listen_sock,SOL_SOCKET,SO_REUSEADDR,&ret,sizeof(ret))==-1)
    {
#ifdef socket_printf_error_info
        perror("set_reuse_addr error");
#endif
        exit(1);
    }

    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(server_ip);
    server_addr.sin_port = htons(server_port);

    if(bind(listen_sock, (sockaddr *)&server_addr, sizeof(server_addr))<0)
    {
#ifdef socket_print_error_info
        perror("listen_socket_bind error");
#endif
        exit(1);
    }

    if(listen(listen_sock,BACKLOG)==-1)
    {
#ifdef socket_print_error_info
        perror("listen_socket_listen error");
#endif
        exit(1);
    }

    sockaddr_in client;
    socklen_t addr_size=sizeof(sockaddr_in);
    while(1)
    {
#ifdef socket_print_msg_info
            printf("start accept\n");
#endif
        int socket_accept=accept(listen_sock, (sockaddr *)&client, &addr_size);
        if(socket_accept==-1)
        {
#ifdef socket_print_error_info
            printf("accept fail!!!\n");
#endif
            continue;
        }
        else
        {
#ifdef socket_print_msg_info
            printf("accept success...\n");
#endif
        }
        int buf_size=1024;

        while((recv(socket_accept,recvBuf,buf_size,0))>0)
        {
            if(send_flag)
            {
                send(socket_accept,sendBuf,15,0);
                send_flag=0;
            }
            else
            {

            }
        }
        close(socket_accept);
    }
}

/*
    图像编码
    @img_encode:编码的图片
*/
void communication::img_encode(cv::Mat img_encode)
{
    std::vector<uchar> data_encode;
    cv::imencode(".png",img_encode,data_encode);
    std::string str_encode(data_encode.begin(),data_encode.end());
#ifdef communication_print_data_info
    std::cout<<str_encode<<std::endl;
#endif
    char send_buf[str_encode.length()];
    //string转char[]
    for(int i=0;i<str_encode.length();i++)
    {
        send_buf[i]=str_encode[i];
    }
    send_buf[str_encode.length()]='\0';
}

/*
    图像编码
    @img_encode:编码的图片的地址
*/
void communication::img_encode_send(cv::Mat *img_encode)
{
    std::vector<uchar> data_encode;

    //发送标志位清除，进行编码操作
    if(!send_flag)
    {
        cv::imencode(".png",*img_encode,data_encode);
        std::string str_encode(data_encode.begin(),data_encode.end());
        std::cout<<str_encode<<std::endl;
#ifdef communication_print_data_info
        std::cout<<str_encode<<std::endl;
#endif
        char send_buf[str_encode.length()];
        //string转char[]
        for(int i=0;i<str_encode.length();i++)
        {
            send_buf[i]=str_encode[i];
        }
        send_buf[str_encode.length()]='\0';
    }

    //编码完成，准备发送
    send_flag=1;
}

void communication::img_decode()
{

}

};
