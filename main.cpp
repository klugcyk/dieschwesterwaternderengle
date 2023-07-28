/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹，她的身旁常伴月光
    start:221129
    last:230727
*/

#include <QApplication>
#include "source.hpp"
#include <sys/time.h> //系统时间
#include "StructureLight/zwei_construct.hpp"
#include "StructureLight/construct.hpp"

//#define zweiChoose

bool link_update=0;
bool richt_update=0;
std::string read_path;
std::string write_path;
bool recal_flag=1; //重新标定标志位,1:重新标定，0:读取标定参数
bool cal_done=~recal_flag; //标定完成标志位
bool communication_open=0; //开启通讯标志
bool camera_continue_switch=0; //开启相机连续采集图像

int main(int argc, char *argv[])
{
#ifdef zweiChoose
    ZweiConstruct::construct c;
#else
    structlight_construct::construct c;
#endif
    return 1;
}
