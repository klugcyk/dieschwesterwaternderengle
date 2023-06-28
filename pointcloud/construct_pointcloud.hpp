/*
    文件等级：密一
    author:klug
    献给我亲爱的好友何塞阿尔卡迪奥
    start:230220
    last:230510
*/

#ifndef contrruct_pointcloud_HPP
#define contrruct_pointcloud_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include "StructureLight/construct_cal.hpp"

#define construct_pointcloud_print_msg_info
#define construct_pointcloud_print_data_info
#define construct_pointcloud_save_process

namespace construct_pc
{

class construct_pointcloud:public structlight_construct::construct_cal
{
public:
    construct_pointcloud();
    ~construct_pointcloud();

public:

protected:

protected:

private:

private:

};

};

#endif // contrruct_pointcloud_HPP
