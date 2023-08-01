/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹，她的眼神常带月光
    start:230511
    last:230731
*/

#pragma once // 只编译一次

#ifndef source_hpp
#define source_hpp

#define klug_improve
#define algorithm_test

#include <string>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>

#define addLibPath "/home/klug/sakura/" //加载动态链接库的路径
#define read_img_path "/home/klug/img/construct/"
#define read_img_path_cal "/home/klug/img/construct/cal/"
#define write_img_path "/home/klug/img/construct/"
#define write_img_path_undistort "/home/klug/img/construct/undistort/"
#define zwei_read_img_path "/home/klug/img/zwei_construct/"
#define zwei_write_img_path "/home/klug/img/zwei_construct/"
#define save_file_path "/home/klug/"
#define undistort_img_path "/home/klug/img/construct/undistort/"
#define read_json_path "/home/klug/"
#define write_json_path "/home/klug/"

#define zwei_cal_img_num 30
#define cal_img_num 30 //加载标定图片的数量
#define laserLineCnt 2 //结构光硬件上光线的条数
#define cameraCnt 1 //相机数量
#define laserLengthThreshold 50 //标定时选择ROI激光线长度阈值

#define useRedLaser //使用红色激光
//#define useBlueLaser //使用蓝色激光

extern std::string read_path;
extern std::string write_path;
extern bool recal_flag;
extern bool cal_done;
extern bool communication_open;
extern bool camera_continue_switch;
extern bool link_update;
extern bool richt_update;

//extern std::mutex continue_lock; //相机连续采集图像线程锁

#endif //source_hpp
