/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹，她的眼神常带月光
    start:230511
    last:230608
*/

#pragma once // 只编译一次

#ifndef source_hpp
#define source_hpp

#include <string>
#include <mutex>

extern bool link_update;
extern bool richt_update;

#define read_img_path "/home/klug/img/construct/"
#define write_img_path "/home/klug/img/construct/"
#define zwei_read_img_path "/home/klug/img/zwei_construct/"
#define zwei_write_img_path "/home/klug/img/zwei_construct/"
#define save_file_path "/home/klug/"
#define undistort_img_path "/home/klug/img/construct/undistort/"
#define read_json_path "/home/klug/"
#define write_json_path "/home/klug/"

extern std::string read_path;
extern std::string write_path;
extern bool recal_flag;
extern bool cal_done;
extern bool communication_open;
extern bool camera_continue_switch;

//extern std::mutex continue_lock; //相机连续采集图像线程锁

#endif //source_hpp
