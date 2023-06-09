/*
    文件等级：密一
    author:klug
    献给杜尔西内娅德尔托博索
    start:230511
    last:230524
*/

#ifndef cal_pose_HPP
#define cal_pose_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include "CameraPose/pose_img.hpp"
#include "source.hpp"

#define zwei_consutrct_img_print_msg_info
#define zwei_consutrct_img_print_data_info
#define zwei_consutrct_img_save_process
#define zwei_consutrct_img_print_error_info

namespace cal_cam_pose
{

class cam_pose:public img_pose::pose_img
{
public:
    cam_pose();
    ~cam_pose();

public:

protected:

protected:

private:

private:

};

};

#endif // cal_pose_HPP
