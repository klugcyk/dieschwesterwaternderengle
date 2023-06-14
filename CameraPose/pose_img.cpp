/*
    文件等级:密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:230524
    last:230609
*/

#include "pose_img.hpp"

namespace img_pose
{

pose_img::pose_img()
{
#ifdef pose_img_msg_print_info
    printf("open the pose_img...\n");
#endif
}

pose_img::~pose_img()
{
#ifdef pose_img_msg_print_info
    printf("close the pose_img...\n");
#endif
}

/*
    图像亚像素边缘提取
*/
void pose_img::devernay(cv::Mat img,cv::Mat &res)
{

}

/*
    图像算法测试
    @src_img:
    @res_img:
*/
void pose_img::img_test(cv::Mat src_img,cv::Mat &res_img)
{
    cv::Mat temp_img;
    cv::GaussianBlur(src_img,temp_img,cv::Size(5,5),5);
    cv::cvtColor(temp_img,res_img,cv::COLOR_BGR2GRAY);

}

};
