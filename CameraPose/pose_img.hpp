/*
    author:klug
    献给我的心上人等待天使的妹妹
    start:230524
    last:230609
*/

#ifndef pose_img_HPP
#define pose_img_HPP

#include <opencv2/opencv.hpp>

#define pose_img_data_print_info
#define pose_img_msg_print_info
#define pose_img_error_print_info
#define pose_img_save_process

namespace img_pose
{

class pose_img
{
public:
    pose_img();
    ~pose_img();
    void img_test(cv::Mat src_img,cv::Mat &res_img);

public:

protected:
    void devernay(cv::Mat img,cv::Mat &res);

protected:
    std::vector<cv::Point2f> mark_zenturm_set; //标点中心点集合
    cv::Point2f mark_zenturm; //标点中心点

private:

private:

};

};

#endif // pose_img_HPP
