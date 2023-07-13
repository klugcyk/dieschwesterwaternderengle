/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:230425
    last:230711
*/

#ifndef construct_HPP
#define construct_HPP

#include "StructureLight/construct_cal.hpp"
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>

#define construct_print_msg_info
#define construct_print_data_info
#define construct_save_process
#define construct_print_error_info
#define construct_img_mark

namespace structlight_construct
{

class construct:protected structlight_construct::construct_cal
{
public:
    construct();
    ~construct();
    void construct_test(std::vector<math_geometry::point3> p);
    void test_main();

public:

protected:
    int constructWithImg(cv::Mat src_img);
    std::vector<math_geometry::point3> construct_point_ein(std::vector<cv::Point2f> src_points);
    std::vector<math_geometry::point3> construct_point_multi(std::vector<std::vector<cv::Point2f>> src_points);
    std::vector<math_geometry::point3> construct_point(std::vector<std::vector<cv::Point2f>> src_points_array);
    void construct_sum(cv::Mat src_img);
    void construct_sum(cv::Mat src_img,int time);
    void constructShow(std::vector<math_geometry::point3> p1);
    void planeParamWrite();
    void planeParamRead();

protected:
    //pcl::PointXYZ pcl_point;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud;

private:

private:
    cv::Mat read_cameraMatrix_construct=cv::Mat(3,3,CV_32FC1,cv::Scalar::all(0)); //从json读取的相机内参
    std::vector<math_geometry::geo_plane_param> read_light_plane_construct; //从json读取的所有激光平面参数
    math_geometry::geo_plane_param read_light_plane_ein_construct; //从json读取的单个激光平面参数

};

namespace cuda
{

class construct:protected structlight_construct::cuda::construct_cal
{
public:
    construct();
    ~construct();
public:

protected:

protected:

private:

private:

};

};

};

#endif // construct_HPP
