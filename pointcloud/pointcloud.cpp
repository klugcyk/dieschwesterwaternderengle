/*
    文件等级：密一
    author:klug
    献给我亲爱的好友何塞阿尔卡迪奥
    start:23322
    last:230322
*/

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include "img_process/construct_cal.hpp"
#include "pointcloud.hpp"

void point_cloud_generate(geo_line_param line)
{
    std::cout << "Test PCL !!!" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    for (int z=0;z<=10000;z++)
    {
        pcl::PointXYZ point;
        float step=0.1;
        point.x=-(line.a*(z*step-line.z0))/line.c+line.x0;
        point.y=-(line.b*(z*step-line.z0))/line.c+line.y0;
        point.z=z*step;
        //uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
        //static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        //point.rgb = *reinterpret_cast<float*>(&rgb);
        point_cloud_ptr->points.push_back (point);
    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
    point_cloud_ptr->height = 1;
    pcl::io::savePCDFile("pc.pcd",*point_cloud_ptr);
}
