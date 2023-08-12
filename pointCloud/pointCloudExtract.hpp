/*
    文件等级：密一
    author:klug
    献给杜尔西内娅德尔托博索
    start:230825
    last:230825
*/

#ifndef pointcloudextract_H
#define pointcloudextract_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <thread>

#define pointCloudExtractPrintError
#define pointCloudExtractPrintDate
#define pointCloudExtractPrintMsg

class pointCloudExtract
{
public:
    pointCloudExtract();
    ~pointCloudExtract();

public:

protected:
    void img2pointcloud(cv::Mat srcImg);
    void img2pointcloud(cv::Mat srcImgXYB,cv::Mat srcImgBGR);

protected:
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::PointCloud<pcl::PointXYZRGB> pcRGB;

private:

private:

};

#endif // pointcloudextract_H
