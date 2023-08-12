/*
    文件等级：密一
    author:klug
    献给杜尔西内娅德尔托博索
    start:230825
    last:230825
*/

#include "pointCloud/pointCloudExtract.hpp"

pointCloudExtract::pointCloudExtract()
{

}

pointCloudExtract::~pointCloudExtract()
{

}

/*
    从三通道图像提取点云数据
    @srcImgXYZ:图像数据,记录位置信息
    @srcImgBGR:图像数据,记录颜色信息
*/
void pointCloudExtract::img2pointcloud(cv::Mat srcImgXYZ,cv::Mat srcImgBGR)
{
    if(srcImgXYZ.channels()!=3||srcImgBGR.channels()!=3)
    {
#ifdef pointCloudExtractPrintError
        printf("ERROR: img can not represent the point cloud\n");
#endif
        return;
    }

    //清除上次的数据
    pcRGB.clear();
    pcl::PointXYZRGB p;
    for(size_t row=0;row<srcImgXYZ.rows;row++)
    {
        for(size_t col=0;col<srcImgXYZ.cols;col++)
        {
            p.x=srcImgXYZ.at<cv::Vec3b>(row,col)[0];
            p.y=srcImgXYZ.at<cv::Vec3b>(row,col)[1];
            p.z=srcImgXYZ.at<cv::Vec3b>(row,col)[2];
            p.r=srcImgBGR.at<cv::Vec3b>(row,col)[0];
            p.g=srcImgBGR.at<cv::Vec3b>(row,col)[1];
            p.b=srcImgBGR.at<cv::Vec3b>(row,col)[2];
            pcRGB.push_back(p);
        }
    }
}

/*
    从三通道图像提取点云数据
    @srcImg:图像数据,记录位置信息
*/
void pointCloudExtract::img2pointcloud(cv::Mat srcImg)
{
    if(srcImg.channels()!=3)
    {
#ifdef pointCloudExtractPrintError
        printf("ERROR: img can not represent the point cloud\n");
#endif
        return;
    }

    //清除上次的数据
    pc.clear();
    pcl::PointXYZ p;
    for(size_t row=0;row<srcImg.rows;row++)
    {
        for(size_t col=0;col<srcImg.cols;col++)
        {
            p.x=srcImg.at<cv::Vec3b>(row,col)[0];
            p.y=srcImg.at<cv::Vec3b>(row,col)[1];
            p.z=srcImg.at<cv::Vec3b>(row,col)[2];
            pc.push_back(p);
        }
    }
}
