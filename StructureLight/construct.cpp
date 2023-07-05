/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:230425
    last:230705
*/

#include "StructureLight/construct.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "source.hpp"
#include "gene_operate/json_operate.hpp"

namespace structlight_construct
{

construct::construct()
{
#ifdef construct_cal_print_msg_info
    printf("open the construct...\n");
#endif

    if(recal_flag)//标定
    {
        std::vector<cv::Mat> cal_img;
        std::vector<cv::Mat> laser_img;

        // 加载相机标定用的图片
        for(int i=1;i<=cal_img_num;i++)
        {
            read_path=read_img_path_cal;
            read_path+=std::to_string(i);
            read_path+=".png";
            cv::Mat temp_img=cv::imread(read_path);
            if(!temp_img.empty())
            {
#ifdef construct_print_error_info
                printf("push back image %d\n",i);
#endif
                cal_img.push_back(temp_img);
            }
            else
            {
#ifdef construct_print_error_info
                printf("no image %d find\n",i);
#endif
            }
        }

        for(int i=126;i<=130;i++)
        {
            read_path=read_img_path;
            read_path+=std::to_string(i);
            read_path+=".png";
            cv::Mat temp_img=cv::imread(read_path);
            if(!temp_img.empty())
            {
#ifdef construct_print_error_info
                printf("push back image %d\n",i);
#endif
                laser_img.push_back(temp_img);
            }
            else
            {
#ifdef construct_print_error_info
                printf("no image %d find\n",i);
#endif
            }
        }
        //system_calibrate(cal_img,laser_img,cv::Size(6,9),cv::Size2f(3.95,3.95)); //小
        system_calibrate(cal_img,laser_img,cv::Size(6,9),cv::Size2f(6.96,6.96)); //大

        //保存标定参数到json文件
        double fx=cameraMatrix.at<double>(0,0);
        double fy=cameraMatrix.at<double>(1,1);
        double u0=cameraMatrix.at<double>(0,2);
        double v0=cameraMatrix.at<double>(1,2);
        double pa=light_plane_ein.A;
        double pb=light_plane_ein.B;
        double pc=light_plane_ein.C;
        double pd=light_plane_ein.D;

        std::string string_temp;
        std::string param_name[9]={"camera_param","fx","fy","u0","v0","A","B","C","D"}; //相机参数
        double param_data[9]={2,fx,fy,u0,v0,pa,pb,pc,pd};
        write_path=write_json_path;
        write_path+="system_param.json";
        mein_json::json_write(write_path,param_name,param_data,9);
    }
    else
    {
        //从json文件加载标定数据
        std::string get_name[9]={"camera_param","fx","fy","u0","v0","A","B","C","D"};
        double get_value[9];

        read_path=read_json_path;
        read_path+="system_param.json";
        mein_json::json_read(read_path,get_name,get_value);
        read_cameraMatrix_construct.at<float>(0,0)=get_value[1];
        read_cameraMatrix_construct.at<float>(1,1)=get_value[2];
        read_cameraMatrix_construct.at<float>(0,2)=get_value[3];
        read_cameraMatrix_construct.at<float>(1,2)=get_value[4];
        read_cameraMatrix_construct.at<float>(2,2)=1;

#ifdef construct_cal_print_data_info
        std::cout<<"read_camera_Matrix"
                <<read_cameraMatrix_construct
                <<std::endl;
#endif

#ifdef construct_cal_print_msg_info
        std::cout<<"read achieve..."
                <<std::endl;
#endif
    }
}

construct::~construct()
{
#ifdef construct_cal_print_msg_info
    printf("close the construct...\n");
#endif

}

std::vector<math_geometry::point3> construct::construct_point(std::vector<std::vector<cv::Point2f>> src_points_array)
{
    std::vector<math_geometry::point3> res_points;

    if(recal_flag) //重新标定用construct_cal类中的参数重建
    {
        double fx=cameraMatrix.at<double>(0,0);
        double fy=cameraMatrix.at<double>(1,1);
        double u0=cameraMatrix.at<double>(0,2);
        double v0=cameraMatrix.at<double>(1,2);

        for(int array_cnt=0;array_cnt<src_points_array.size();array_cnt++)
        {
            std::vector<cv::Point2f> points=src_points_array[array_cnt];
            for(size_t point_cnt=0;point_cnt<points.size();point_cnt++)
            {
                math_geometry::point3 p;
                double temp1=light_plane_ein.D;
                temp1+=light_plane_ein.A*points[point_cnt].x/fx;
                temp1+=light_plane_ein.B*points[point_cnt].y/fy;
                double temp2=light_plane_ein.A*u0/fx;
                temp2+=light_plane_ein.B*v0/fy;
                temp2-=light_plane_ein.C;

                p.z=temp1/temp2;
                p.x=(points[point_cnt].x-u0*p.z)/fx;
                p.y=(points[point_cnt].y-v0*p.z)/fy;

                res_points.push_back(p);
            }
        }
    }
    else //不是重新标定的用读取到的参数重建
    {
        double fx=read_cameraMatrix.at<double>(0,0);
        double fy=read_cameraMatrix.at<double>(1,1);
        double u0=read_cameraMatrix.at<double>(0,2);
        double v0=read_cameraMatrix.at<double>(1,2);

        for(int array_cnt=0;array_cnt<src_points_array.size();array_cnt++)
        {
            std::vector<cv::Point2f> points=src_points_array[array_cnt];
            for(size_t point_cnt=0;point_cnt<points.size();point_cnt++)
            {
                math_geometry::point3 p;
                double temp1=read_light_plane_ein.D;
                temp1+=read_light_plane_ein.A*points[point_cnt].x/fx;
                temp1+=read_light_plane_ein.B*points[point_cnt].y/fy;
                double temp2=read_light_plane_ein.A*u0/fx;
                temp2+=read_light_plane_ein.B*v0/fy;
                temp2-=read_light_plane_ein.C;

                p.z=temp1/temp2;
                p.x=(points[point_cnt].x-u0*p.z)/fx;
                p.y=(points[point_cnt].y-v0*p.z)/fy;

                res_points.push_back(p);
            }
        }
    }

    return res_points;
}

/*
    单条激光线，点集合三维重建
    @src_points:从图像中提取到的二维点
    @返回值:res_points，结果三维的点集合
*/
std::vector<math_geometry::point3> construct::construct_point_ein(std::vector<cv::Point2f> src_points)
{
    std::vector<math_geometry::point3> res_points;

    if(recal_flag) //重新标定用construct_cal类中的参数重建
    {
        double fx=cameraMatrix.at<double>(0,0);
        double fy=cameraMatrix.at<double>(1,1);
        double u0=cameraMatrix.at<double>(0,2);
        double v0=cameraMatrix.at<double>(1,2);

        for(size_t point_cnt=0;point_cnt<src_points.size();point_cnt++)
        {
            math_geometry::point3 p;
            double temp1=light_plane_ein.D;
            temp1+=light_plane_ein.A*src_points[point_cnt].x/fx;
            temp1+=light_plane_ein.B*src_points[point_cnt].y/fy;
            double temp2=light_plane_ein.A*u0/fx;
            temp2+=light_plane_ein.B*v0/fy;
            temp2-=light_plane_ein.C;

            p.z=temp1/temp2;
            p.x=(src_points[point_cnt].x-u0*p.z)/fx;
            p.y=(src_points[point_cnt].y-v0*p.z)/fy;

            res_points.push_back(p);
        }
    }
    else //不是重新标定的用读取到的参数重建
    {
        double fx=read_cameraMatrix.at<double>(0,0);
        double fy=read_cameraMatrix.at<double>(1,1);
        double u0=read_cameraMatrix.at<double>(0,2);
        double v0=read_cameraMatrix.at<double>(1,2);

        for(size_t point_cnt=0;point_cnt<src_points.size();point_cnt++)
        {
            math_geometry::point3 p;
            double temp1=read_light_plane_ein.D;
            temp1+=read_light_plane_ein.A*src_points[point_cnt].x/fx;
            temp1+=read_light_plane_ein.B*src_points[point_cnt].y/fy;
            double temp2=light_plane_ein.A*u0/fx;
            temp2+=read_light_plane_ein.B*v0/fy;
            temp2-=read_light_plane_ein.C;

            p.z=temp1/temp2;
            p.x=(src_points[point_cnt].x-u0*p.z)/fx;
            p.y=(src_points[point_cnt].y-v0*p.z)/fy;

            res_points.push_back(p);
        }
    }

    return res_points;
}

/*
    三条线三维重建测试
    @p1:从图像中提取，利用相机模型计算得到的点集合
*/
void construct::construct_test(std::vector<math_geometry::point3> p1)
{
    pcl::PointXYZ p;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for(int i=0;i<p1.size();++i)
    {
        p.x=p1[i].x;
        p.y=p1[i].y;
        p.z=p1[i].z;
        cloud->push_back(p);
    }

    // 保存点云数据
    const std::string saved_pcd_path="/home/klug/img/construct/cloud_test.pcd";
    bool binary_mode=false;
    pcl::io::savePCDFile(saved_pcd_path, *cloud, binary_mode);
    // 显示点云数据
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer1->setBackgroundColor(0,0,0);
    viewer1->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

    while(!viewer1->wasStopped())
    {
        viewer1->spinOnce(100);
        usleep(5000);
    }
}

/*
    最终三维重建
    @src_img:包含多条激光线的图像
*/
void construct::construct_sum(cv::Mat src_img)
{
    //图像处理提取中心点

    //利用相机模型计算点的三维坐标

    //生成点云

}

/*
    最终三维重建
    @src_img:包含多条激光线的图像
    @time:时间戳，ms
*/
void construct::construct_sum(cv::Mat src_img,int time)
{
    //图像处理提取中心点

    //利用相机模型计算点的三维坐标

    //生成点云

}

/*
    最终三维重建
    @src_img:包含多条激光线的图像
    @返回值：1 重建成功，-1 重建失败
*/
int construct::construct_with_img(cv::Mat src_img)
{
    return 1;
}

};
