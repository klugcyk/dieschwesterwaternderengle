/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:230425
    last:230710
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
            read_path=read_img_path_cal;
            read_path+=std::to_string(i);
            read_path+=".png";
            cv::Mat temp_img=cv::imread(read_path);
            if(!temp_img.empty())
            {
#ifdef construct_print_data_info
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
        //system_calibrate(cal_img,laser_img,cv::Size(6,9),cv::Size2f(11.2,11.2)); //大
        planeParamWrite();
    }
    else
    {
        //从json文件加载标定数据
        std::string get_name[5]={"cameraParam","fx","fy","u0","v0"};
        double get_value[5];

        read_path=read_json_path;
        read_path+="camera_param.json";
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
        //从json文件加载平面参数

    }
}

construct::~construct()
{
#ifdef construct_cal_print_msg_info
    printf("close the construct...\n");
#endif

}

void construct::planeParamWrite()
{
    //保存标定参数到json文件
    double planeParamArray[4][laserLineCnt]; //标定平面参数
    for(size_t pCnt=0;pCnt<light_plane.size();pCnt++)
    {
        planeParamArray[0][pCnt]=light_plane[pCnt].A;
        planeParamArray[1][pCnt]=light_plane[pCnt].B;
        planeParamArray[2][pCnt]=light_plane[pCnt].C;
        planeParamArray[3][pCnt]=light_plane[pCnt].D;
#ifdef construct_print_data_info
        std::cout<<"planeParam 1 "<<pCnt+1<<" "<<planeParamArray[0][pCnt]<<std::endl;
        std::cout<<"planeParam 2 "<<pCnt+1<<" "<<planeParamArray[1][pCnt]<<std::endl;
        std::cout<<"planeParam 3 "<<pCnt+1<<" "<<planeParamArray[2][pCnt]<<std::endl;
        std::cout<<"planeParam 4 "<<pCnt+1<<" "<<planeParamArray[3][pCnt]<<std::endl;
        std::cout<<std::endl;
#endif
    }

    double planeParamArray_[4*laserLineCnt+1];
    planeParamArray_[0]=6174;
    int nameCnt=1;
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<laserLineCnt;j++)
        {
            planeParamArray_[nameCnt]=planeParamArray[i][j];
            nameCnt++;
        }
    }

    std::string planeName[4*laserLineCnt+1];
    std::string nameTemp[4]={"A","B","C","D"};
    planeName[0]="planeParam";
    nameCnt=1;
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<laserLineCnt;j++)
        {
            std::string name;
            name=nameTemp[i];
            name+=std::to_string(j+1);
            planeName[nameCnt]=name;
            //std::cout<<"name"<<nameCnt<<":="<<name<<std::endl;
            nameCnt++;
        }
    }

    write_path=write_json_path;
    write_path+="plane_param.json";
    mein_json::json_write(write_path,planeName,planeParamArray_,4*laserLineCnt+1);

    //保存相机参数到json文件
    double fx=cameraMatrix.at<double>(0,0);
    double fy=cameraMatrix.at<double>(1,1);
    double u0=cameraMatrix.at<double>(0,2);
    double v0=cameraMatrix.at<double>(1,2);

    std::string string_temp;
    std::string param_name[5]={"cameraParam","fx","fy","u0","v0"}; //相机参数
    double param_data[5]={6174.0,fx,fy,u0,v0};
    write_path=write_json_path;
    write_path+="camera_param.json";
    mein_json::json_write(write_path,param_name,param_data,5);//路径，名称，数据，位数
}

void construct::planeParamRead()
{

}

std::vector<math_geometry::point3> construct::construct_point_multi(std::vector<std::vector<cv::Point2f>> src_points)
{
    std::vector<math_geometry::point3> res_points;

    if(recal_flag) //重新标定用construct_cal类中的参数重建
    {
        double fx=cameraMatrix.at<double>(0,0);
        double fy=cameraMatrix.at<double>(1,1);
        double u0=cameraMatrix.at<double>(0,2);
        double v0=cameraMatrix.at<double>(1,2);

        for(size_t setCnt=0;setCnt<src_points.size();setCnt++)
        {
            for(size_t point_cnt=0;point_cnt<src_points[setCnt].size();point_cnt++)
            {
                math_geometry::point3 p;
                /*double temp1=light_plane[setCnt].D;
                temp1-=light_plane[setCnt].A*u0/fx;
                temp1-=light_plane[setCnt].B*v0/fy;
                double temp2=light_plane[setCnt].C;
                temp2+=light_plane[setCnt].A*src_points[setCnt][point_cnt].x/fx;
                temp2+=light_plane[setCnt].B*src_points[setCnt][point_cnt].y/fy;

                p.z=-temp1/temp2;
                p.x=(src_points[setCnt][point_cnt].x*p.z-u0)/fx;
                p.y=(src_points[setCnt][point_cnt].y*p.z-v0)/fy;*/
                double temp1=light_plane[setCnt].D;
                double temp2=light_plane[setCnt].C;
                temp2+=(src_points[setCnt][point_cnt].x-u0)/fx*light_plane[setCnt].A;
                temp2+=(src_points[setCnt][point_cnt].y-v0)/fy*light_plane[setCnt].B;
                p.z=-temp1/temp2;
                p.x=(src_points[setCnt][point_cnt].x-u0)*p.z/fx;
                p.y=(src_points[setCnt][point_cnt].y-v0)*p.z/fy;

                res_points.push_back(p);
            }
        }
    }
    else //不是重新标定的用读取到的参数重建
    {/*
        double fx=read_cameraMatrix.at<double>(0,0);
        double fy=read_cameraMatrix.at<double>(1,1);
        double u0=read_cameraMatrix.at<double>(0,2);
        double v0=read_cameraMatrix.at<double>(1,2);

        for(size_t setCnt=0;setCnt<src_points.size();setCnt++)
        {
        for(size_t point_cnt=0;point_cnt<src_points[setCnt].size();point_cnt++)
        {
            math_geometry::point3 p;
            double temp1=read_light_plane[setCnt].D;
            temp1+=read_light_plane[setCnt].A*src_points[setCnt][point_cnt].x/fx;
            temp1+=read_light_plane[setCnt].B*src_points[setCnt][point_cnt].y/fy;
            double temp2=light_plane[setCnt].A*u0/fx;
            temp2+=read_light_plane[setCnt].B*v0/fy;
            temp2-=read_light_plane[setCnt].C;

            p.z=temp1/temp2;
            p.x=(src_points[setCnt][point_cnt].x-u0*p.z)/fx;
            p.y=(src_points[setCnt][point_cnt].y-v0*p.z)/fy;

            res_points.push_back(p);
        }
        }*/
    }

    return res_points;
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
    pcl::PointXYZRGB p;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    for(int i=0;i<p1.size();++i)
    {
        p.x=p1[i].x;
        p.y=p1[i].y;
        p.z=p1[i].z;
        p.r=0;
        p.g=255;
        p.b=255;
        cloud->push_back(p);
    }
    //相机轴线
    for(int i=0;i<500;i++)
    {
        p.x=0;
        p.y=0;
        p.z=i;
        cloud->push_back(p);
    }

    for(int y=-30;y<30;y+=1)
    {
        for(int z=300;z<400;z+=1)
        {
            for(int i=0;i<laserLineCnt;i++)
            {
                p.x=-(light_plane[i].B*y+light_plane[i].C*z+light_plane[i].D)/light_plane[i].A;
                p.y=y;//-(light_plane[i].A*x+light_plane[i].C*z+light_plane[i].D)/light_plane[i].B;
                p.z=z;//-(light_plane[i].A*x+light_plane[i].B*y+light_plane[i].D)/light_plane[i].C;
                p.r=0;
                p.g=255;
                p.b=0;
                if(p.x<100&&p.x>-100)
                {
                    cloud->push_back(p);
                }
            }
        }
    }


    // 保存点云数据
    const std::string saved_pcd_path="/home/klug/img/construct/cloud_test.pcd";
    bool binary_mode=false;
    pcl::io::savePCDFile(saved_pcd_path, *cloud, binary_mode);
    // 显示点云数据
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer1->setBackgroundColor(0,0,0);
    viewer1->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");

    while(!viewer1->wasStopped())
    {
        viewer1->spinOnce(100);
        usleep(5000);
    }
}

/*
    三条线三维重建测试
    @p1:从图像中提取，利用相机模型计算得到的点集合
*/
void construct::constructShow(std::vector<math_geometry::point3> p1)
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
    const std::string saved_pcd_path="/home/klug/img/construct/cloudShow.pcd";
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
int construct::constructWithImg(cv::Mat src_img)
{

    return 1;
}

namespace cuda
{

};

};
