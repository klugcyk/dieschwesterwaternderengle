/*
    文件等级:密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:230425
    last:230731
*/

#include "StructureLight/construct.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "source.hpp"
#include "json_operate/json_operate.hpp"
#include "data_array/data_array.hpp"

namespace StructlightConstruct
{

construct::construct()
{
#ifdef construct_cal_print_msg_info
    printf("opened the construct...\n");
#endif

    if(recal_flag) //recalibrate
    {
        std::vector<cv::Mat> cal_img;
        std::vector<cv::Mat> laser_img;

        // load the image for calibrate
        for(int i=1;i<=cal_img_num;i++)
        {
            read_path=read_img_path_cal;
            read_path+=std::to_string(i);
            read_path+=".png";
            cv::Mat temp_img=cv::imread(read_path);
            if(!temp_img.empty())
            {
#ifdef construct_print_msg_info
                printf("push back image %d\n",i);
#endif
                cal_img.push_back(temp_img);
            }
            else
            {
#ifdef construct_print_msg_info
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
        // structure system calibrate,
        system_calibrate(cal_img,laser_img,cv::Size(8,11),cv::Size2f(5.00,5.00)); //大
        // save the calibrate data
        //planeParamWrite();
    }
    else
    {
        //load the calibrate data from json
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
        //load the plane parameter from jso

    }
    // 测试算法用
#ifdef algorithm_test
    cv::Mat img=cv::imread("/home/klug/img/construct/111.png");
    cv::cvtColor(img,img,cv::COLOR_BGR2GRAY);
    if(!img.empty())
    {
        cv::Mat res(img.rows,img.cols,CV_32FC1);
        float *zenturm;
        laserZenturmLineMulti(img,res,zenturm);
        cv::imwrite("/home/klug/img/construct/convolution.png",res);
    }

#endif
}

construct::~construct()
{
#ifdef construct_cal_print_msg_info
    printf("close the construct...\n");
#endif

}

void construct::planeParamWrite()
{
    //save calibrate to json file
    double planeParamArray[4][laserLineCnt]; //calibcate plane parameter
    for(size_t pCnt=0;pCnt<light_plane.size();pCnt++)
    {
        planeParamArray[0][pCnt]=light_plane[pCnt].A;
        planeParamArray[1][pCnt]=light_plane[pCnt].B;
        planeParamArray[2][pCnt]=light_plane[pCnt].C;
        planeParamArray[3][pCnt]=light_plane[pCnt].D;
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
            nameCnt++;
        }
    }

    write_path=write_json_path;
    write_path+="plane_param.json";
    mein_json::json_write(write_path,planeName,planeParamArray_,4*laserLineCnt+1);

    //save camera parameter to json
    double fx=cameraMatrix.at<double>(0,0);
    double fy=cameraMatrix.at<double>(1,1);
    double u0=cameraMatrix.at<double>(0,2);
    double v0=cameraMatrix.at<double>(1,2);

    std::string string_temp;
    std::string param_name[5]={"cameraParam","fx","fy","u0","v0"}; //camera parameter
    double param_data[5]={6174.0,fx,fy,u0,v0};
    write_path=write_json_path;
    write_path+="camera_param.json";
    mein_json::json_write(write_path,param_name,param_data,5);
}

void construct::planeParamRead()
{

}

std::vector<math_geometry::point3> construct::construct_point_multi(std::vector<std::vector<cv::Point2f>> src_points)
{
    std::vector<math_geometry::point3> res_points;

    if(recal_flag) //recalibrate in construct_cal
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
    else
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

    if(recal_flag)
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
    else
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
    single laser line, point set 3d construct
    @src_points:point in 2d
    @return:res_points
*/
std::vector<math_geometry::point3> construct::construct_point_ein(std::vector<cv::Point2f> src_points)
{
    std::vector<math_geometry::point3> res_points;

    if(recal_flag)
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
    else
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
    3 line 3d construct test
    @p1:extract from img use the camera model calculate the points set
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

    //camera axis
    for(int i=0;i<50;i++)
    {
        p.x=0;
        p.y=0;
        p.z=i;
        //cloud->push_back(p);
    }

    for(int y=-30;y<30;y+=1)
    {
        for(int z=300;z<400;z+=1)
        {
            for(int i=0;i<laserLineCnt;i++)
            {
                p.x=-(light_plane[i].B*y+light_plane[i].C*z+light_plane[i].D)/light_plane[i].A;
                p.y=y;
                p.z=z;
                p.r=0;
                p.g=255;
                p.b=0;
                if(p.x<100&&p.x>-100)
                {
                    //cloud->push_back(p);
                }
            }
        }
    }


    // save point cloud
    const std::string saved_pcd_path="/home/klug/img/construct/cloud_test.pcd";
    bool binary_mode=false;
    pcl::io::savePCDFile(saved_pcd_path, *cloud, binary_mode);
    // show point cloud
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
    3 line constructtest
    @p1:extract from image with the camera model
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

    // save point cloud data
    const std::string saved_pcd_path="/home/klug/img/construct/cloudShow.pcd";
    bool binary_mode=false;
    pcl::io::savePCDFile(saved_pcd_path, *cloud, binary_mode);
    // show point cloud data
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
    end construct
    @src_img:image with multi laser line
*/
void construct::construct_sum(cv::Mat src_img)
{
    //extract point line zenturm

    //calculate the 3d coordination

    //generate point cloud

}

/*
    end construct
    @src_img:image with multi laser line
    @time:time，ms
*/
void construct::construct_sum(cv::Mat src_img,int time)
{
    //extract point line zenturm

    //calculate the 3d coordination

    //generate point cloud

}

/*
    end construct
    @src_img:image with multi laser line
    @return:
*/
int construct::constructWithImg(cv::Mat src_img)
{

    return 1;
}

void construct::test_main()
{

}

};
