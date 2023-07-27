/*
    文件等级：密一
    author:klug
    献给我亲爱的好友梅尔基亚德斯
    start:230220
    last:230727
*/

#include "StructureLight/construct_cal.hpp"
#include "StructureLight/construct_img.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "math/geometry.hpp"
#include "math/least_sqaure.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "source.hpp"
#include "json_operate/json_operate.hpp"

namespace structlight_construct
{

construct_cal::construct_cal()
{
#ifdef construct_cal_print_msg_info
    printf("start the 3d construct calibrate...\n");
#endif
    if(!recal_flag)
    {
        //从json文件加载标定数据
        std::string get_name[9]={"camera_param","fx","fy","u0","v0","A","B","C","D"};
        double get_value[9];

        read_path=read_json_path;
        read_path+="system_param.json";
        mein_json::json_read(read_path,get_name,get_value);
        read_cameraMatrix.at<double>(0,0)=get_value[1];
        read_cameraMatrix.at<double>(1,1)=get_value[2];
        read_cameraMatrix.at<double>(0,2)=get_value[3];
        read_cameraMatrix.at<double>(1,2)=get_value[4];
        read_cameraMatrix.at<double>(2,2)=1;
        read_light_plane_ein.A=get_value[5];
        read_light_plane_ein.B=get_value[6];
        read_light_plane_ein.C=get_value[7];
        read_light_plane_ein.D=get_value[8];
#ifdef construct_cal_print_data_info
        printf("read camera fx:=%f\n",get_value[1]);
        printf("read camera fy:=%f\n",get_value[2]);
        printf("read camera u0:=%f\n",get_value[3]);
        printf("read camera v0:=%f\n",get_value[4]);
        printf("read light plane_ein A:=%f\n",get_value[5]);
        printf("read light plane_ein B:=%f\n",get_value[6]);
        printf("read light plane_ein C:=%f\n",get_value[7]);
        printf("read light plane_ein D:=%f\n",get_value[8]);
#endif
#ifdef construct_cal_print_msg_info
        printf("read the parameter successed...\n");
#endif
    }
    else
    {
#ifdef construct_cal_print_msg_info
        printf("choose the sys cal...\n");
#endif
    }
}

construct_cal::~construct_cal()
{
#ifdef construct_cal_print_msg_info
    printf("end the 3d construct calibrate...\n");
#endif

}

/*
    测试程序
*/
void construct_cal::cal_test()
{
    pcl::PointXYZRGB p;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // 标定点
    for(int i=0;i<zenturm_camera_array.size();i++)
    {
        std::vector<Eigen::Vector3d> zc=zenturm_camera_array[i];
        for(int pc=0;pc<zc.size();++pc)
        {
           p.x=zc[pc].x();
           p.y=zc[pc].y();
           p.z=zc[pc].z();

           p.r=255;
           p.g=0;
           p.b=0;

           cloud->push_back(p);
        }
    }

    // 相机轴线点
    for(int i=0;i<50;++i)
    {
         p.x=0;
         p.y=0;
         p.z=i;
         p.r=255;
         p.g=255;
         p.b=255;
         cloud->push_back(p);
    }

    // 激光平面点
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
            /*
            p.z=-(light_plane[1].A*x+light_plane[1].B*y+light_plane[1].D)/light_plane[1].C;
            p.r=0;
            p.g=255;
            p.b=0;
            if(p.z>0&&p.z<300)
            {
                cloud->push_back(p);
            }

            p.z=-(light_plane[2].A*x+light_plane[2].B*y+light_plane[2].D)/light_plane[2].C;
            p.r=0;
            p.g=255;
            p.b=0;
            if(p.z>0&&p.z<300)
            {
                cloud->push_back(p);
            }

            p.z=-(light_plane[3].A*x+light_plane[3].B*y+light_plane[3].D)/light_plane[3].C;
            p.r=0;
            p.g=255;
            p.b=0;
            if(p.z>0&&p.z<300)
            {
                cloud->push_back(p);
            }

            p.z=-(light_plane[4].A*x+light_plane[4].B*y+light_plane[4].D)/light_plane[4].C;
            p.r=0;
            p.g=255;
            p.b=0;
            if(p.z>0&&p.z<300)
            {
                cloud->push_back(p);
            }

            p.z=-(light_plane[5].A*x+light_plane[5].B*y+light_plane[5].D)/light_plane[5].C;
            p.r=0;
            p.g=255;
            p.b=0;
            if(p.z>0&&p.z<300)
            {
                cloud->push_back(p);
            }

            p.z=-(light_plane[6].A*x+light_plane[6].B*y+light_plane[6].D)/light_plane[6].C;
            p.r=0;
            p.g=255;
            p.b=0;
            if(p.z>0&&p.z<300)
            {
                cloud->push_back(p);
            }*/
        }
    }

    // 标定平面
    for (int i=25;i<extrinsic_matrix.size();++i)
    {
        math_geometry::geo_plane_param p1=extrinsic2plane(extrinsic_matrix[i]);
        for(int x=-100;x<100;x+=1)
        {
            for(int y=-50;y<50;y+=1)
            {
                p.x=x;
                p.y=y;
                p.z=-(p1.A*x+p1.B*y+p1.D)/p1.C;
                p.r=0;
                p.g=0;
                p.b=255;
                cloud->push_back(p);
            }
        }
    }

    // 保存点云数据
    const std::string saved_pcd_path="/home/klug/img/construct/cloud_tmp.pcd";
    bool binary_mode=false;
    pcl::io::savePCDFile(saved_pcd_path, *cloud, binary_mode);
    // 显示点云数据
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        usleep(5000);
    }
}

/*
    测试程序

int construct_cal::cal_test_()
{
    return light_plane.size();
}*/

/*
    结构光系统标定，包括光源标定和相机标定
    @src_img:相机标定用图像
    @laser_img:包含激光线图像，用于激光平面标定
    @chess_size:棋盘格大小
    @chess_length:棋盘格尺寸
    @返回值:1,标定成功，0，图像数量不足
*/
int construct_cal::system_calibrate(std::vector<cv::Mat> src_img,std::vector<cv::Mat> laser_img,cv::Size chess_size,cv::Size2f chess_length)
{
    if(src_img.size()<10||laser_img.size()<2)
    {
        return 0;
    }

    // 相机标定
    camera_calibrate(src_img,chess_size,chess_length);

    // 激光线标定
    lightsourceCalibrate(src_img,laser_img);

    cal_done=1;
#ifdef construct_cal_print_msg_info
    printf("system calibtate done...\n");
#endif

    return 1;
}

/*
    结构光系统标定，包括光源标定和相机标定
    @src_img:相机标定用图像
    @laser_img:包含激光线图像，用于激光平面标定
    @chess_size:棋盘格大小
    @chess_length:棋盘格尺寸
    @res:返回值状态
*/
int construct_cal::system_calibrate(std::vector<cv::Mat> src_img,std::vector<cv::Mat> laser_img)
{
    if(src_img.size()<8||laser_img.size()<2)
    {
        return 0;
    }

    camera_calibrate(src_img);
    //lightsource_calibrate_ein(5,laser_img);
    lightsource_calibrate(src_img,laser_img);

    cal_done=1;

#ifdef construct_cal_print_msg_info
    printf("system calibtate done...\n");
#endif

    return 1;
}

/*
    光源单个光平面参数标定，在相机坐标系下的坐标
    @img_cnt:第几个平面，按第几个平面提取出需要的某几张图片
    @img_laser:包含激光线的图像集合
*/
void construct_cal::lightsource_calibrate_ein(int img_cnt,std::vector<cv::Mat> img_laser)
{
    cv::Mat res(img_laser[0].rows,img_laser[0].cols,CV_8UC1); // 不使用，只为缓存函数处理的图片结果

    // 提取各张图像上激光中心线上点的坐标，保存到zenturm_camera
    zenturm_camera.clear();

    double fx=cameraMatrix.at<double>(0,0);
    double fy=cameraMatrix.at<double>(1,1);
    double u0=cameraMatrix.at<double>(0,2);
    double v0=cameraMatrix.at<double>(1,2);

    for(size_t img_cnt=0;img_cnt<img_laser.size();img_cnt++)
    {
        laser_zenturm_line(img_laser[img_cnt],res);

        // 相机外参转换成平面方程，标定板平面
        math_geometry::geo_plane_param p=extrinsic2plane(extrinsic_matrix[img_cnt+25]); //从第26张图片为激光平面标定用的标定板

        // 计算中线上点坐标在相机坐标系中的值
        for(size_t i=0;i<zenturm_line.size();i++)
        {
            double zc=-p.D/(p.A*(zenturm_line[i].x-u0)/fx+p.B*(zenturm_line[i].y-v0)/fy+p.C);
            double xc=zc*(zenturm_line[i].x-u0)/fx;
            double yc=zc*(zenturm_line[i].y-v0)/fy;
            zenturm_camera.push_back(Eigen::Vector3d(xc,yc,zc));
        }
    }

    // 最小二乘法计算平面方程
    light_plane_ein=math_leastsqaure::plane_calculate(zenturm_camera);

#ifdef construct_cal_print_data_info
    // 计算平面与相机轴线的角度
    float up=light_plane_ein.C*1;
    float l1=sqrt(light_plane_ein.A*light_plane_ein.A+light_plane_ein.B*light_plane_ein.B+light_plane_ein.C*light_plane_ein.C);
    float theat=abs(acos(up/l1)*57.3-90);

    std::cout<<"theat:="<<theat<<std::endl;
    std::cout<<std::endl;
    std::cout<<"A:="<<light_plane_ein.A<<std::endl;
    std::cout<<"B:="<<light_plane_ein.B<<std::endl;
    std::cout<<"C:="<<light_plane_ein.C<<std::endl;
    std::cout<<"D:="<<light_plane_ein.D<<std::endl;
    std::cout<<std::endl;
#endif
}

/*
    外参计算平面方程参数
    @extrinsic:标定得到的外参，4*4
    @res_plane:返回值，平面的参数
*/
math_geometry::geo_plane_param construct_cal::extrinsic2plane(cv::Mat extrinsic)
{
    math_geometry::geo_plane_param res_plane;

    res_plane.A=extrinsic.at<double>(0,2);//+extrinsic.at<double>(0,3);
    res_plane.B=extrinsic.at<double>(1,2);//+extrinsic.at<double>(1,3);
    res_plane.C=extrinsic.at<double>(2,2);//+extrinsic.at<double>(2,3);
    res_plane.D=-(res_plane.A*extrinsic.at<double>(0,3)+res_plane.B*extrinsic.at<double>(1,3)+res_plane.C*extrinsic.at<double>(2,3));

#ifdef construct_cal_print_data_info
    printf("extrinsic plane.a:=%f\n",res_plane.A);
    printf("extrinsic plane.b:=%f\n",res_plane.B);
    printf("extrinsic plane.c:=%f\n",res_plane.C);
    printf("extrinsic plane.d:=%f\n",res_plane.D);
#endif

    return res_plane;
}

/*
    二激光平面参数标定，激光线为两条是使用，两个激光平面
    @img_chess:包含棋盘格的图片,30张，从中提取后5张
    @img_laser:包含激光线的图片,5张
*/
void construct_cal::lightsource_calibrate(std::vector<cv::Mat> img_chess,std::vector<cv::Mat> img_laser)
{
    cv::Mat res; //useless

    //相机坐标系中，中心点的坐标值
    zenturm_camera_array.clear();
    std::vector<Eigen::Vector3d> zc1,zc2;

    double fx=cameraMatrix.at<double>(0,0);
    double fy=cameraMatrix.at<double>(1,1);
    double u0=cameraMatrix.at<double>(0,2);
    double v0=cameraMatrix.at<double>(1,2);

    //激光平面参数集合清除
    light_plane.clear();
    //zenturm_camera_array.clear();
    //相机坐标系中，激光中心线上的点
    zenturm_camera.clear();
    for(int img_cnt=0;img_cnt<img_laser.size();img_cnt++)
    {
        //激光线图像中，激光中心线提取，后续读取图像处理类中的参数来做最小二乘法拟合标定
        laser_zenturm_line_zwei(img_laser[img_cnt],res);
        // 相机外参转换成平面方程，标定板平面
        math_geometry::geo_plane_param p=extrinsic2plane(extrinsic_matrix[img_cnt+25]); //从第26张图片为激光平面标定用的标定板

        // 计算中线上点坐标在相机坐标系中的值
        for(size_t i=0;i<zenturm_line.size();i++)
        {
            double zc=-p.D/(p.A*(zenturm_line[i].x-u0)/fx+p.B*(zenturm_line[i].y-v0)/fy+p.C);
            double xc=zc*(zenturm_line[i].x-u0)/fx;
            double yc=zc*(zenturm_line[i].y-v0)/fy;
            zenturm_camera.push_back(Eigen::Vector3d(xc,yc,zc));
        }
    }

    line_point_array(zenturm_camera,zenturm_camera_array);

    // 最小二乘法计算平面方程
    light_plane.clear();
    for(size_t plane_cnt=0;plane_cnt<zenturm_camera_array.size();plane_cnt++)
    {
        light_plane_ein=math_leastsqaure::plane_calculate(zenturm_camera_array[plane_cnt]);
        light_plane.push_back(light_plane_ein);
#ifdef construct_cal_print_data_info
        std::cout<<"A:="<<light_plane_ein.A<<std::endl;
        std::cout<<"B:="<<light_plane_ein.B<<std::endl;
        std::cout<<"C:="<<light_plane_ein.C<<std::endl;
        std::cout<<"D:="<<light_plane_ein.D<<std::endl;
#endif
    }
}

/*
    多条激光平面参数标定，超过两条
    @img_chess:包含棋盘格的图片,30张，从中提取后5张
    @img_laser:包含激光线的图片,5张
*/
void construct_cal::lightsourceCalibrate(std::vector<cv::Mat> img_chess,std::vector<cv::Mat> img_laser)
{
    cv::Mat res; //useless

    //去除相机畸变
    imgLaserUndistort.clear();
    for(int imgCnt=0;imgCnt<img_laser.size();imgCnt++)
    {
        cv::Mat ui;
        cv::undistort(img_laser[imgCnt],ui,cameraMatrix,distCoeffs);
        imgLaserUndistort.push_back(ui);
#ifdef construct_cal_save_process
        write_path=write_img_path;
        write_path+="undistort/";
        write_path+=std::to_string(imgCnt);
        write_path+=".png";
        cv::imwrite(write_Path,ui);
#endif
    }
#ifdef construct_cal_print_msg_info
        printf("achieve the img undistort for light source calibrate...\n");
#endif

    //相机坐标系中，中心点的坐标值
    zenturm_camera_array.clear();

    std::vector<Eigen::Vector3d> zc1,zc2;

    double fx=cameraMatrix.at<double>(0,0);
    double fy=cameraMatrix.at<double>(1,1);
    double u0=cameraMatrix.at<double>(0,2);
    double v0=cameraMatrix.at<double>(1,2);

    //激光平面参数集合清除
    light_plane.clear();
    for(int img_cnt=0;img_cnt<imgLaserUndistort.size();img_cnt++)
    {
        //激光线图像中，激光中心线提取，后续读取图像处理类中的参数来做最小二乘法拟合标定
        laserZenturmLineMultiCal(imgLaserUndistort[img_cnt],res);
        //图像中提取到的中心线数量与实际不相同，结束标定
        if(zenturm_line_array.size()!=laserLineCnt)
        {
#ifdef construct_cal_print_error_info
            printf("cal img process not correct,calibrate failed...\n");
            return;
#endif
        }
#ifdef construct_cal_save_process
        write_path=write_img_path;
        write_path+="sys_cal_res_";
        write_path+=std::to_string(img_cnt+1);
        write_path+=".png";
        cv::imwrite(write_path,res);
#endif
        // 相机外参转换成平面方程，标定板平面
        math_geometry::geo_plane_param p=extrinsic2plane(extrinsic_matrix[img_cnt+25]); //从第26张图片为激光平面标定用的标定板

        // 计算中线上点坐标在相机坐标系中的值
        for(size_t i=0;i<zenturm_line_array.size();i++)
        {
            //相机坐标系中，激光中心线上的点，每次清除
            zenturm_camera.clear();
            for(size_t j=0;j<zenturm_line_array[i].size();j++)
            {
                double zc=-p.D/(p.A*(zenturm_line_array[i][j].x-u0)/fx+p.B*(zenturm_line_array[i][j].y-v0)/fy+p.C);
                double xc=zc*(zenturm_line_array[i][j].x-u0)/fx;
                double yc=zc*(zenturm_line_array[i][j].y-v0)/fy;
                zenturm_camera.push_back(Eigen::Vector3d(xc,yc,zc));
            }
            zenturm_camera_array.push_back(zenturm_camera);
        }
    }

    //line_point_array(zenturm_camera,zenturm_camera_array);

    // 最小二乘法计算平面方程
    light_plane.clear();
    /*for(size_t plane_cnt=0;plane_cnt<zenturm_camera_array.size();plane_cnt+=laserLineCnt)
    {
        light_plane_ein=math_leastsqaure::plane_calculate(zenturm_camera_array[plane_cnt]);
        light_plane.push_back(light_plane_ein);
#ifdef construct_cal_print_data_info
        std::cout<<"A:="<<light_plane_ein.A<<std::endl;
        std::cout<<"B:="<<light_plane_ein.B<<std::endl;
        std::cout<<"C:="<<light_plane_ein.C<<std::endl;
        std::cout<<"D:="<<light_plane_ein.D<<std::endl;
        std::cout<<std::endl;
#endif
    }*/
    //5张图，每张图laserLineCnt条线
    for(size_t plane_cnt=0;plane_cnt<laserLineCnt;plane_cnt+=1)
    {
        std::vector<Eigen::Vector3d> pointsT;
        for(size_t pointSet=plane_cnt;pointSet<zenturm_camera_array.size();pointSet+=laserLineCnt)
        {
            for(size_t j=0;j<zenturm_camera_array[pointSet].size();j++)
            {
                pointsT.push_back(zenturm_camera_array[pointSet][j]);
            }
        }

        light_plane_ein=math_leastsqaure::plane_calculate(pointsT);
        light_plane.push_back(light_plane_ein);

#ifdef construct_cal_print_data_info
        std::cout<<"A:="<<light_plane_ein.A<<std::endl;
        std::cout<<"B:="<<light_plane_ein.B<<std::endl;
        std::cout<<"C:="<<light_plane_ein.C<<std::endl;
        std::cout<<"D:="<<light_plane_ein.D<<std::endl;
        std::cout<<std::endl;
#endif
    }
}

/*
    直线上点排序，两条激光线时使用
    @points:左右激光线融合的点集合
    @array_points:左右分开后的结果
*/
void construct_cal::line_point_array(std::vector<Eigen::Vector3d> points,std::vector<std::vector<Eigen::Vector3d>> &array_points)
{
    std::vector<Eigen::Vector3d> array1,array2;
    int x_max=-10000;
    int x_min=10000;

    for(int point_cnt=0;point_cnt<points.size();point_cnt++)
    {
        if(points[point_cnt](0)>x_max)
        {
            x_max=points[point_cnt](0);
        }
        else if(points[point_cnt](0)<x_min)
        {
            x_min=points[point_cnt](0);
        }
    }

    //int x_m=abs(x_max-x_min)/2;
    int x_m=10;
    for(int point_cnt=0;point_cnt<points.size();point_cnt++)
    {
        if(abs(points[point_cnt](0)-x_max)<x_m)
        {
            array1.push_back(points[point_cnt]);
        }
        else if(abs(points[point_cnt](0)-x_min)<x_m)
        {
            array2.push_back(points[point_cnt]);
        }
    }

    array_points.push_back(array1);
    array_points.push_back(array2);
}

/*
    相机标定程序，用于三维重建和激光点测距,使用类中默认的参数
    @img_vector:图像容器,30张,前20张用于相机标定，后10张用于激光线方程计算，激光点测距时
    @img_vector:图像容器,20张,三维重建相机标定时
    @transform_matrix:返回值，外参矩阵
*/
std::vector<cv::Mat> construct_cal::camera_calibrate(std::vector<cv::Mat> img_vector)
{
#ifdef construct_cal_print_msg_info
    printf("start the calibrate from img_array...\n");
#endif

    int image_count=0;
    cv::Size image_size;

    std::vector<cv::Point2f> image_points;
    std::vector<std::vector<cv::Point2f>> image_points_seq;

    for(size_t i=0;i<img_vector.size();i++)
    {
        image_count++;
#ifdef construct_cal_print_msg_info
        std::cout << "image_count=" << image_count << std::endl;
#endif
        cv::Mat imageInput = img_vector[i];
        if(image_count==1)
        {
            image_size.width = imageInput.cols;
            image_size.height = imageInput.rows;
        }

        bool bRes=findChessboardCorners(imageInput,board_size,image_points,0);

        if(bRes)
        {
            cv::Mat view_gray;
            cvtColor(imageInput,view_gray,cv::COLOR_RGB2GRAY);
            cv::cornerSubPix(view_gray,image_points,cv::Size(11,11),cv::Size(-1, -1),cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT,30,0.01));
            image_points_seq.push_back(image_points);
            drawChessboardCorners(view_gray,board_size,image_points,true);
#ifdef construct_cal_save_process
            write_path=write_img_path;
            write_path+="view_gray_";
            write_path+=std::to_string(i+1);
            write_path+=".png";
            cv::imwrite(write_path,view_gray);
#endif
        }
        else
        {
#ifdef construct_cal_print_error_info
            printf("img fail...\n");
#endif
        }
    }

    std::vector<std::vector<cv::Point3f>> object_points_seq;

    for(int t=0;t<image_count;t++)
    {
        std::vector<cv::Point3f> object_points;
        for (int i = 0; i < board_size.height; i++)
        {
            for (int j = 0; j < board_size.width; j++)
            {
                cv::Point3f realPoint;
                realPoint.x = i * square_size.width;
                realPoint.y = j * square_size.height;
                realPoint.z = 0;
                object_points.push_back(realPoint);
            }
        }
        object_points_seq.push_back(object_points);
    }

    double rms;
    if(object_points_seq.size()==image_points_seq.size())
    {
        rms=calibrateCamera(object_points_seq,image_points_seq,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat, cv::CALIB_FIX_K3+cv::CALIB_ZERO_TANGENT_DIST);
    }
/*
    Eigen::Matrix3d ci;
    ci<<cameraMatrix.at<double>(0,0),
        cameraMatrix.at<double>(0,1),
        cameraMatrix.at<double>(0,2),
        cameraMatrix.at<double>(1,0),
        cameraMatrix.at<double>(1,1),
        cameraMatrix.at<double>(1,2),
        cameraMatrix.at<double>(2,0),
        cameraMatrix.at<double>(2,1),
        cameraMatrix.at<double>(2,2);
    cameraMatrix_inverse=ci.inverse();
*/
#ifdef construct_cal_print_data_info
    std::cout<<"cameraMatrix:="
            <<cameraMatrix
           <<std::endl;

    //std::cout<<"cameraMatrix_inverse:="
    //        <<cameraMatrix_inverse
    //       <<std::endl;
#endif

#ifdef construct_cal_print_data_info
    std::cout << "RMS:" << rms << "pixel" << std::endl << std::endl;
    for(size_t i=0;i<tvecsMat.size();i++)
    {
        std::cout<<"transform "
              <<i
              <<" := "
              <<tvecsMat[i]
              <<std::endl;
    }
#endif
    rotation_matrix.clear();
    for(size_t i=0;i<rvecsMat.size();i++)
    {
#ifdef construct_cal_print_data_info
        std::cout<<"rotate vector:="
              <<i
              <<" :="
              <<rvecsMat[i]
              <<std::endl;
#endif
        cv::Mat rm = cv::Mat(3,3,CV_64FC1,cv::Scalar::all(0));
        Rodrigues(rvecsMat[i],rm);
        rotation_matrix.push_back(rm);
#ifdef construct_cal_print_data_info
        std::cout<<"rotate matriz "
              <<i
              <<" :="
              <<rm
              <<std::endl;
#endif
    }

    extrinsic_matrix.clear();
    for(size_t i=0;i<rotation_matrix.size();i++)
    {
        cv::Mat tm=cv::Mat(4,4,CV_64FC1,cv::Scalar::all(0));
        for(int c=0;c<3;c++)
        {
            for(int r=0;r<3;r++)
            {
                tm.at<double>(c,r)=rotation_matrix[i].at<double>(c,r);
            }
        }
        tm.at<double>(0,3)=tvecsMat[i].at<double>(0,0);
        tm.at<double>(1,3)=tvecsMat[i].at<double>(0,1);
        tm.at<double>(2,3)=tvecsMat[i].at<double>(0,2);
        tm.at<double>(3,3)=1;
        extrinsic_matrix.push_back(tm);
#ifdef construct_cal_print_data_info
        std::cout<<"extrinsic matrix "
                <<i
                <<":="
                <<std::endl
                <<tm
                <<std::endl;
#endif
    }
#ifdef construct_cal_print_msg_info
    printf("calibrate done from img array...\n");
#endif
    return extrinsic_matrix;
}

/*
    相机标定程序，用于三维重建和激光点测距，使用自定义的参数
    @img_vector:图像容器,30张,前20张用于相机标定，后10张用于激光线方程计算，激光点测距时
    @img_vector:图像容器,30张,前20张用于相机标定，后10张用于激光平面方程计算，三维重建相机标定时
    @chess_size:棋盘格点数
    @chess_length:棋盘格格子长度，mm
    @transform_matrix:返回值，外参矩阵
*/
std::vector<cv::Mat> construct_cal::camera_calibrate(std::vector<cv::Mat> img_vector,cv::Size chess_size,cv::Size2f chess_length)
{
#ifdef construct_cal_print_msg_info
    printf("start the calibrate from img array...\n");
#endif

    int image_count=0;
    cv::Size image_size;
    int calibrate_cnt=0;

    std::vector<cv::Point2f> image_points;
    std::vector<std::vector<cv::Point2f>> image_points_seq;

    for(size_t i=0;i<img_vector.size();i++)
    {
        image_count++;

        cv::Mat imageInput = img_vector[i];
        if(image_count==1)
        {
            image_size.width = imageInput.cols;
            image_size.height = imageInput.rows;
        }

        bool bRes=findChessboardCorners(imageInput,chess_size,image_points,0);

        if(bRes)
        {
            cv::Mat view_gray;
            if(imageInput.channels()!=1)
            {
                cvtColor(imageInput,view_gray,cv::COLOR_RGB2GRAY);
            }
            else
            {
                view_gray=imageInput;
            }

            cv::cornerSubPix(view_gray,image_points,cv::Size(11,11),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.01));
            image_points_seq.push_back(image_points);
            calibrate_cnt++;
#ifdef construct_cal_save_process
            drawChessboardCorners(view_gray,chess_size,image_points,true);
            write_path=write_img_path;
            write_path+="view_gray_";
            write_path+=std::to_string(i+1);
            write_path+=".png";
            cv::imwrite(write_path,view_gray);
#endif
        }
        else
        {
#ifdef construct_cal_print_error_info
            std::cout << "image_count=" << image_count << std::endl;
            printf("chessboard find fail...\n");
#endif
        }
    }

    std::vector<std::vector<cv::Point3f>> object_points_seq;

    for(int t=0;t<image_count;t++)
    {
        std::vector<cv::Point3f> object_points;
        for (int i=0;i<chess_size.height;i++)
        {
            for (int j=0; j<chess_size.width;j++)
            {
                cv::Point3f realPoint;
                realPoint.x=i*chess_length.width;
                realPoint.y=j*chess_length.height;
                realPoint.z= 0;
                object_points.push_back(realPoint);
            }
        }
        object_points_seq.push_back(object_points);
    }

    double rms;
    if(object_points_seq.size()==image_points_seq.size())
    {
        rms=calibrateCamera(object_points_seq,image_points_seq,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,cv::CALIB_FIX_K3+cv::CALIB_ZERO_TANGENT_DIST);
    }

#ifdef construct_cal_print_data_info
    std::cout << "RMS:" << rms << "pixel" << std::endl << std::endl;
    for(size_t i=0;i<tvecsMat.size();i++)
    {
        std::cout<<"transform "
              <<i
              <<" := "
              <<tvecsMat[i]
              <<std::endl;
    }
#endif

    //图像去除畸变
    undistort_img_vector.clear();
    for(size_t img_cnt=0;img_cnt<img_vector.size();img_cnt++)
    {
        cv::Mat undistort_img;
        cv::undistort(img_vector[img_cnt],undistort_img,cameraMatrix,distCoeffs);
#ifdef construct_cal_save_process
        write_path=write_img_path_undistort;
        write_path+="undistort_";
        write_path+=std::to_string(img_cnt+1);
        write_path+=".png";
        cv::imwrite(write_path,undistort_img);
#endif
        undistort_img_vector.push_back(undistort_img);
    }

    rotation_matrix.clear();
    for(size_t i=0;i<rvecsMat.size();i++)
    {
#ifdef construct_cal_print_data_info
        std::cout<<"rotate vector "
              <<i
              <<" :="
              <<rvecsMat[i]
              <<std::endl;
#endif
        cv::Mat rm = cv::Mat(3,3,CV_64FC1,cv::Scalar::all(0));
        Rodrigues(rvecsMat[i],rm);
        rotation_matrix.push_back(rm);
#ifdef construct_cal_print_data_info
        std::cout<<"rotate matrix "
              <<i
              <<" :="
              <<rm
              <<std::endl;
#endif
    }

    extrinsic_matrix.clear();
    for(size_t i=0;i<rotation_matrix.size();i++)
    {
        cv::Mat tm=cv::Mat(4,4,CV_64FC1,cv::Scalar::all(0));
        for(int c=0;c<3;c++)
        {
            for(int r=0;r<3;r++)
            {
                tm.at<double>(c,r)=rotation_matrix[i].at<double>(c,r);
            }
        }
        tm.at<double>(0,3)=tvecsMat[i].at<double>(0,0);
        tm.at<double>(1,3)=tvecsMat[i].at<double>(0,1);
        tm.at<double>(2,3)=tvecsMat[i].at<double>(0,2);
        tm.at<double>(3,3)=1;
        extrinsic_matrix.push_back(tm);
#ifdef construct_cal_print_data_info
        std::cout<<"extrinsic matrix "
                <<i
                <<":="
                <<std::endl
                <<tm
                <<std::endl;
#endif
    }

#ifdef construct_cal_print_msg_info
    if(calibrate_cnt>15)
    {
        printf("calibrate done from img array...\n");
    }
    else
    {
        printf("calibrate failed from img array...\n");
    }
#endif
    return extrinsic_matrix;
}

/*
    对提取到的激光中心点按激光线分类
    @all_points:提取到的左右点的坐标集合
    @assigned_points:按激光线分类好的点集合
*/
void construct_cal::assign_points(std::vector<cv::Point2f> all_points,std::vector<std::vector<cv::Point2f>> &assigned_points)
{

}

namespace cuda
{

};

};
