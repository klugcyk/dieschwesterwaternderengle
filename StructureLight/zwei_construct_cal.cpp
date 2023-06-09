/*
    文件等级：密一
    author:klug
    献给我亲爱的好友梅尔基亚德斯
    start:230423
    last:230515
*/

#include "StructureLight/zwei_construct_cal.hpp"

namespace calibrate_zwei_construct
{

zwei_construct_cal::zwei_construct_cal()
{
#ifdef zwei_construct_cal_print_msg_info
    printf("start zwei_construct_cal...\n");
#endif

}

zwei_construct_cal::~zwei_construct_cal()
{
#ifdef zwei_construct_cal_print_msg_info
    printf("end zwei_construct_cal...\n");
#endif

}

/*
    双目结构光相机位置标定
    @img_link:左相机采集的图片集合
    @img_richt:右相机采集的图片集合
*/
void zwei_construct_cal::relationship_calibrate(std::vector<cv::Mat> img_link,std::vector<cv::Mat> img_richt)
{
    // link相机标定
    camera_calibrate(img_link,cameraMatrixLink,extrinsic_matrix_link,distCoeffsLink);
    // richt相机标定
    camera_calibrate(img_richt,cameraMatrixRicht,extrinsic_matrix_richt,distCoeffsRicht);
    //双目标定，利用opencv的API
    //cv::stereoCalibrate(,);
}

/*
    相机标定程序，用于三维重建和激光点测距,使用类中默认的参数
    @img_vector:图像容器,30张,
    @cameraMatrix:相机内参矩阵
    @extrinsic_matrix:相机外参矩阵集合
    @返回值，标定结果
*/
int zwei_construct_cal::camera_calibrate(std::vector<cv::Mat> img_vector,cv::Mat &cameraMatrix,std::vector<cv::Mat> &extrinsic_matrix,cv::Mat &distCoeffs)
{
    int image_count=0;
    cv::Size image_size;

    std::vector<cv::Point2f> image_points;
    std::vector<std::vector<cv::Point2f>> image_points_seq;

    for(size_t i=0;i<img_vector.size();i++)
    {
        image_count++;
#ifdef zwei_construct_cal_print_msg_info
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
#ifdef zwei_construct_cal_save_process
            std::string name="/home/klug/img/construct/view_gray_";
            name+=std::to_string(i+1);
            name+=".png";
            cv::imwrite(name,view_gray);
#endif
        }
        else
        {
#ifdef zwei_construct_cal_print_error_info
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

#ifdef zwei_construct_cal_print_data_info
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
#ifdef zwei_construct_cal_print_data_info
        std::cout<<"rotate vector:="
              <<i
              <<" :="
              <<rvecsMat[i]
              <<std::endl;
#endif
        cv::Mat rm = cv::Mat(3,3,CV_64FC1,cv::Scalar::all(0));
        Rodrigues(rvecsMat[i],rm);
        rotation_matrix.push_back(rm);
#ifdef zwei_construct_cal_print_data_info
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
#ifdef zwei_construct_cal_print_data_info
        std::cout<<"extrinsic matrix "
                <<i
                <<":="
                <<std::endl
                <<tm
                <<std::endl;
#endif
    }

    return 1;
}

/*
    双目结构光系统标定
    1 完成两个相机标定
    2 完成两相机位置关系标定
    3 完成激光线位置标定
*/
int zwei_construct_cal::system_calibrate()
{

    return 1;
}

};
