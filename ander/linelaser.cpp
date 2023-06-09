/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:230322
    last:230421
*/

#include "linelaser.hpp"

// xjg_image
std::vector<cv::Point2f> real_points;
std::vector<cv::Point2f> extract_points; //端点坐标，数组大小由标定物决定
xjg_image::distortion cal_dis;
std::vector<cv::Point2f> res_array_point; //排列得到的标定物上的点
int col,row;
bool cal_flag=0;

#ifdef test

int main()
{
    // calibrate
    cv::Mat img=cv::imread("/home/klug/img/linelaser/calibrate/104.bmp");
    col=img.cols;
    row=img.rows;

    cv::Mat res;
    if(!img.empty())
    {
        cv::Point2f p1,p2;
        res=xjg_image::laser_zenturm_line_cal(img);
#ifdef linelaser_save_process_img
        cv::imwrite("/home/klug/img/linelaser/res_104.png",res);
#endif
    }

    xjg_image::camera_calibrate(img);

    return 0;
}

#endif

namespace xjg_image
{

/*
    按坐标排列提取到的端点
    @points_array:输入点列
    @type:排列类型，0按x排列，1按y排列
*/
void array_extract_points(std::vector<cv::Point2f> &points_array,bool type)
{
    float coordinate[2][points_array.size()];

    for(size_t i=0;i<points_array.size();i++)
    {
        coordinate[0][i]=points_array[i].x;
        coordinate[1][i]=points_array[i].y;
    }

    int point_cnt=points_array.size();
    points_array.clear();

    if(type)
    {
        for(size_t i=0;i<point_cnt-1;i++)
        {
            for(size_t j=0;j<point_cnt-1;j++)
            {
                float ein_x=coordinate[0][j];
                float ein_y=coordinate[1][j];
                float zwei_x=coordinate[0][j+1];
                float zwei_y=coordinate[1][j+1];

                // 如果前一个点的行位置大于后一个点的行位置，交换位置
                if(ein_y>zwei_y)
                {
                    coordinate[0][j]=zwei_x;
                    coordinate[1][j]=zwei_y;
                    coordinate[0][j+1]=ein_x;
                    coordinate[1][j+1]=ein_y;
                }
            }
        }
    }
    else
    {
        for(size_t i=0;i<point_cnt-1;i++)
        {
            for(size_t j=0;j<point_cnt-1;j++)
            {
                float ein_x=coordinate[0][j];
                float ein_y=coordinate[1][j];
                float zwei_x=coordinate[0][j+1];
                float zwei_y=coordinate[1][j+1];

                // 如果前一个点的列位置大于后一个点的列位置，交换位置
                if(ein_x>zwei_x)
                {
                    coordinate[0][j]=zwei_x;
                    coordinate[1][j]=zwei_y;
                    coordinate[0][j+1]=ein_x;
                    coordinate[1][j+1]=ein_y;
                }
            }
        }
    }

    for(int i=0;i<point_cnt;i++)
    {
        cv::Point2f p;
        p.x=coordinate[0][i];
        p.y=coordinate[1][i];
        points_array.push_back(p);
    }
}

/*
    按坐标排列提取到的端点
    @points_array:输入点列
    @res_array:结果点列
    @type:排列类型，0按x排列，1按y排列
*/
void array_extract_points(std::vector<cv::Point2f> points_array,std::vector<cv::Point2f> &res_array,bool type)
{
    float coordinate[2][points_array.size()];

    res_array.clear();

    for(size_t i=0;i<points_array.size();i++)
    {
        coordinate[0][i]=points_array[i].x;
        coordinate[1][i]=points_array[i].y;
    }

    if(type)
    {
        for(size_t i=0;i<points_array.size()-1;i++)
        {
            for(size_t j=0;j<points_array.size()-1;j++)
            {
                float ein_x=coordinate[0][j];
                float ein_y=coordinate[1][j];
                float zwei_x=coordinate[0][j+1];
                float zwei_y=coordinate[1][j+1];

                // 如果前一个点的行位置大于后一个点的行位置，交换位置
                if(ein_y>zwei_y)
                {
                    coordinate[0][j]=zwei_x;
                    coordinate[1][j]=zwei_y;
                    coordinate[0][j+1]=ein_x;
                    coordinate[1][j+1]=ein_y;
                }
            }
        }
    }
    else
    {
        for(size_t i=0;i<points_array.size()-1;i++)
        {
            for(size_t j=0;j<points_array.size()-1;j++)
            {
                float ein_x=coordinate[0][j];
                float ein_y=coordinate[1][j];
                float zwei_x=coordinate[0][j+1];
                float zwei_y=coordinate[1][j+1];

                // 如果前一个点的列位置大于后一个点的列位置，交换位置
                if(ein_x>zwei_x)
                {
                    coordinate[0][j]=zwei_x;
                    coordinate[1][j]=zwei_y;
                    coordinate[0][j+1]=ein_x;
                    coordinate[1][j+1]=ein_y;
                }
            }
        }
    }

    for(int i=0;i<points_array.size();i++)
    {
        cv::Point2f p;
        p.x=coordinate[0][i];
        p.y=coordinate[1][i];
        res_array.push_back(p);
    }
}

/*
    计算标定误差
    @point:单应性转换到世界坐标系下标定物中的点
    @error:返回值，每个点的误差平方和
*/
float error_calculate(std::vector<cv::Point2f> points)
{
    float error=0;

    //提取到的点与实际标定物上点数量不匹配
    if(points.size()!=real_points.size())
    {
#ifdef linelaser_print_error_info
        printf("the extract_points size!=real_points size\n");
#endif
        return -1;
    }

    for(size_t point_cnt=0;point_cnt<points.size();point_cnt++)
    {
        if(point_cnt!=0&&point_cnt!=3&&point_cnt!=4&&point_cnt!=7)
        {
            error+=(points[point_cnt].x-real_points[point_cnt].x)*(points[point_cnt].x-real_points[point_cnt].x);
            error+=(points[point_cnt].y-real_points[point_cnt].y)*(points[point_cnt].y-real_points[point_cnt].y);
        }
    }

    return error;
}

/*
    根据畸变系数按点去除畸变
    @points_array:输入特征点位置
    @res_array:去除后特征点位置
    @dis:畸变系数
*/
void distortion_eliminate(std::vector<cv::Point2f> points_array,std::vector<cv::Point2f> &res_array,distortion dis)
{
    res_array.clear();

    for(size_t point_cnt=0;point_cnt<points_array.size();point_cnt++)
    {
        float x0=points_array[point_cnt].x;
        float y0=points_array[point_cnt].y;
        float r=sqrt((col/2-x0)*(col/2-x0)+(row/2-y0)*(row/2-y0));
        float r2=r*r;
        float r4=r2*r2;
        float r6=r2*r2*r2;
        float s=dis.k1*r2+dis.k2*r4+dis.k3*r6;
        float dis_x=s*(col/2-x0);
        float dis_y=s*(row/2-y0);

        cv::Point2f p(x0+dis_x,y0+dis_y);
        res_array.push_back(p);
    }
}

/*
    根据畸变系数去除畸变
    @src_img:输入图像
    @res_img:去除畸变的图像
    @dis:畸变系数
*/
void distortion_eliminate(cv::Mat src_img,cv::Mat &res_img,distortion dis)
{
    res_img=src_img;
    int row=src_img.rows;
    int col=src_img.cols;
    int x0=col/2;
    int y0=row/2;

    for(size_t row_=0;row_<row;row_++)
    {
        for(size_t col_=0;col_<col;col_++)
        {
            float r=sqrt((col_-x0)*(col_-x0)+(row_-y0)*(row_-y0));
            float r2=r*r;
            float r4=r2*r2;
            float r6=r2*r2*r2;
            float s=1+dis.k1*r2+dis.k2*r4+dis.k3*r6;
            float dis_x=s*(col_-x0);
            float dis_y=s*(row_-y0);

            if(row_-dis_y>0&&row_-dis_y<row&&col_-dis_x>0&&col_-dis_x<col)
            {
                res_img.at<uchar>(row_,col_)=src_img.at<uchar>(row_-dis_y,col_-dis_x);
            }
            else
            {
                res_img.at<uchar>(row_,col_)=0;
            }
        }
    }
}

/*
    标定初始化，输入标定物上实际点的坐标
    @array_points:提取到端点的坐标
*/
void calibrate_initial(std::vector<cv::Point2f> array_points)
{
    real_points.clear();

    cv::Point2f p;

    float middle=(array_points[0].x+array_points[1].x)/2;
    float real_points_array[2][array_points.size()];

    for(int i=0;i<array_points.size();i=i+1)
    {
        int judge_cnt=0;
        for(size_t j=0;j<array_points.size();j=j+1)
        {
            if(array_points[i].x>array_points[j].x)
            {
                judge_cnt++;
            }
        }

        p.x=8.9*judge_cnt;

        if(i==0)
        {
            p.y=90.0;
        }
        else
        {
            p.y=90.0-10.0*(i-1);
        }

        real_points.push_back(p);
    }

    for(int i=0;i<real_points.size();i++)
    {
        for(int j=0;j<real_points.size();j++)
        {
            if(abs(real_points[j].x-i*8.9)<0.01)
            {
                real_points_array[0][i]=real_points[j].x;
                real_points_array[1][i]=real_points[j].y;
            }
        }
    }

    real_points.clear();
    array_extract_points(extract_points,0);
    for(int i=0;i<array_points.size();i++)
    {
        p.x=real_points_array[0][i];
        p.y=real_points_array[1][i];
        real_points.push_back(p);
#ifdef linelaser_print_data_info
        printf("real_array x:=%f\n",real_points[i].x);
        printf("real_array y:=%f\n",real_points[i].y);
        printf("extract_array x:=%f\n",extract_points[i].x);
        printf("extract_array y:=%f\n",extract_points[i].y);
        printf("\n");
#endif
    }
}



/*
    迭代标定计算误差
    @real:标定物上真实点坐标
    @extract:标定物提取到的点坐标
    @first_dis:迭代初始值
    @step:迭代步长
*/
distortion calibrate_iterate(cv::Mat src_img,std::vector<cv::Point2f> real,std::vector<cv::Point2f> extract,distortion first_dis,float step)
{
    distortion res_dis,dis;
    cv::Mat temp_img;
    float error_buf=1000000;
    std::vector<cv::Point2f> extract_;

    if(!cal_flag)
    {
        for(int k1=-9;k1<10;k1++)
        {
            for(int k2=-9;k2<10;k2++)
            {
                for(int k3=-9;k3<10;k3++)
                {
                    dis.k1=first_dis.k1+k1*step;
                    dis.k2=first_dis.k2+k2*step;
                    dis.k3=first_dis.k3+k3*step;

                    distortion_eliminate(extract_points,extract_,dis);
                    cv::Mat homo=cv::findHomography(extract_,real_points);

                    std::vector<cv::Point2f> points_reinject=point_reinject(extract_,homo);

                    float error=error_calculate(points_reinject);
                    if(error<error_buf)
                    {
#ifdef linelaser_print_res_info
                        printf("error:=%f\n",error);
#endif
                        error_buf=error;
                        res_dis=dis;
                    }
                }
            }
        }
        cal_flag=1;
    }
    else
    {
        for(int k1=0;k1<10;k1++)
        {
            for(int k2=0;k2<10;k2++)
            {
                for(int k3=0;k3<10;k3++)
                {
                    if(first_dis.k1<0)
                    {
                        dis.k1=first_dis.k1-k1*step;
                    }
                    else
                    {
                        dis.k1=first_dis.k1+k1*step;
                    }

                    if(first_dis.k2<0)
                    {
                        dis.k2=first_dis.k2-k2*step;
                    }
                    else
                    {
                        dis.k2=first_dis.k2+k2*step;
                    }

                    if(first_dis.k3<0)
                    {
                        dis.k3=first_dis.k3-k3*step;
                    }
                    else
                    {
                        dis.k3=first_dis.k3+k3*step;
                    }

                    distortion_eliminate(src_img,temp_img,dis);
                    cv::Mat homo=cv::findHomography(real_points,extract_points);

                    std::vector<cv::Point2f> points_reinject=point_reinject(extract_points,homo);

                    float error=error_calculate(points_reinject);
                    if(error<error_buf)
                    {
#ifdef linelaser_print_res_info
                        printf("error:=%f\n",error);
#endif
                        error_buf=error;
                        res_dis=dis;
                    }
                }
            }
        }
    }

    return res_dis;
}

/*
    线激光相机镜头标定
    @src_img:标定物图像
*/
void camera_calibrate(cv::Mat src_img)
{
    cv::Mat res_img,temp_img;
    //初始化标定参数
    calibrate_initial(extract_points);

    //判断点提取是否正确
    if(real_points.size()!=extract_points.size())
    {
#ifdef linelaser_print_error_info
        printf("points extract not correct...\n");
#endif
        return ;
    }

    std::vector<cv::Point2f> homo_points_real;
    std::vector<cv::Point2f> homo_points_extract;

    homo_points_extract.clear();
    homo_points_real.clear();

    homo_points_extract.push_back(extract_points[0]); //left
    homo_points_extract.push_back(extract_points[3]); //top
    homo_points_extract.push_back(extract_points[4]); //top
    homo_points_extract.push_back(extract_points[7]); //right

    homo_points_real.push_back(real_points[0]); //left
    homo_points_real.push_back(real_points[3]); //top
    homo_points_real.push_back(real_points[4]); //top
    homo_points_real.push_back(real_points[7]); //right

    float step=0.0001;
    distortion first_res;
    first_res.k1=0;
    first_res.k2=0;
    first_res.k3=0;

    for(int i=1;i<1000;i*=10)
    {
        step/=i;
        cal_dis=calibrate_iterate(src_img,homo_points_real,homo_points_extract,first_res,step);
        first_res=cal_dis;
    }

#ifdef linelaser_print_res_info
    printf("k1:=%f\n",cal_dis.k1);
    printf("k2:=%f\n",cal_dis.k2);
    printf("k3:=%f\n",cal_dis.k3);
#endif
}

/*
    计算点的重投影
    @points:初始点
    @homo:单应性矩阵
    @res_points:重投影的点
*/
std::vector<cv::Point2f> point_reinject(std::vector<cv::Point2f> points,cv::Mat homo)
{
    std::vector<cv::Point2f> res_points;
    cv::Point2f rp;
    res_points.clear();

    for(size_t point_cnt=0;point_cnt<points.size();point_cnt++)
    {
        float temp=points[point_cnt].x*homo.at<double>(2,0)+points[point_cnt].y*homo.at<double>(2,1)+homo.at<double>(2,2);
        rp.x=points[point_cnt].x*homo.at<double>(0,0)+points[point_cnt].y*homo.at<double>(0,1)+homo.at<double>(0,2);
        rp.x/=temp;
        rp.y=points[point_cnt].x*homo.at<double>(1,0)+points[point_cnt].y*homo.at<double>(1,1)+homo.at<double>(1,2);
        rp.y/=temp;
        res_points.push_back(rp);
    }

    return res_points;
}

/*
    在图像中打印中心线的点
    @src_img:原图
    @zenturm:中心线点集
    @offset_x:偏置x
    @offset_y:偏置y
*/
void draw_point(cv::Mat &src_img,std::vector<cv::Point2f> zenturm,int offset_x,int offset_y)
{
    for(size_t i=0;i<zenturm.size();i++)
    {
        int x=zenturm[i].x+offset_x;
        int y=zenturm[i].y+offset_y;

        circle(src_img,cv::Point(x,y),1,cv::Scalar(0,255,0),1);
    }
}

/*
    单条激光线提取
    @src_img:输入图像
    @zenturm:激光中心点图像
    @type:图像类型0，1，0为row>col,1为row<col
*/
cv::Mat laser_zenturm_line_ein(cv::Mat src_img,std::vector<cv::Point2f> &zenturm,bool type)
{
    cv::Mat res_img=src_img;
    float p;
    cv::Point zenturm_point;
    if(type)
    {
        for(size_t col=0;col<res_img.cols;col++)
        {
            int cnt=0;
            int pos_sum=0;
            for(size_t row=0;row<res_img.rows;row++)
            {
                int gray=src_img.at<uchar>(row,col);
                if(gray>=1)
                {
                    pos_sum+=row;
                    cnt++;
                }
            }
            p=pos_sum/cnt;
            res_img.at<uchar>(p,col)=0;
            zenturm_point.x=col;
            zenturm_point.y=p;
            zenturm.push_back(zenturm_point);
        }
    }
    else
    {
        for(size_t row=0;row<res_img.rows;row++)
        {
            int cnt=0;
            int pos_sum=0;
            for(size_t col=0;col<res_img.cols;col++)
            {
                int gray=src_img.at<uchar>(row,col);
                if(gray>=1)
                {
                    pos_sum+=col;
                    cnt++;
                }
            }
            p=pos_sum/cnt;
            res_img.at<uchar>(row,p)=0;
            zenturm_point.x=p;
            zenturm_point.y=row;
            zenturm.push_back(zenturm_point);
        }
    }

    return res_img;
}

/*
    激光中心线提取算法，标定使用
    @src_img:输入图像
    @end_point:
    @res_img:提取结果
*/
cv::Mat laser_zenturm_line_cal(cv::Mat src_img)
{
    cv::Mat res_img=src_img;
    cv::Mat temp2;

    if(!src_img.empty())
    {
        cv::cvtColor(src_img,temp2,cv::COLOR_BGR2GRAY);
        //二值化
        threshold(temp2,temp2,50,255,cv::THRESH_BINARY);
        //形态学
        cv::Mat kernel=getStructuringElement(cv::MORPH_RECT,cv::Size(3,3),cv::Point(-1,-1));
        morphologyEx(temp2,temp2,CV_MOP_OPEN,kernel);
#ifdef linelaser_save_process_img
        cv::imwrite("/home/klug/sharonchriswinyard/linelaser/res_bin_img.png",temp2);
#endif

        cv::Mat labels,lab;
        cv::Mat connect_info,connect_centor;
        int cnt=cv::connectedComponentsWithStats(temp2,lab,connect_info,connect_centor,8,CV_16U);
#ifdef linelaser_print_data_info
        std::cout<<"connected cnt:="<<cnt<<std::endl;
#endif
        std::vector<cv::Mat> roi;
        for(int i=1;i<cnt;i++)
        {
            int x=connect_info.at<int>(i,0);
            int y=connect_info.at<int>(i,1);
            int h=connect_info.at<int>(i,2);
            int w=connect_info.at<int>(i,3);
            cv::Mat roi_img=temp2(cv::Rect(x,y,h,w));
            roi.push_back(roi_img);

#ifdef linelaser_save_process_img
            std::string name="/home/klug/sharonchriswinyard/linelaser/res_roi_img_";
            name+=std::to_string(i);
            name+=".png";
            cv::imwrite(name,roi_img);
#endif
        }

        std::vector<cv::Point2f> zenturm;
        std::vector<cv::Point2f> end_points;
        for(size_t i=0;i<roi.size();i++)
        {
            int row=roi[i].rows;
            int col=roi[i].cols;

            zenturm.clear();

            if(row>col)
            {
                cv::Mat roi_mark=laser_zenturm_line_ein(roi[i],zenturm,0);

                int offset_x=connect_info.at<int>(i+1,0); //roi在原图上的位置，列偏置
                int offset_y=connect_info.at<int>(i+1,1); //roi在原图上的位置，行偏置
                draw_point(res_img,zenturm,offset_x,offset_y);

                //查找端点
                laser_endpoint(zenturm,end_points,offset_x,offset_y,0);
            }
            else
            {
                cv::Mat roi_mark=laser_zenturm_line_ein(roi[i],zenturm,1);

                int offset_x=connect_info.at<int>(i+1,0);
                int offset_y=connect_info.at<int>(i+1,1);
                draw_point(res_img,zenturm,offset_x,offset_y);

                //查找端点
                laser_endpoint(zenturm,end_points,offset_x,offset_y,1);
            }
        }

        res_array_point.clear();
        extract_points.clear();
        res_array_point=array_point(end_points,1);

        cv::Point2f dp;

        dp.x=res_array_point[0].x;
        dp.y=res_array_point[0].y;
        extract_points.push_back(dp);
#ifdef linelaser_markcircle_on_img
        circle(res_img,dp,10,cv::Scalar(0,0,255),1,cv::LINE_8,0);
#endif
        dp.x=res_array_point[1].x;
        dp.y=res_array_point[1].y;
        extract_points.push_back(dp);
#ifdef linelaser_markcircle_on_img
        circle(res_img,dp,10,cv::Scalar(0,0,255),1,cv::LINE_8,0);
#endif
        for(int p_cnt=2;p_cnt<14;p_cnt=p_cnt+2) // -4表示去除最下面平面上的两个非标定物的点
        {
            if(res_array_point[p_cnt].x<(res_array_point[0].x+res_array_point[1].x)/2) //左侧的点
            {
                if(res_array_point[p_cnt].x<res_array_point[p_cnt+1].x)
                {
                    dp.x=res_array_point[p_cnt].x;
                    dp.y=res_array_point[p_cnt].y;
                }
                else
                {
                    dp.x=res_array_point[p_cnt+1].x;
                    dp.y=res_array_point[p_cnt+1].y;
                }
                extract_points.push_back(dp);
#ifdef linelaser_markcircle_on_img
                circle(res_img,dp,10,cv::Scalar(0,0,255),1,cv::LINE_8,0);
#endif
            }
            else //右侧的点
            {
                if(res_array_point[p_cnt].x>res_array_point[p_cnt+1].x)
                {
                    dp.x=res_array_point[p_cnt].x;
                    dp.y=res_array_point[p_cnt].y;
                }
                else
                {
                    dp.x=res_array_point[p_cnt+1].x;
                    dp.y=res_array_point[p_cnt+1].y;
                }
                extract_points.push_back(dp);
#ifdef linelaser_markcircle_on_img
                circle(res_img,dp,10,cv::Scalar(0,0,255),1,cv::LINE_8,0);
#endif
            }
        }
    }
    else
    {
#ifdef linelaser_print_error_info
        printf("img is empty!!!\n");
#endif
    }

    return res_img;
}

/*
    激光中心线提取算法
    @src_img:输入图像
    @end_point:
    @res_img:提取结果
*/
cv::Mat laser_zenturm_line(cv::Mat src_img)
{
    cv::Mat res_img=src_img;
    cv::Mat temp2;

    if(!src_img.empty())
    {
        cv::cvtColor(src_img,temp2,cv::COLOR_BGR2GRAY);
        //二值化
        threshold(temp2,temp2,100,255,cv::THRESH_BINARY);
        //形态学
        cv::Mat kernel=getStructuringElement(cv::MORPH_RECT,cv::Size(3,3),cv::Point(-1,-1));
        morphologyEx(temp2,temp2,CV_MOP_OPEN,kernel);
#ifdef linelaser_save_process_img
        cv::imwrite("/home/klug/sharonchriswinyard/linelaser/res_bin_img.png",temp2);
#endif

        cv::Mat labels,lab;
        cv::Mat connect_info,connect_centor;
        int cnt=cv::connectedComponentsWithStats(temp2,lab,connect_info,connect_centor,8,CV_16U);
#ifdef linelaser_print_data_info
        std::cout<<"connected cnt:="<<cnt<<std::endl;
#endif
        std::vector<cv::Mat> roi;
        for(int i=1;i<cnt;i++)
        {
            int x=connect_info.at<int>(i,0);
            int y=connect_info.at<int>(i,1);
            int h=connect_info.at<int>(i,2);
            int w=connect_info.at<int>(i,3);
            cv::Mat roi_img=temp2(cv::Rect(x,y,h,w));
            roi.push_back(roi_img);

#ifdef linelaser_save_process_img
            std::string name="/home/klug/sharonchriswinyard/linelaser/res_roi_img_";
            name+=std::to_string(i);
            name+=".png";
            cv::imwrite(name,roi_img);
#endif
        }

        std::vector<cv::Point2f> zenturm;
        std::vector<cv::Point2f> end_points;
        for(size_t i=0;i<roi.size();i++)
        {
            int row=roi[i].rows;
            int col=roi[i].cols;

            zenturm.clear();
            extract_points.clear();

            if(row>col)
            {
                cv::Mat roi_mark=laser_zenturm_line_ein(roi[i],zenturm,0);

                int offset_x=connect_info.at<int>(i+1,0); //roi在原图上的位置，列偏置
                int offset_y=connect_info.at<int>(i+1,1); //roi在原图上的位置，行偏置
                draw_point(res_img,zenturm,offset_x,offset_y);
            }
            else
            {
                cv::Mat roi_mark=laser_zenturm_line_ein(roi[i],zenturm,1);

                int offset_x=connect_info.at<int>(i+1,0);
                int offset_y=connect_info.at<int>(i+1,1);
                draw_point(res_img,zenturm,offset_x,offset_y);
            }
        }
    }
    else
    {
#ifdef linelaser_print_error_info
        printf("img is empty!!!\n");
#endif
    }

    return res_img;
}

/*
    排列点
    @point_array:点列
    @type:0:按列排列,1:按行排列
*/
std::vector<cv::Point2f> array_point(std::vector<cv::Point2f> point_array,bool type)
{
    std::vector<cv::Point2f> res_array;
    float coordinate[2][point_array.size()];

    for(size_t i=0;i<point_array.size();i++)
    {
        coordinate[0][i]=point_array[i].x;
        coordinate[1][i]=point_array[i].y;
    }

    if(type)
    {
        for(size_t i=0;i<point_array.size()-1;i++)
        {
            for(size_t j=0;j<point_array.size()-1;j++)
            {
                float ein_x=coordinate[0][j];
                float ein_y=coordinate[1][j];
                float zwei_x=coordinate[0][j+1];
                float zwei_y=coordinate[1][j+1];

                // 如果前一个点的行位置大于后一个点的行位置，交换位置
                if(ein_y>zwei_y)
                {
                    coordinate[0][j]=zwei_x;
                    coordinate[1][j]=zwei_y;
                    coordinate[0][j+1]=ein_x;
                    coordinate[1][j+1]=ein_y;
                }
            }
        }
    }
    else
    {
        for(size_t i=0;i<point_array.size();i++)
        {
            for(size_t j=0;j<point_array.size();j++)
            {
                float ein_x=coordinate[0][j];
                float ein_y=coordinate[1][j];
                float zwei_x=coordinate[0][j+1];
                float zwei_y=coordinate[1][j+1];

                // 如果前一个点的列位置大于后一个点的列位置，交换位置
                if(ein_x>zwei_x)
                {
                    coordinate[0][j]=zwei_x;
                    coordinate[1][j]=zwei_y;
                    coordinate[0][j+1]=ein_x;
                    coordinate[1][j+1]=ein_y;
                }
            }
        }
    }

    for(int i=0;i<point_array.size();i++)
    {
        cv::Point2f p;
        p.x=coordinate[0][i];
        p.y=coordinate[1][i];
#ifdef linelaser_print_data_info
        printf("%d,%f\n",i,coordinate[1][i]);
#endif
        res_array.push_back(p);
    }

    return res_array;
}

/*
    激光中心线边缘点提取，标定中使用
    @zenturm:中心线上的点集合
    @p:上点或下点
    @row:图像行数
    @offset_y:roi区域在y方向上的偏置
*/
void laser_endpoint(std::vector<cv::Point2f> zenturm,std::vector<cv::Point2f> &ep,int row,int offset_y)
{
    float x_max=0;
    float y_max=0;
    float x_min=5000;
    float y_min=5000;

    for(int i=0;i<zenturm.size();i++)
    {
        if(y_max<zenturm[i].y)
        {
            x_max=zenturm[i].x;
            y_max=zenturm[i].y;
        }

        if(y_min>zenturm[i].y)
        {
            x_min=zenturm[i].x;
            y_min=zenturm[i].y;
        }
    }

    cv::Point2f p;
    if(y_min+offset_y<row/2&&y_max+offset_y>row/2)
    {
        p.x=x_min;
        p.y=y_min;
        ep.push_back(p);
        p.x=x_max;
        p.y=y_max;
        ep.push_back(p);
        return ;
    }

    if(y_min+offset_y<row/2&&y_max+offset_y<row/2)
    {
        p.x=x_min;
        p.y=y_min;
        ep.push_back(p);
        return ;
    }

    if(y_min+offset_y>row/2&&y_max+offset_y>row/2)
    {
        p.x=x_max;
        p.y=y_max;
        ep.push_back(p);
        return ;
    }
}

/*
    激光中心线边缘点提取，标定中使用
    @zenturm:中心线上的点集合
    @p:上点或下点
    @row:图像行数
    @offset_y:roi区域在y方向上的偏置
    @type:激光线类型，0:row>col,1:row<col
*/
void laser_endpoint(std::vector<cv::Point2f> zenturm,std::vector<cv::Point2f> &ep,int offset_x,int offset_y,bool type)
{
    float x_max=0;
    float y_max=0;
    float x_min=5000;
    float y_min=5000;

    if(type) //col>row
    {
        for(int i=0;i<zenturm.size();i++)
        {
            if(x_max<zenturm[i].x)
            {
                x_max=zenturm[i].x;
                y_max=zenturm[i].y;
            }

            if(x_min>zenturm[i].x)
            {
                x_min=zenturm[i].x;
                y_min=zenturm[i].y;
            }
        }
    }
    else //row>col
    {
        for(int i=0;i<zenturm.size();i++)
        {
            if(y_max<zenturm[i].y)
            {
                x_max=zenturm[i].x;
                y_max=zenturm[i].y;
            }

            if(y_min>zenturm[i].y)
            {
                x_min=zenturm[i].x;
                y_min=zenturm[i].y;
            }
        }
    }

    cv::Point2f p;

    p.x=x_min+offset_x;
    p.y=y_min+offset_y;
    ep.push_back(p);
    p.x=x_max+offset_x;
    p.y=y_max+offset_y;
    ep.push_back(p);
}

/*
    激光中心线边缘点提取，标定中使用
    @zenturm:中心线上的点集合
    @p:上点或下点
    @row:图像行数
    @offset_y:roi区域在y方向上的偏置
    @type:激光线类型，0:row>col,1:row<col
*/
void laser_endpoint(std::vector<cv::Point2f> zenturm,std::vector<cv::Point2f> &ep,bool type)
{
    float x_max=0;
    float y_max=0;
    float x_min=5000;
    float y_min=5000;

    if(type) //col>row
    {
        for(int i=0;i<zenturm.size();i++)
        {
            if(x_max<zenturm[i].x)
            {
                x_max=zenturm[i].x;
                y_max=zenturm[i].y;
            }

            if(x_min>zenturm[i].x)
            {
                x_min=zenturm[i].x;
                y_min=zenturm[i].y;
            }
        }
    }
    else //row>col
    {
        for(int i=0;i<zenturm.size();i++)
        {
            if(y_max<zenturm[i].y)
            {
                x_max=zenturm[i].x;
                y_max=zenturm[i].y;
            }

            if(y_min>zenturm[i].y)
            {
                x_min=zenturm[i].x;
                y_min=zenturm[i].y;
            }
        }
    }

    cv::Point2f p;

    p.x=x_min;
    p.y=y_min;
    ep.push_back(p);
    p.x=x_max;
    p.y=y_max;
    ep.push_back(p);
}

/*
    激光中心线提取算法
    @src_img:输入图像
    @res_img:标记中心线的图像
    @zenturm:激光中心线点坐标
*/
void laser_extract(cv::Mat src_img,cv::Mat &res_img,std::vector<cv::Point2f> &zenturm)
{
    float p;

    if(src_img.channels()!=1)
    {
        cv::cvtColor(src_img,res_img,cv::COLOR_BGR2GRAY);
    }
    else
    {
        res_img=src_img;
    }

    for(size_t row=0;row<src_img.rows;row++)
    {
        int cnt=0;
        int pos_sum=0;
        for(size_t col=0;col<src_img.cols;col++)
        {
            int gray=res_img.at<uchar>(row,col);
            if(gray>=100)
            {
                pos_sum+=col;
                cnt++;
            }
        }

        if(cnt==0)
        {
            continue;
        }

        p=pos_sum/cnt;
        res_img.at<uchar>(row,p)=0;
        cv::Point2f point(row,p);
        zenturm.push_back(point);
    }
}

/*
    查找homography矩阵，调试代码使用
    @real:真实点坐标
    @extract:提取点坐标
*/
void find_homo(std::vector<cv::Point2f> real,std::vector<cv::Point2f> extract)
{
    if(real.size()!=extract.size())
    {
        return;
    }

    std::vector<cv::Point2f> homo_points_real;
    std::vector<cv::Point2f> homo_points_extract;

    homo_points_extract.clear();
    homo_points_real.clear();

    homo_points_extract.push_back(extract[0]); //left
    homo_points_extract.push_back(extract[3]); //top
    homo_points_extract.push_back(extract[4]); //top
    homo_points_extract.push_back(extract[7]); //right

    homo_points_real.push_back(real[0]); //left
    homo_points_real.push_back(real[3]); //top
    homo_points_real.push_back(real[4]); //top
    homo_points_real.push_back(real[7]); //right

    cv::Mat homo=cv::findHomography(homo_points_extract,homo_points_real);

    for(int point_cnt=0;point_cnt<extract.size();point_cnt++)
    {
        float temp=extract[point_cnt].x*homo.at<double>(2,0)+extract[point_cnt].y*homo.at<double>(2,1)+homo.at<double>(2,2);
        float x=extract[point_cnt].x*homo.at<double>(0,0)+extract[point_cnt].y*homo.at<double>(0,1)+homo.at<double>(0,2);
        x/=temp;
        float y=extract[point_cnt].x*homo.at<double>(1,0)+extract[point_cnt].y*homo.at<double>(1,1)+homo.at<double>(1,2);
        y/=temp;
#ifdef linelaser_print_data_info
        printf("homo transform %d x:%f\n",point_cnt,x);
        printf("homo transform %d y:%f\n",point_cnt,y);
        printf("\n");
#endif
    }
}

};
