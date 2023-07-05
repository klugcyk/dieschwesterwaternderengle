/*
    文件等级：密一
    author:klug
    献给我的心上人等待天使的妹妹
    start:230215
    last:230605
*/

#include "StructureLight/construct_img.hpp"
#include "source.hpp"

namespace structlight_construct
{

construct_img::construct_img()
{
#ifdef construct_img_print_msg_info
    printf("start the 3d construct node...\n");
#endif

}

construct_img::construct_img(int r,int g,int b)
{
#ifdef construct_img_print_msg_info
    printf("start the 3d construct node set some value...\n");
#endif

    green_threshold=g;
    red_threshold=r;
    blue_threshold=b;
}

construct_img::~construct_img()
{
#ifdef construct_img_print_msg_info
    printf("end the 3d construct node...\n");
#endif

}

/*
    激光中心线提取算法，标定中2条中心线提取
    @src_img:输入图像
    @res_img:提取结果
    @res:返回值，提取到中心线计数
*/
int construct_img::laser_zenturm_line_zwei(cv::Mat src_img,cv::Mat &res_img)
{
    int line_cnt=0;

    res_img=src_img;
    //图像预处理
    cv::Mat bin_img(src_img.rows,src_img.cols,CV_8UC1);
    //增强红色通道
    for(size_t row=0;row<src_img.rows;row++)
    {
        for(size_t col=0;col<src_img.cols;col++)
        {
            int R=src_img.at<cv::Vec3b>(row,col)[2];
            if(R>200)
            {
                bin_img.at<uchar>(row,col)=255;
            }
            else
            {
                bin_img.at<uchar>(row,col)=0;
            }
        }
    }
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="line_bin.png";
    cv::imwrite(write_path,bin_img);
#endif

    cv::Mat mor_img;
    cv::Mat kernel=getStructuringElement(cv::MORPH_RECT,cv::Size(9,9),cv::Point(-1,-1));
    morphologyEx(bin_img,mor_img,CV_MOP_CLOSE,kernel);
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="line_morphology.png";
    cv::imwrite(write_path,mor_img);
#endif

    cv::Mat blur_img;
    cv::medianBlur(mor_img,blur_img,5);
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="line_blur.png";
    cv::imwrite(write_path,blur_img);
#endif
    //连通域roi提取
    cv::Mat labels,lab;
    cv::Mat connect_info,connect_centor;
    line_cnt=cv::connectedComponentsWithStats(blur_img,lab,connect_info,connect_centor,8,CV_16U);
#ifdef construct_img_print_data_info
    std::cout<<"connected cnt:="<<line_cnt<<std::endl;
#endif
    std::vector<cv::Mat> roi;
    roi.clear();
    for(int i=1;i<line_cnt;i++)
    {
        int x=connect_info.at<int>(i,0);
        int y=connect_info.at<int>(i,1);
        int h=connect_info.at<int>(i,2);
        int w=connect_info.at<int>(i,3);
        cv::Mat roi_img=blur_img(cv::Rect(x,y,h,w));
        roi.push_back(roi_img);

#ifdef construct_img_save_img
        write_path=write_img_path;
        write_path+="line_connect_roi_img_";
        write_path+=std::to_string(i);
        write_path+=".png";
        cv::imwrite(write_path,roi_img);
#endif
    }

    //单条激光中心提取
    std::vector<cv::Point2f> zenturm; //只保存单个roi中的点

    zenturm_line.clear();

    for(size_t i=0;i<roi.size();i++)
    {
        int row=roi[i].rows;
        int col=roi[i].cols;

        zenturm.clear();
        if(row>col)
        {
            cv::Mat roi_mark=laser_zenturm_line_ein(roi[i],zenturm,0);
        }
        else
        {
            cv::Mat roi_mark=laser_zenturm_line_ein(roi[i],zenturm,1);
        }

        int offset_x=connect_info.at<int>(i+1,0); //roi在原图上的位置，列偏置
        int offset_y=connect_info.at<int>(i+1,1); //roi在原图上的位置，行偏置
        save_point(zenturm,offset_x,offset_y);
    }
    //点按照激光线分类
    //zenturm_line_array.clear();
    //point_array(zenturm_line,zenturm_line_array);
#ifdef construct_img_mark
    //draw_point(res_img,zenturm_line_array[0],cv::Scalar(255,0,0));
    //draw_point(res_img,zenturm_line_array[1],cv::Scalar(0,255,0));
    draw_point(res_img,zenturm_line,cv::Scalar(255,0,0));
#endif

#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="line_res.png";
    cv::imwrite(write_path,res_img);
#endif

    return line_cnt;
}

/*
    对获取的所有激光点按激光线分类
    @points:所获取的所有点的集合
    @points_array:分类完点集合
*/
void construct_img::point_array(std::vector<cv::Point2f> points,std::vector<std::vector<cv::Point2f>> &points_array)
{
    std::vector<cv::Point2f> array1,array2;
    int x_max=0;
    int x_min=10000;

    for(int point_cnt=0;point_cnt<points.size();point_cnt++)
    {
        if(points[point_cnt].x>x_max)
        {
            x_max=points[point_cnt].x;
        }
        else if(points[point_cnt].x<x_min)
        {
            x_min=points[point_cnt].x;
        }
    }

    int x_m=(x_max+x_min)/2;
    for(int point_cnt=0;point_cnt<points.size();point_cnt++)
    {
        if(points[point_cnt].x>x_m)
        {
            array1.push_back(points[point_cnt]);
        }
        else
        {
            array2.push_back(points[point_cnt]);
        }
    }

    points_array.push_back(array1);
    points_array.push_back(array2);
}

/*
    提取网格中心线
    @src_img:原始图像
    @res_img:结果图像
    @p1:参数1
    @p2:参数2
*/
void construct_img::grid_extract_preprocess(cv::Mat src_img,cv::Mat &res_img,int p1,int p2)
{
    cv::Mat transform_img(src_img.rows,src_img.cols,CV_8UC1);

    for(size_t row=0;row<src_img.rows;row++)
    {
        for(size_t col=0;col<src_img.cols;col++)
        {
            /*int gray=src_img.at<cv::Vec3b>(row,col)[0]*1.5;
            gray-=src_img.at<cv::Vec3b>(row,col)[1];
            gray-=src_img.at<cv::Vec3b>(row,col)[2];

            if(gray>255)
            {
                gray=255;
            }
            else if(gray<50)
            {
                gray=0;
            }*/
            int gray;
            if(src_img.at<cv::Vec3b>(row,col)[0]==255&&src_img.at<cv::Vec3b>(row,col)[1]<100&&src_img.at<cv::Vec3b>(row,col)[2]<100)
            {
                gray=255;
            }
            else
            {
                gray=0;
            }

            transform_img.at<uchar>(row,col)=gray;
        }
    }

    for(size_t row=0;row<src_img.rows;row++)
    {
        for(size_t col=0;col<src_img.cols;col++)
        {
            int white_cnt=0;
            transform_img.at<uchar>(row,col);
        }
    }

#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="gray_transform.png";
    cv::imwrite(write_path,transform_img);
#endif
    cv::medianBlur(transform_img,transform_img,p1);
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="blur_img.png";
    cv::imwrite(write_path,transform_img);
#endif
    cv::Mat con_img(src_img.rows,src_img.cols,CV_8UC1);
    cv::Mat kernel=getStructuringElement(cv::MORPH_RECT,cv::Size(5,5),cv::Point(-1,-1));
    morphologyEx(transform_img,con_img,CV_MOP_CLOSE,kernel);
    //convolution_grid_extract(transform_img,con_img,3,p2);
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="con_img.png";
    cv::imwrite(write_img_path,con_img);
#endif
    cv::threshold(con_img,res_img,p2,255,cv::THRESH_BINARY);
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="res_img.png";
    cv::imwrite(write_path,res_img);
#endif
}

/*
    挖去图像中心过亮的区域
    @img1:曝光度低，突出结构光中心的图像
    @img2:曝光度正常，包含网格的图像
    @length:挖孔直径
    @res_img:返回值去除中心过亮区域的图像
*/
cv::Mat construct_img::zenturm_abandon(cv::Mat img1,cv::Mat img2,int length)
{
    cv::Mat res_img;

    if(img1.channels()!=1)
    {
        cv::cvtColor(img1,img1,cv::COLOR_BGR2GRAY);
    }

    if(img2.channels()!=1)
    {
        cv::cvtColor(img2,img2,cv::COLOR_BGR2GRAY);
    }

    cv::Mat bin_img;
    cv::threshold(img1,bin_img,100,255,cv::THRESH_BINARY);
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="bin_img.png";
    cv::imwrite(write_path,bin_img);
#endif

    //提取中心
    int row_cnt=0;
    int col_cnt=0;
    int pixel_cnt=0;
    for(size_t row=0;row<bin_img.rows;row++)
    {
        for(size_t col=0;col<bin_img.cols;col++)
        {
            int gray=bin_img.at<uchar>(row,col);
            if(gray>=200)
            {
                row_cnt+=row;
                col_cnt+=col;
                pixel_cnt++;
            }
        }
    }
    row_cnt/=pixel_cnt;
    col_cnt/=pixel_cnt;
#ifdef construct_img_mark
    cv::Mat mark=img1;
    cv::circle(mark,cv::Point(col_cnt,row_cnt),10,cv::Scalar(255,255,255),1,5,0);
    write_path=write_img_path;
    write_path+="res_mark1.png";
    imwrite(write_path,mark);
#endif
    res_img=img2;
    cv::circle(res_img,cv::Point(col_cnt,row_cnt),length,cv::Scalar(0,0,0),-1,5,0);
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="res_mark_abandon.png";
    imwrite(write_path,res_img);
#endif
    return res_img;
}

/*
    单条连通域激光线提取
    @src_img:输入图像
    @zenturm:激光中心点坐标
    @type:图像类型0，1，0为row>col,1为row<col
    @res_img:返回值，标记激光中心线的图像
*/
cv::Mat construct_img::laser_zenturm_line_ein(cv::Mat src_img,std::vector<cv::Point2f> &zenturm,bool type)
{
    cv::Mat res_img=src_img;
    float p;
    cv::Point2f zenturm_point;
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
    提取有交叉部分的中心线
*/
cv::Mat construct_img::laser_zenturm_line_zwei(cv::Mat src_img,std::vector<cv::Point> &zenturm,bool type)
{
    cv::Mat res_img=src_img;
    cv::cvtColor(src_img,src_img,cv::COLOR_BGR2GRAY);
    cv::Mat bin_img,element_img;
    threshold(src_img,bin_img,0,255,cv::THRESH_OTSU);
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="bin.png";
    cv::imwrite(write_path,bin_img);
#endif
    cv::Mat element;
    element = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::erode(bin_img,element_img,element,cv::Point(-1,-1));
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="element.png";
    cv::imwrite(write_path,element_img);
#endif

    cv::Mat harris;
    int blockSize=5;     // 邻域半径
    int apertureSize=11;  // 邻域大小
    cornerHarris(bin_img, harris, blockSize, apertureSize, 0.04);

    int max_x=0;
    int max_y=0;
    int min_x=src_img.cols;
    int min_y=src_img.rows;

    cv::Mat harrisn;
    normalize(harris, harrisn, 0, 255, cv::NORM_MINMAX);
    //将图像的数据类型变成CV_8U
    convertScaleAbs(harrisn, harrisn);
    //寻找Harris角点
    std::vector<cv::KeyPoint> keyPoints;
    for (int row = 0; row < harrisn.rows; row++)
    {
        for (int col = 0; col < harrisn.cols; col++)
        {
            int R = harrisn.at<uchar>(row, col);
            if (R > 125)
            {
                //向角点存入KeyPoint中
                cv::KeyPoint keyPoint;
                keyPoint.pt.y = row;
                keyPoint.pt.x = col;
                keyPoints.push_back(keyPoint);
            }
        }
    }

    for(int i=0;i<keyPoints.size();i++)
    {
        int _x=keyPoints[i].pt.x;
        int _y=keyPoints[i].pt.y;
        if(_x>max_x)
        {
            max_x=_x;
        }
        else if(_x<min_x)
        {
            min_x=_x;
        }

        if(_y>max_y)
        {
            max_y=_y;
        }
        else if(_y<min_y)
        {
            min_y=_y;
        }
    }

    cv::Point up(min_x,min_y);
    cv::Point down(max_x,max_y);

    for(int i=min_x;i<max_x;i++)
    {
        for(int j=min_y;j<max_y;j++)
        {
            src_img.at<uchar>(j,i)=0;
        }
    }
    res_img=src_img;

    return res_img;
}

/*
    在图像中打印中心线的点，并记录点的坐标信息到zenturm_line容器
    @zenturm:中心线点集
    @offset_x:偏置x
    @offset_y:偏置y
*/
void construct_img::save_point(std::vector<cv::Point2f> zenturm,int offset_x,int offset_y)
{
    for(size_t i=0;i<zenturm.size();i++)
    {
        float x=zenturm[i].x+offset_x;
        float y=zenturm[i].y+offset_y;

        zenturm_line.push_back(cv::Point2f(x,y)); //记录点信息
    }
}

/*
    在图像中打印中心线的点
    @src_img:原图
    @zenturm:中心线点集
    @offset_x:偏置x
    @offset_y:偏置y
*/
void construct_img::draw_point(cv::Mat &src_img,std::vector<cv::Point2f> zenturm,int offset_x,int offset_y)
{
    for(size_t i=0;i<zenturm.size();i++)
    {
        float x=zenturm[i].x+offset_x;
        float y=zenturm[i].y+offset_y;

        circle(src_img,cv::Point2f(x,y),5,cv::Scalar(0,255,0),1);
    }
}

/*
    在图像中打印中心线的点
    @src_img:原图
    @zenturm:中心线点集
    @color:颜色
*/
void construct_img::draw_point(cv::Mat &src_img,std::vector<cv::Point2f> zenturm,cv::Scalar color)
{
    for(size_t i=0;i<zenturm.size();i++)
    {
        circle(src_img,cv::Point(zenturm[i].x,zenturm[i].y),1,color,1);
    }
}

/*
    网格提取，卷积增强网格中心线区域
    @src_img:待处理的图像
    @kernel_size:卷积处理核大小，必须为奇数
*/
void construct_img::convolution_grid_extract(cv::Mat src_img,cv::Mat &res_img,int kernel_size,int threshold)
{
    for(size_t row=(kernel_size-1)/2;row<src_img.rows-(kernel_size-1)/2;row++)
    {
        for(size_t col=(kernel_size-1)/2;col<src_img.cols-(kernel_size-1)/2;col++)
        {
            int cnt=0;
            for(size_t _row=row-(kernel_size-1)/2;_row<=row+(kernel_size-1)/2;_row++)
            {
                for(size_t _col=col-(kernel_size-1)/2;_col<=col+(kernel_size-1)/2;_col++)
                {
                    if(src_img.at<uchar>(_row,_col)>threshold)
                    {
                        cnt++;
                    }
                }
            }

            if(cnt==kernel_size*kernel_size)
            {
                res_img.at<uchar>(row,col)=255;
            }
            else
            {
                res_img.at<uchar>(row,col)=0;
            }
        }
    }
}

/*
    网格中心线提取
    @src_img:经过预处理提取相对较好的网格二值图
    @res_img:结果图
*/
void construct_img::grid_extract(cv::Mat src_img,cv::Mat &res_img)
{
    //形态学
    cv::Mat kernel=getStructuringElement(cv::MORPH_RECT,cv::Size(7,7),cv::Point(-1,-1));
    morphologyEx(src_img,res_img,CV_MOP_CLOSE,kernel);
}

/*
    圆弧边缘检测
    @contour:轮廓边缘点
    @threshold:圆度阈值，误差几个像素
    @step:圆被分割的步长，步长越小，检测精度越高
    @返回值，1 为圆弧，0 为非圆弧
*/
int construct_img::circle_detect(std::vector<cv::Point> contour,int threshold,int step)
{
    cv::RotatedRect box;

    box=cv::minAreaRect(contour);

    float rate=box.boundingRect().width/box.boundingRect().height;

    if(abs(1-rate)>0.1)
    {
        return 0;
    }

    int _x=box.center.x;//圆弧中心点坐标
    int _y=box.center.y;
    int _r=(box.boundingRect().width+box.boundingRect().height)/4;//圆弧半径

    float r_error=0;
    for(size_t i=0;i<contour.size();i+=step)
    {
        r_error+=abs(sqrt((contour[i].x-_x)*(contour[i].x-_x)+(contour[i].y-_y)*(contour[i].y-_y))-_r);
        if(r_error>threshold)
        {
            return 0;
        }
    }

    return 1;
}

/*
    激光中心线提取算法，标定中使用,提取单条线
    @src_img:输入图像
    @res_img:提取结果
*/
cv::Mat construct_img::laser_zenturm_line(cv::Mat src_img,cv::Mat &res_img)
{
    res_img=src_img;
    //图像预处理
    cv::Mat bin_img(src_img.rows,src_img.cols,CV_8UC1);
    //增强红色通道
    for(size_t row=0;row<src_img.rows;row++)
    {
        for(size_t col=0;col<src_img.cols;col++)
        {
            int R=src_img.at<cv::Vec3b>(row,col)[2];
            if(R>250)
            {
                bin_img.at<uchar>(row,col)=255;
            }
            else
            {
                bin_img.at<uchar>(row,col)=0;
            }
        }
    }
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="line_bin.png";
    cv::imwrite(write_path,bin_img);
#endif

    cv::Mat mor_img;
    cv::Mat kernel=getStructuringElement(cv::MORPH_RECT,cv::Size(9,9),cv::Point(-1,-1));
    morphologyEx(bin_img,mor_img,CV_MOP_CLOSE,kernel);
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="line_morphology.png";
    cv::imwrite(write_path,mor_img);
#endif

    cv::Mat blur_img;
    cv::medianBlur(mor_img,blur_img,5);
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="line_blur.png";
    cv::imwrite(write_path,blur_img);
#endif
    //连通域roi提取
    cv::Mat labels,lab;
    cv::Mat connect_info,connect_centor;
    int cnt=cv::connectedComponentsWithStats(blur_img,lab,connect_info,connect_centor,8,CV_16U);
#ifdef construct_img_print_data_info
    std::cout<<"connected cnt:="<<cnt<<std::endl;
#endif
    std::vector<cv::Mat> roi;
    for(int i=1;i<cnt;i++)
    {
        int x=connect_info.at<int>(i,0);
        int y=connect_info.at<int>(i,1);
        int h=connect_info.at<int>(i,2);
        int w=connect_info.at<int>(i,3);
        cv::Mat roi_img=blur_img(cv::Rect(x,y,h,w));
        roi.push_back(roi_img);

#ifdef construct_img_save_img
        write_path=write_img_path;
        write_path+="line_connect_roi_img_";
        write_path+=std::to_string(i);
        write_path+=".png";
        cv::imwrite(write_path,roi_img);
#endif
    }

    //单条激光中心提取
    std::vector<cv::Point2f> zenturm; //只保存单个roi中的点

    zenturm_line.clear();

    for(size_t i=0;i<roi.size();i++)
    {
        int row=roi[i].rows;
        int col=roi[i].cols;

        zenturm.clear();
        if(row>col)
        {
            cv::Mat roi_mark=laser_zenturm_line_ein(roi[i],zenturm,0);
        }
        else
        {
            cv::Mat roi_mark=laser_zenturm_line_ein(roi[i],zenturm,1);
        }

        int offset_x=connect_info.at<int>(i+1,0); //roi在原图上的位置，列偏置
        int offset_y=connect_info.at<int>(i+1,1); //roi在原图上的位置，行偏置
        save_point(zenturm,offset_x,offset_y);
    }
    //删除不需要的点
    point_filter(zenturm_line);
#ifdef construct_img_mark
    draw_point(res_img,zenturm_line,cv::Scalar(0,255,0));
#endif

#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="line_res.png";
    cv::imwrite(write_path,res_img);
#endif

    return res_img;
}

/*
    超过两条激光中心线提取算法，标定中使用
    要求标定时，激光线打在平面上
    @src_img:包含激光中心线的图像
    @res_img:结果图像
    @line_cnt:返回值，提取到激光中线的条数
*/
int construct_img::laserZenturmLineMultiCal(cv::Mat src_img, cv::Mat &res_img)
{
    int line_cnt=0;

    //清除上一张图片上提取的点位缓存
    zenturm_line_array.clear();

    res_img=src_img;
    //图像预处理
    cv::Mat bin_img(src_img.rows,src_img.cols,CV_8UC1);
    //增强红色通道
    for(size_t row=0;row<src_img.rows;row++)
    {
        for(size_t col=0;col<src_img.cols;col++)
        {
            int B=src_img.at<cv::Vec3b>(row,col)[0];
            if(B>250)
            {
                bin_img.at<uchar>(row,col)=255;
            }
            else
            {
                bin_img.at<uchar>(row,col)=0;
            }
        }
    }
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="line_bin.png";
    cv::imwrite(write_path,bin_img);
#endif

    cv::Mat mor_img;
    cv::Mat kernel=getStructuringElement(cv::MORPH_RECT,cv::Size(15,15),cv::Point(-1,-1));
    morphologyEx(bin_img,mor_img,CV_MOP_CLOSE,kernel);
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="line_morphology.png";
    cv::imwrite(write_path,mor_img);
#endif

    cv::Mat blur_img;
    cv::medianBlur(mor_img,blur_img,5);
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="line_blur.png";
    cv::imwrite(write_path,blur_img);
#endif
    //连通域roi提取
    cv::Mat labels,lab;
    cv::Mat connect_info,connect_centor;
    int cnt=cv::connectedComponentsWithStats(blur_img,lab,connect_info,connect_centor,8,CV_16U);
#ifdef construct_img_print_data_info
    std::cout<<"connected cnt:="<<cnt<<std::endl;
#endif
    std::vector<cv::Mat> roi;
    for(int i=1;i<cnt;i++)
    {
        int x=connect_info.at<int>(i,0);
        int y=connect_info.at<int>(i,1);
        int h=connect_info.at<int>(i,2);
        int w=connect_info.at<int>(i,3);
        cv::Mat roi_img=blur_img(cv::Rect(x,y,h,w));
        //去除短激光线的影响，只对长激光线做中心线提取
        if(w>1000)
        {
            roi.push_back(roi_img);
        }

#ifdef construct_img_save_img
        write_path=write_img_path;
        write_path+="line_connect_roi_img_";
        write_path+=std::to_string(i);
        write_path+=".png";
        cv::imwrite(write_path,roi_img);
#endif
    }

    //单条激光中心提取
    std::vector<cv::Point2f> zenturm; //只保存单个roi中的点
    zenturm_line.clear();

    //设置返回值提取到线的条数
    //line_cnt=roi.size();
    for(size_t i=0;i<roi.size();i++)
    {
        int row=roi[i].rows;
        int col=roi[i].cols;

        zenturm.clear();
        if(row>col)
        {
            cv::Mat roi_mark=laser_zenturm_line_ein(roi[i],zenturm,0);
        }
        else
        {
            cv::Mat roi_mark=laser_zenturm_line_ein(roi[i],zenturm,1);
        }

        int offset_x=connect_info.at<int>(i+1,0); //roi在原图上的位置，列偏置
        int offset_y=connect_info.at<int>(i+1,1); //roi在原图上的位置，行偏置
        savePointArray(zenturm,offset_x,offset_y); //保存激光中心点到类中保存中心点的变量中
    }
    //删除不需要的点
    //point_filter(zenturm_line);
#ifdef construct_img_mark
    draw_point(res_img,zenturm_line,cv::Scalar(0,255,0));
#endif

#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="line_res.png";
    cv::imwrite(write_path,res_img);
#endif

    line_cnt=zenturm_line_array.size();

    return line_cnt;
}

/*
    激光中心点加入集合
    @zenturm:提取到的激光中心点
    @offset_x:偏置x
    @offset_y:偏置y
*/
void construct_img::savePointArray(std::vector<cv::Point2f> zenturm,int offset_x,int offset_y)
{
    std::vector<cv::Point2f> zenturm_temp;
    for(size_t i=0;i<zenturm.size();i++)
    {
        float x=zenturm[i].x+offset_x;
        float y=zenturm[i].y+offset_y;
        zenturm_temp.push_back(cv::Point2f(x,y)); //记录点信息
    }
    zenturm_line_array.push_back(zenturm_temp);
}

/*
    对提取到的中心点排序，按x轴大小排列
    @zenturm:初始点
    @resZenturm:结果点
*/
void construct_img::PointArray(std::vector<std::vector<cv::Point2f>> zenturm,std::vector<std::vector<cv::Point2f>> &resZenturm)
{

}

/*
    按坐标值去除获取点中不需要的点
    @points:点集合
*/
void construct_img::point_filter(std::vector<cv::Point2f> &points)
{
    float x_max=0;
    float x_min=10000;

    for(size_t point_cnt=0;point_cnt<points.size();point_cnt++)
    {
        if(points[point_cnt].x>x_max)
        {
            x_max=points[point_cnt].x;
        }
        else if(points[point_cnt].x<x_min)
        {
            x_min=points[point_cnt].x;
        }
    }

    float x_m=(x_max+x_min)/2;
    for(size_t point_cnt=0;point_cnt<points.size();point_cnt++)
    {
        if(points[point_cnt].x>x_m)
        {
            std::vector<cv::Point2f>::iterator it=points.begin()+point_cnt;
            points.erase(it);
            point_cnt=0;
        }
    }
}

/*
    三维重建测试，提取激光线的点位置
    @src_img:重建用的图片
*/
void construct_img::construct_img_test(cv::Mat src_img,cv::Mat &res_img,std::vector<cv::Point2f> &points)
{
    res_img=src_img;

    //图像预处理
    cv::Mat bin_img(src_img.rows,src_img.cols,CV_8UC1);
    //增强红色通道
    for(size_t row=0;row<src_img.rows;row++)
    {
        for(size_t col=0;col<src_img.cols;col++)
        {
            int R=src_img.at<cv::Vec3b>(row,col)[2];
            if(R>50)
            {
                bin_img.at<uchar>(row,col)=255;
            }
            else
            {
                bin_img.at<uchar>(row,col)=0;
            }
        }
    }
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="test_line_bin.png";
    cv::imwrite(write_path,bin_img);
#endif

    cv::Mat mor_img;
    cv::Mat kernel=getStructuringElement(cv::MORPH_RECT,cv::Size(9,9),cv::Point(-1,-1));
    morphologyEx(bin_img,mor_img,CV_MOP_CLOSE,kernel);
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="test_line_morphology.png";
    cv::imwrite(write_path,mor_img);
#endif

    cv::Mat blur_img;
    cv::medianBlur(mor_img,blur_img,5);
#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="test_line_blur.png";
    cv::imwrite(write_path,blur_img);
#endif
    //连通域roi提取
    cv::Mat labels,lab;
    cv::Mat connect_info,connect_centor;
    int cnt=cv::connectedComponentsWithStats(blur_img,lab,connect_info,connect_centor,8,CV_16U);
#ifdef construct_img_print_data_info
    std::cout<<"connected cnt:="<<cnt<<std::endl;
#endif
    std::vector<cv::Mat> roi;
    for(int i=1;i<cnt;i++)
    {
        int x=connect_info.at<int>(i,0);
        int y=connect_info.at<int>(i,1);
        int h=connect_info.at<int>(i,2);
        int w=connect_info.at<int>(i,3);
        cv::Mat roi_img=blur_img(cv::Rect(x,y,h,w));
        roi.push_back(roi_img);

#ifdef construct_img_save_img
        write_path=write_img_path;
        write_path+="test_line_connect_roi_img_";
        write_path+=std::to_string(i);
        write_path+=".png";
        cv::imwrite(write_path,roi_img);
#endif
    }

    //单条激光中心提取
    std::vector<cv::Point2f> zenturm; //只保存单个roi中的点

    points.clear();

    for(size_t i=0;i<roi.size();i++)
    {
        int row=roi[i].rows;
        int col=roi[i].cols;

        zenturm.clear();
        if(row>col)
        {
            cv::Mat roi_mark=laser_zenturm_line_ein(roi[i],zenturm,0);
        }
        else
        {
            cv::Mat roi_mark=laser_zenturm_line_ein(roi[i],zenturm,1);
        }

        int offset_x=connect_info.at<int>(i+1,0); //roi在原图上的位置，列偏置
        int offset_y=connect_info.at<int>(i+1,1); //roi在原图上的位置，行偏置

        for(size_t i=0;i<zenturm.size();i++)
        {
            float x=zenturm[i].x+offset_x;
            float y=zenturm[i].y+offset_y;

            points.push_back(cv::Point2f(x,y)); //记录点信息
        }
    }
    //删除不需要的点
    point_filter(points);
#ifdef construct_img_mark
    draw_point(res_img,points,cv::Scalar(0,255,0));
#endif

#ifdef construct_img_save_img
    write_path=write_img_path;
    write_path+="test_line_res.png";
    cv::imwrite(write_path,res_img);
#endif
}

/*
    从内存中读取8位图片
    @pixel_add:第一个像素的地址
    @row:图像行数
    @col:图像列数
    @channels:图像通道数
*/
void construct_img::getimgfrommeony(uchar *pixel_add,int row,int col,int channels)
{

}

/*
    从内存中读取64位图片
    @pixel_add:第一个像素的地址
    @row:图像行数
    @col:图像列数
    @channels:图像通道数
*/
void construct_img::getimgfrommeony(double *pixel_add,int row,int col,int channels)
{

}

};
