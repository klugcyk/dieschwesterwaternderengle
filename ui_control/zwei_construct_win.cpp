/*
    文件等级：密一
    author:klug
    献给不喜欢我的弗雷德里希冯海因洛特
    start:230510
    last:230625
*/

#include "zwei_construct_win.h"
#include "ui_zwei_construct_win.h"

zwei_construct_win::zwei_construct_win(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::zwei_construct_win)
{
    ui->setupUi(this);
    setWindowTitle("zwei_construct");
}

zwei_construct_win::~zwei_construct_win()
{
    delete ui;
}

void zwei_construct_win::on_read_param_link_clicked()
{

}

void zwei_construct_win::on_read_param_richt_clicked()
{
    camera_set_parameter_zwei();
}

void zwei_construct_win::on_set_param_link_clicked()
{

}

void zwei_construct_win::on_set_param_richt_clicked()
{

}

void zwei_construct_win::on_grab_zwei_clicked()
{
    camera_grab_zwei();
}

void zwei_construct_win::on_sys_cal_clicked()
{

}

void zwei_construct_win::on_save_clicked()
{
    int img_num=ui->img_name->text().toInt();
    int save_for=0;
    if(ui->for_cal->isChecked())
    {
        save_for=1;
    }

    img_read_path=zwei_read_img_path;
    img_read_path+="cam_link.png";

    cv::Mat read_img=cv::imread(img_read_path);
    if(!read_img.empty())
    {
        img_write_path=zwei_write_img_path;
        if(save_for==1)
        {
            img_write_path+="cal/link";
        }
        else
        {
            img_write_path+="link";
        }
        img_write_path+=std::to_string(img_num);
        img_write_path+=".png";
        cv::imwrite(img_write_path,read_img);
    }
    else
    {
#ifdef zwei_win_print_error_info
        printf("img link no found...\n");
#endif
    }

    img_read_path=zwei_read_img_path;
    img_read_path+="cam_richt.png";

    read_img=cv::imread(img_read_path);
    if(!read_img.empty())
    {
        img_write_path=zwei_write_img_path;
        if(save_for==1)
        {
            img_write_path+="cal/richt";
        }
        else
        {
            img_write_path+="richt";
        }
        img_write_path+=std::to_string(img_num);
        img_write_path+=".png";
        cv::imwrite(img_write_path,read_img);
    }
    else
    {
#ifdef zwei_win_print_error_info
        printf("img richt no found...\n");
#endif
    }
}

