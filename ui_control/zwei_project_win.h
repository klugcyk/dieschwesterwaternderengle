/*
    文件等级：密一
    author:klug
    献给杜尔西内娅德尔托博索
    start:230706
    last:230706
*/

#ifndef ZWEI_PROJECT_WIN_H
#define ZWEI_PROJECT_WIN_H

#include <QWidget>

#define zwei_pro_win_print_error_info
#define zwei_pro_win_print_msg_info
#define zwei_pro_win_print_data_info
#define zwei_pro_win_save_process

namespace Ui {
class zwei_project_win;
}

class zwei_project_win : public QWidget
{
    Q_OBJECT

public:
    explicit zwei_project_win(QWidget *parent = nullptr);
    ~zwei_project_win();

private slots:

private:
    Ui::zwei_project_win *ui;

};

#endif // ZWEI_PROJECT_WIN_H
