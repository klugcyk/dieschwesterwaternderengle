/*
    文件等级：密一
    author:klug
    献给我的好友脚踏西瓜皮的胡安帕诺麦克
    start:230829
    last:230829
*/

#ifndef TOOLPAGE_H
#define TOOLPAGE_H

#include <QWidget>
#include <QLabel>

namespace Ui {
class toolPage;
}

class toolPage : public QWidget
{
    Q_OBJECT

public:
    explicit toolPage(QWidget *parent = nullptr);
    ~toolPage();

private slots:
    void on_openTree_clicked();

private:
    Ui::toolPage *ui;
    bool status=0;

};

#endif // TOOLPAGE_H
