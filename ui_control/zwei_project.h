/*
    文件等级：密一
    author:klug
    献给杜尔西内娅德尔托博索
    start:230706
    last:230706
*/

#ifndef ZWEI_PROJECT_H
#define ZWEI_PROJECT_H

#include <QWidget>

namespace Ui {
class zwei_project;
}

class zwei_project : public QWidget
{
    Q_OBJECT

public:
    explicit zwei_project(QWidget *parent = nullptr);
    ~zwei_project();

private:
    Ui::zwei_project *ui;
};

#endif // ZWEI_PROJECT_H
