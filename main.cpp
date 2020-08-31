#include "mainwindow.h"
#include <QApplication>
// #include <QTime>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    // 设置窗口属性
    {
        w.setWindowTitle("JRR Controller");
    //  w.setFixedSize(608,263);
    }
    w.show();

    return a.exec();
}
