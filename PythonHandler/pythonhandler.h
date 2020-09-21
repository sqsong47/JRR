#ifndef PYTHONHANDLER_H
#define PYTHONHANDLER_H

#include <QObject>
#include <Python.h>


// ------------------C++ & Python-------------------
class PythonHandler : public QObject
{
    Q_OBJECT
public:
    explicit PythonHandler(QObject *parent = nullptr);
    ~PythonHandler();

    // 初始化的谓词函数
    bool initPython();

    // 获取python脚本信息
    bool getPyScript();

    // 启动网络训练的run函数
    void startRun();

    // 接口函数，trainOneStep执行一次训练操作，有输入有输出
    double trainOneStep(const double velocity, const double accelaration, const double jerk, const double distance);

    // 每次episode结束时，调用该函数显示总奖励
    double sumReward();

    // 测试函数：调用的接口函数
    void testCallMethod();

private:
    PyObject* _pModule;
    PyObject* _pRet;
    PyObject* _pClassDDPG;
    PyObject* _pInstance;

signals:

public slots:
};



#endif // PYTHONHANDLER_H
