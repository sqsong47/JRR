#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QThread>
#include <QTimer>
#include "emgcollector.h"
#include "qmotor.h"
#include "Sri/srisensor.h"
#include "Robot/qrobot.h"
#include "RLS/encoder.h"
#include "RLS/rasterencoder.h"
#include "PythonHandler/pythonhandler.h"
#include <vector>
#include <fstream>
#include <queue>

// SQL操作
#include <QSqlDatabase>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

    QThread* _myoThread;
    QThread* _motorThread;
    QThread* _sriThread;

    QThread* _rlsThread1;
    QThread* _rlsThread2;
    QThread* _rlsThread3;
    QThread* _rlsThread4;
    QThread* _rlsThread5;
    QThread* _rlsThread6;

    QThread* _pyThread;

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // 对机器人的回零程序做了封装,这里使用的是PPM
    bool runToHome();

    // 对读取编码器和计算关节角进行了封装
    void updateJointAngle();

    // 更新力值
    void updateForceValue();

    // 单步导纳运动
    void startRunning();

    // 读取电机的角速度，更新关节的角速度
    void readJointVelocity();

    //使用VM进行机器人回零函数
    bool moveToHome();

private:
    Ui::MainWindow *ui;

    MyoDevice* _myoDevice;              // 肌电信号采集器
    MotorDevice* _motorDevice;          // 电机指令控制器
    SriDevice* _sriDevice;              // 力传感器采集器

    // 编码器2-6关节是磁编码器，1关节是光栅编码器
    RasterRLS* _rlsJoint1;
    MagnecticRLS* _rlsJoint2;
    MagnecticRLS* _rlsJoint3;
    MagnecticRLS* _rlsJoint4;
    MagnecticRLS* _rlsJoint5;
    MagnecticRLS* _rlsJoint6;

    QRobot* _robot;                     // 运算控制器
    QTimer* _timer_control;             // 主控制的定时器
    QTimer* _timer_update_info;         // 更新机器人和电机数据的定时器，其频率需要很高


    BOOL _isSriStartFlag;               // 传感器启动的标志标量
    BOOL _isInAdmittanceFlag;           // 导纳模式的标志变量
    BOOL _isMyoStartFlag;               // Myo启动的标志变量
    BOOL _isTraingFlag;                 // 启动训练的标志变量

    BOOL _isDangerFlag;                 // 关节接近限位的标志变量
    bool _isTestVelFlag = false;        // 测试末端速度

    // PythonHandler
    PythonHandler* _pyHandler;
    double _virtualDamp;

    // sql操作
    QSqlDatabase _db;


private slots:
    void showError(std::string Msg);    // 显示错误信息
    void mainControlLoop();             // 给_robot输入数据，并做数据处理
    void showSriConfigState();          // 显示力矩传感器的状态
    bool createConnetion(int time_stamp,
                         double velocity,
                         double acceleration,
                         double jerk,
                         double B);             // 数据库的链接函数

signals:
    void errorOccur(std::string);       // 错误信号

private slots:
    void on_pushButton_enable_clicked();
    void on_pushButton_clearFault_clicked();
    void on_pushButton_release_pressed();
    void on_pushButton_release_released();
    void on_pushButton_robotState_clicked();
    void on_pushButton_moveForward_pressed();
    void on_pushButton_moveForward_released();
    void on_pushButton_buildConnect_clicked();
    void on_pushButton_moveBackward_pressed();
    void on_pushButton_moveBackward_released();
    void on_pushButton_configSri_clicked();
    void on_pushButton_startSri_clicked();
    void on_pushButton_forceInit_clicked();
    void on_pushButton_startAdmittanceMode_clicked();
    void on_pushButton_quickStop_clicked();
    void on_pushButton_startMyo_clicked();
    void on_setAdmittancePara_clicked();
    void on_moveToHome_clicked();
    void on_ptn_pythonInit_clicked();
    void on_ptn_suspendTrain_clicked();
    void on_ptn_startTrain_clicked();
    void on_ptn_activeVM_clicked();
    void on_ptn_testVel_clicked();
    void on_ptn_dataBase_clicked();
};

#endif // MAINWINDOW_H
