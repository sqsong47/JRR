﻿#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QStringList>
#include <QDebug>
#include <iostream>
#include <QThread>
#include <time.h>
#include <string>
#include <iomanip>

#include <QSqlError>
#include <QSqlQuery>



/* 条件编译指令，可进行扩展
 * OPERATE_MODE = 1 代表处于普通的定导纳运动
 * OPERATE_MODE = 2 代表处于强化学习训练的变导纳过程
 * 使用时可以通过修改该宏的值进行操作模式选择
*/
#define OPERATE_MODE 4
#define TRAIN_MODE 2

// 控制输出debug信息的宏定义
#define DEBUG


/*******************************************************************
 *    2020.06.10  宋世奇
 * 1. 每次发生错误的时候都弹出模态对话框进行显示，确保操作的安全性
 *
 * 2. 电机Enable之后的状态设置为PPM，确保安全性，因为制动器会在enable之后打开
 *
 * 3. 打开制动器是通过使用CM，设置电流值为0并enabl完成的，在此状态下电机扭矩为0
 *
 * 4. 执行清除错误操作是会有点费时，此时不要乱点其它按键
 *
 * 5. 这里的清除错误是带查看错误信息的，如果要提高执行速度可以粗暴的让所有
 *    电机直接执行清理代码
 *
 * 6. 执行clearFault和getCurrentInfo程序及其卡，推测可能是调试过程是只用到
 *    一个电机，但函数段中6个节点都要执行指令函数，系统找不到其他的5个节点就卡死了
 *
 * 7. 这个版本的程序中点击了连接之后还要再点一次清除错误才会实时更新机器人中数据
 *    可以通过修改RobotState进行修改。因为更新必须大于Initialized，但是建立
 *    连接后的状态是connect，比Initialized小
 *
 * 8. 单关节调试后机器人的状态是Initialized，此时可以执行位置初始化记录q_bias
 *
 * 9. 由于只用一个电机调试，执行事件循环时会卡死，所以暂时把定时器关了，后面打开
 *
 * 10.换电脑运行的话要更改一下项目文件中的文件路径或者选择发布程序
 *
 * 11.有时候程序可能在首次运行时crash，重新打开就行，这常常在刚加载dll文件后出现
 *
 * 12.程序运行之前应该首先打开控制柜看看Epos有没有闪红灯
 *
 * 13.将所有弹框提示的错误信息都输出为Epos内部的错误信息
 *
 * 14.电机去使能的时候，三关节制动状态不对。看现象应该是制动偏慢
********************************************************************/



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    _virtualDamp = 20;

    // ------------------------MyoThread--------------------------------
    _myoThread = new QThread(this);
    _myoDevice = new MyoDevice;
    _myoDevice->moveToThread(_myoThread);

    connect(_myoThread,SIGNAL(started()),_myoDevice,SLOT(startEmgSample()));
    connect(_myoThread,SIGNAL(finished()),_myoThread,SLOT(deleteLater()));


    // ------------------------MotorThread------------------------------
    _motorThread = new QThread(this);
    _motorDevice = new MotorDevice;
    _motorDevice->moveToThread(_motorThread);
    _motorThread->start();

    connect(_motorThread,SIGNAL(finished()),_motorThread,SLOT(deleteLater()));

    // ------------------------SriThread--------------------------------
    _sriThread = new QThread(this);
    _sriDevice = new SriDevice;
    _sriDevice->moveToThread(_sriThread);
    _sriThread->start();

    connect(_sriThread,SIGNAL(finished()),_sriThread,SLOT(deleteLater()));
    connect(_sriDevice->getObject(),SIGNAL(configSystemFinished()),this,SLOT(showSriConfigState()));

    // ------------------------rlsThread--------------------------------

    _rlsThread1 = new QThread;
    _rlsThread2 = new QThread;
    _rlsThread3 = new QThread;
    _rlsThread4 = new QThread;
    _rlsThread5 = new QThread;
    _rlsThread6 = new QThread;



    _rlsJoint1 = new RasterRLS;
    _rlsJoint1->moveToThread(_rlsThread1);

    _rlsJoint2 = new MagnecticRLS;
    _rlsJoint2->moveToThread(_rlsThread2);

    _rlsJoint3 = new MagnecticRLS;
    _rlsJoint3->moveToThread(_rlsThread3);

    _rlsJoint4 = new MagnecticRLS;
    _rlsJoint4->moveToThread(_rlsThread4);

    _rlsJoint5 = new MagnecticRLS;
    _rlsJoint5->moveToThread(_rlsThread5);

    _rlsJoint6 = new MagnecticRLS;
    _rlsJoint6->moveToThread(_rlsThread6);



    _rlsThread1->start();
    _rlsThread2->start();
    _rlsThread3->start();
    _rlsThread4->start();
    _rlsThread5->start();
    _rlsThread6->start();

    // 目前只有234关节的磁编码器接到了采集卡上，因此只需要打开234关节的编码器即可
    // 由于234关节是检测到可用串口就打开，所以应该一开始就先打开1关节的编码器
    _rlsJoint1->openDevice();
    _rlsJoint2->openDevice();
    _rlsJoint3->openDevice();
    _rlsJoint4->openDevice();
    _rlsJoint5->openDevice();
    _rlsJoint6->openDevice();

    _rlsJoint1->startRealtimeSample();
    _rlsJoint2->startRealtimeSample();
    _rlsJoint3->startRealtimeSample();
    _rlsJoint4->startRealtimeSample();
    _rlsJoint5->startRealtimeSample();
    _rlsJoint6->startRealtimeSample();



    connect(_rlsThread1,SIGNAL(finished()),_rlsThread1,SLOT(deleteLater()));
    connect(_rlsThread2,SIGNAL(finished()),_rlsThread2,SLOT(deleteLater()));
    connect(_rlsThread3,SIGNAL(finished()),_rlsThread3,SLOT(deleteLater()));
    connect(_rlsThread4,SIGNAL(finished()),_rlsThread4,SLOT(deleteLater()));
    connect(_rlsThread5,SIGNAL(finished()),_rlsThread5,SLOT(deleteLater()));
    connect(_rlsThread6,SIGNAL(finished()),_rlsThread6,SLOT(deleteLater()));



    // -----------------------RobotController---------------------------
    _robot = new QRobot(this);

    // --------------------------QTimer---------------------------------
    _timer_control = new QTimer(this);
    connect(_timer_control,SIGNAL(timeout()),this,SLOT(mainControlLoop()));
    _timer_control->start(TS); // 控制周期为30ms

    // 配置机器人采集信息的定时器时间
    _timer_update_info = new QTimer(this);
    connect(_timer_update_info, SIGNAL(timeout()), this, SLOT(updateRobotInfo()));
    _timer_update_info->start(TW);      // 以5ms为周期进行数据的更新，先不打开该定时器


    //----------------------------ERROR---------------------------------
    connect(_myoDevice,SIGNAL(errorOccur(std::string)),this,SLOT(showError(std::string)));
    connect(this,SIGNAL(errorOccur(std::string)),this,SLOT(showError(std::string)));

    // ------------------------------------------------------------------
    _isSriStartFlag = FALSE;
    _isInAdmittanceFlag = FALSE;
    _isMyoStartFlag = FALSE;
    _isTraingFlag = FALSE;
    _isDangerFlag = FALSE;
    _isTestVelFlag = FALSE;

    // ------------------------------------------------------------------
    QStringList jointSel = {"joint 1","joint 2","joint 3","joint 4","joint 5","joint 6"};
    ui->comboBox_jointSelection->addItems(jointSel);
    ui->comboBox_jointSelection->setCurrentIndex(0);

    //--------------------------PyHandler--------------------------------
    _pyHandler = new PythonHandler;
    _pyThread = new QThread(this);
    _pyHandler->moveToThread(_pyThread);
    _pyThread->start();

    connect(_pyThread,SIGNAL(finished()), _pyThread, SLOT(deleteLater()));

    //--------------------------vel & accel & jerk-----------------------
    for(int i = 0; i < 2; i++){
        _vel_queue.push_back(0);
        _accel_queue.push_back(0);
        _jerk_queue.push_back(0);
    }

    for(int i = 0; i < 2; i++){
        _omega_queue.push_back(0);
        _alpha_queue.push_back(0);
    }

    // --------------------------set velocity units-----------------------
    test_joint_number = 4;
}

MainWindow::~MainWindow()
{
    if(_myoThread->isRunning())
    {
        _myoThread->quit();
        _myoThread->wait();
    }

    if(_motorThread->isRunning())
    {
        _motorThread->quit();
        _motorThread->wait();
    }

    if(_sriThread->isRunning())
    {
        _sriThread->quit();
        _sriThread->exit();
    }

    // 先停止连续数据采集，再关闭串口，最后delete
    _rlsJoint1->stopRealtimeSample();
    _rlsJoint2->stopRealtimeSample();
    _rlsJoint3->stopRealtimeSample();
    _rlsJoint4->stopRealtimeSample();
    _rlsJoint5->stopRealtimeSample();
    _rlsJoint6->stopRealtimeSample();



    _rlsJoint1->closeDevice();
    _rlsJoint2->closeDevice();
    _rlsJoint3->closeDevice();
    _rlsJoint4->closeDevice();
    _rlsJoint5->closeDevice();
    _rlsJoint6->closeDevice();



    if(_rlsThread1->isRunning())
    {
        _rlsThread1->quit();
        _rlsThread1->wait();
    }

    if(_rlsThread2->isRunning())
    {
        _rlsThread2->quit();
        _rlsThread2->wait();
    }

    if(_rlsThread3->isRunning())
    {
        _rlsThread3->quit();
        _rlsThread3->wait();
    }

    if(_rlsThread4->isRunning())
    {
        _rlsThread4->quit();
        _rlsThread4->wait();
    }

    if(_rlsThread5->isRunning())
    {
        _rlsThread5->quit();
        _rlsThread5->wait();
    }

    if(_rlsThread6->isRunning())
    {
        _rlsThread6->quit();
        _rlsThread6->wait();
    }

    if(_pyThread->isRunning())
    {
        _pyThread->quit();
        _pyThread->wait();
    }


    delete _rlsJoint1;
    delete _rlsJoint2;
    delete _rlsJoint3;
    delete _rlsJoint4;
    delete _rlsJoint5;
    delete _rlsJoint6;


    delete _myoDevice;
    delete _motorDevice;
    delete _sriDevice;
    delete _robot;
    delete _timer_control;

    // 自动析构？
    //    delete _pyHandler;
    delete _pyThread;

    delete ui;
}

// 该函数有待改进，程序只有一个true出口，不太符合逻辑，现在只有来判断是否已经归零结束
bool MainWindow::runToHome()
{
    // 获取电机需要的运动行程,motorDisplacement是电机需要运动的行程,单位时counts
    Eigen::Array<long, JRR_JOINT_NUMBER, 1> motorStroke = _robot->calMotorStroke();

    // 激活PPM，设置归零参数
    for(WORD i = 0; i < JRR_JOINT_NUMBER; ++i)
    {
        _motorDevice->getQMotorObject()->ePOS_ActiveProfilePositionMode(i+1);
        DWORD profileVolecity = 1 * static_cast<DWORD>(_robot->transRatio[i]);
        DWORD profileAcceleration = 5 * profileVolecity;
        DWORD profileDeceleration = 5 * profileVolecity;
        _motorDevice->getQMotorObject()->ePOS_SetPositionProfile(
                    i+1, profileVolecity, profileAcceleration, profileDeceleration);
    }

    // 设置完参数后开始运动
    for(WORD i = 0; i < JRR_JOINT_NUMBER; ++i)
    {
        long targetPosition = motorStroke[i];
        _motorDevice->getQMotorObject()->ePOS_MoveToPosition(i+1, targetPosition);
    }

    // 等待回零动作完成,超时时间为10s
    DWORD timeout = 10000;
    for(WORD i = 0; i < JRR_JOINT_NUMBER; ++i)
    {
        _motorDevice->getQMotorObject()->ePOS_WaitForTargetReached(i+1, timeout);
        _motorDevice->getQMotorObject()->ePOS_ActivePositionMode(i+1);
    }

    ui->textEdit_screen->append("Homing Finished");
    return true;
}

void MainWindow::updateJointAngle()
{
    // 第一步应该给机器人传递关节角参数，传入参数应该在构造完成之后就开始，这样编码器从一开始就工作
    double theta_1 = _rlsJoint1->requestAngleValue();
    double theta_2 = _rlsJoint2->requestAngleValue();
    double theta_3 = _rlsJoint3->requestAngleValue();
    double theta_4 = _rlsJoint4->requestAngleValue();
    double theta_5 = _rlsJoint5->requestAngleValue();
    double theta_6 = _rlsJoint6->requestAngleValue();

    // 传入编码器的读数，该函数同时包含更新关节角的功能
    _robot->setCurrentEncoderAngle(theta_1,theta_2,theta_3,theta_4,theta_5,theta_6);
}

void MainWindow::updateForceValue()
{
    _robot->setCurrentForceValue(_sriDevice->readForceData());
}

void MainWindow::startRunning()
{
    // 在启动按钮的槽函数下已经激活了PVM，这里直接设置参数使用就可以了
    long* tmpVel = _robot->getAdmMotorVel();

    // 以下代码为设置PVM的参数和调用PVM的运动函数
    for (WORD i = 0; i < JRR_JOINT_NUMBER; ++i)
    {
        _motorDevice->getQMotorObject()->ePOS_SetVelocityProfile(
                    i+1, static_cast<DWORD>(40 * tmpVel[i]),
                    static_cast<DWORD>(40 * tmpVel[i]));
        _motorDevice->getQMotorObject()->ePOS_MoveWithVelocity(i+1, tmpVel[i]);
    }
}

void MainWindow::readJointVelocity()
{
    // 注意这里的速度是电机的速度，需要进行传动比的转换才能映射到关节速度上
    double joint1, joint2, joint3, joint4, joint5, joint6;

    //     读取速度应该用平均值，使用ePOS_GetVelocityIs()可能存在一些数值方面的问题
    if(!_motorDevice->getQMotorObject()->ePOS_GetVelocityIsAveraged(1, joint1)
            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIsAveraged(2, joint2)
            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIsAveraged(3, joint3)
            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIsAveraged(4, joint4)
            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIsAveraged(5, joint5)
            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIsAveraged(6, joint6))
        //    if(!_motorDevice->getQMotorObject()->ePOS_GetVelocityIs(1, joint1)
        //            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIs(2, joint2)
        //            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIs(3, joint3)
        //            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIs(4, joint4)
        //            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIs(5, joint5)
        //            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIs(6, joint6))
    {
        qDebug() << "Fail to read joint velocity";
        return;
    }

    // setJointVelocity函数中应该包含正确的速度映射公式，传动比好说，关键是方向的转换
    _robot->setJointVelocity(joint1, joint2, joint3, joint4, joint5, joint6);
}


// moveToHome()添加了一段代码，其正确性有待验证
bool MainWindow::moveToHome()       // 里面已经写了激活VM的代码段
{
    // 添加一段机器人读取关节角度的代码，执行该代码会读取实时的编码器数据，已经转为关节角
    updateJointAngle();

#define DEBUG_MOVE_TO_HOME
#ifndef DEBUG_MOVE_TO_HOME
    Eigen::Array<double, 6, 1> jointAngle = _robot->getJointAngle();

    // 输出1关节的关节角
    qDebug() << "joint 1 angle is: " << jointAngle[0];
#endif

    // 从关节角计算电机需要的回零行程
    Eigen::Array<long, JRR_JOINT_NUMBER, 1> motorStroke = _robot->calMotorStroke();

#ifndef DEBUG_MOVE_TO_HOME
    std::cout << "motor stroke 1 is: " << motorStroke.transpose()[0] << std::endl;
#endif

    auto sign = Eigen::sign(motorStroke);

    // 设置关节的运转速度为1rpm，则电机的运转速度需要乘上传动比
    Eigen::Array<long, 6, 1> motorVel;
    for(int i = 0; i < JRR_JOINT_NUMBER; i++)
    {
        // 此时都是正值
        motorVel[i] = static_cast<long>( 2 * _robot->transRatio[i]);
    }

    // 设置电机的PVM参数
    for(WORD i = 0; i < JRR_JOINT_NUMBER; ++i)
    {
        DWORD ProfileAcceleration = static_cast<DWORD>(10 * motorVel[i]);
        DWORD ProfileDeceleration = static_cast<DWORD>(10 * motorVel[i]);
        _motorDevice->getQMotorObject()->ePOS_SetVelocityProfile(i+1,ProfileAcceleration,ProfileDeceleration);
    }

    // 计算每个电机需要运转的时间------------------------>ERROE
    Eigen::Array<long, 6, 1> motionTime;
    for(int i = 0; i < 6; i++)
    {
        // 时间以ms为单位
        motionTime[i] = static_cast<long>(
                    qAbs((60000.0 * motorStroke[i]) / (1.0* _robot->countsPerTurn[i] * motorVel[i])));
    }

#ifndef DEBUG_MOVE_TO_HOME
    std::cout << "motion time is: " << motionTime.transpose()[0] << std::endl;
#endif

    // 接下来根据此时间让电机开始运动，这里是六个电机依次运动，可能有点不是很合理
    for(WORD i = 0; i < JRR_JOINT_NUMBER; i++)
    {
        // 使用PVM的运动代码
        _motorDevice->getQMotorObject()->ePOS_MoveWithVelocity(i+1, motorVel[i] * sign[i]);

        // 使用线程的sleep函数
        QThread::msleep(  static_cast<unsigned int>( motionTime[i]));

        // 运动完成后，重置为0
        _motorDevice->getQMotorObject()->ePOS_MoveWithVelocity(i+1, 0);
    }


#ifndef DEBUG_MOVE_TO_HOME
    // 输出1关节速度
    long veltmp = 100;
    _motorDevice->getQMotorObject()->ePOS_GetVelocityMust(1, veltmp);
    qDebug() << "final velocity is: " << veltmp;
#endif

    qDebug() << "Homing finished";
    ui->textEdit_screen->append("Homing Finished");
    return true;
}

double MainWindow::getAcceleration()
{
    // 根据速度更新加速度
    double accel = (_vel_queue[1] - _vel_queue[0]) / TS * 1000;
    _accel_queue.erase(_accel_queue.begin());
    _accel_queue.push_back(accel);
    return accel;
}

double MainWindow::getJerk()
{
    // 根据加速度更新jerk
    double jerk = (_accel_queue[1] - _accel_queue[0]) / TS * 1000;
    _jerk_queue.erase(_jerk_queue.begin());
    _jerk_queue.push_back(jerk);
    return jerk;
}

void MainWindow::resetMotionInfo()
{
    // 重置速度/加速度/加加速度为0
    for(auto& c: _vel_queue){
        c = 0;
    }

    for(auto& c: _accel_queue){
        c = 0;
    }

    for(auto& c: _jerk_queue){
        c = 0;
    }

    //*************joint 5 *************
    for(auto& omega: _omega_queue)
    {
        omega = 0;
    }
    for(auto& alpha: _alpha_queue)
    {
        alpha = 0;
    }
}

bool MainWindow::joint_homing()
{

    // 添加一段机器人读取关节角度的代码，执行该代码会读取实时的编码器数据，已经转为关节角
    updateJointAngle();

    // 从关节角计算电机需要的回零行程
    Eigen::Array<long, JRR_JOINT_NUMBER, 1> motorStroke = _robot->calMotorStroke();
    auto sign = Eigen::sign(motorStroke);

    // 设置关节的运转速度为2rpm，则电机的运转速度需要乘上传动比
    Eigen::Array<long, 6, 1> motorVel;
    for(int i = 0; i < JRR_JOINT_NUMBER; i++)
    {
        // 此时都是正值
        motorVel[i] = static_cast<long>( 2 * _robot->transRatio[i]);
    }

    // 设置电机的PVM参数
    for(WORD i = 0; i < JRR_JOINT_NUMBER; ++i)
    {
        DWORD ProfileAcceleration = static_cast<DWORD>(10 * motorVel[i]);
        DWORD ProfileDeceleration = static_cast<DWORD>(10 * motorVel[i]);
        _motorDevice->getQMotorObject()->ePOS_SetVelocityProfile(i+1,ProfileAcceleration,ProfileDeceleration);
    }

    // 计算每个电机需要运转的时间------------------------>ERROE
    Eigen::Array<long, 6, 1> motionTime;
    for(int i = 0; i < 6; i++)
    {
        // 时间以ms为单位
        motionTime[i] = static_cast<long>(
                    qAbs((60000.0 * motorStroke[i]) / (1.0* _robot->countsPerTurn[i] * motorVel[i])));
    }

    // 回零采用PVM
    if(!_motorDevice->getQMotorObject()->ePOS_ActiveProfileVelocityMode(test_joint_number))
    {
        qDebug() << "Fail to active PVM";
    }

    // 只需要5关节进行回零
    _motorDevice->getQMotorObject()->ePOS_MoveWithVelocity(test_joint_number, motorVel[test_joint_number-1] * sign[test_joint_number-1], -3);
    QThread::msleep( static_cast<unsigned int>(motionTime[test_joint_number-1]));
    if(!_motorDevice->getQMotorObject()->ePOS_MoveWithVelocity(test_joint_number, 0, -3))
    {
        qDebug() << "Fail to halt";
    }

    qDebug() << "Homing finished";
    ui->textEdit_screen->append("Homing Finished");
    return true;
}

bool MainWindow::createConnetion(int time_stamp,
                                 double velocity,
                                 double acceleration,
                                 double jerk,
                                 double B,
                                 double distance,
                                 double vel_exp)
{
    _query.prepare("INSERT INTO episode_info (time_stamp, velocity, acceleration, jerk, B, distance, vel_exp)"
                   "VALUES (:time_stamp, :velocity, :acceleration, :jerk, :B, :distance, :vel_exp)");
    _query.bindValue(":time_stamp", time_stamp);
    _query.bindValue(":velocity", velocity);
    _query.bindValue(":acceleration", acceleration);
    _query.bindValue(":jerk", jerk);
    _query.bindValue(":B", B);
    _query.bindValue(":distance", distance);
    _query.bindValue(":vel_exp", vel_exp);

    _query.exec();

    return true;
}

void MainWindow::updateRobotInfo()
{


#define DEBUG_UPDATE_ROBOT_INFO
#ifndef DEBUG_UPDATE_ROBOT_INFO
    // 测试时间大概是10ms左右，比较符合预期
    {
        static clock_t time_start = clock();
        qDebug() << "Update robot info time is: " << clock() - time_start;
        time_start = clock();
    }
#endif

#if TRAIN_MODE == 1


#define DEBUG_SAMPLE_TIME
#ifndef DEBUG_SAMPLE_TIME
    clock_t start_sample = clock();
#endif
    updateJointAngle();     // 通过关节编码器更新关节角度信息，以5ms的频率进行更新
    if(_isSriStartFlag){
        // 读取一次传感器的输出数据，保存到robot中，以5ms的数据进行更新
        updateForceValue();
    }

    // 强化学习的标志量打开的时候进行关节速度的更新
    if(_isTraingFlag){
        readJointVelocity();            // 读取电机速度，以10ms的频率进行关节转速的更新
        _robot->updateCartesianVel();   // 以5ms的速度进行运动信息的更新
    }

#ifndef DEBUG_SAMPLE_TIME
    qDebug() << "sampling time is: " << clock() - start_sample;
#endif


#endif


#if TRAIN_MODE == 2         // 5ms的更新频率
#define DEBUG_SAMPLE_TIME
#ifndef DEBUG_SAMPLE_TIME
    clock_t start_sample = clock();
#endif
    updateJointAngle();     // 更新关节角度
    if(_isSriStartFlag){
        updateForceValue(); // 更新关节力矩
    }

#define JOINT_ENCODER_READ_VELOCITY
#ifndef JOINT_ENCODER_READ_VELOCITY
    // 使用关节编码器做差分计算角速度，证明噪声太大
    // 该段代码和下段的一样，作用都是更新 mean_omega
    if(_isTraingFlag)
    {
        // 添加一个函数，根据读取到的关节角度计算关节角速度
        _robot->get_angle_velocity(test_joint_number);
    }
#endif

//#define MOTOR_ENCODER_READ_VELOCITY
#ifndef MOTOR_ENCODER_READ_VELOCITY

    // 使用电机读取速度的api接口
    if(_isTraingFlag)
    {
        double omega_j5_motor = 0;
        if(!_motorDevice->getQMotorObject()->ePOS_GetVelocityIsAveraged(test_joint_number, omega_j5_motor, -3)) // 读取电机转速，转速为double类型，单位是rpm
        {
            return;
        }
        _robot->update_omega_joint(omega_j5_motor, test_joint_number);
    }
#endif

#ifndef DEBUG_SAMPLE_TIME
    qDebug() << "sampling time is: " << clock() - start_sample;
#endif

#endif
}

void MainWindow::showError(std::string Msg)
{
    QMessageBox msgBox;
    msgBox.setText(QString::fromStdString(Msg));
    msgBox.exec();
}
















// ***********************************************************************************
// 更新robot中的数据,机器人的主控制循环
void MainWindow::mainControlLoop()
{
#define DEBUG_TIME_BETWEEN_LOOP
#ifndef DEBUG_TIME_BETWEEN_LOOP

    // 查看两次控制循环之间的时间间隔，发现在60ms以上，偶尔有一些间隔能达到120ms
    static clock_t loop_start_time = clock();
    clock_t time_interval = clock() - loop_start_time;
    // qDebug() << "time interval between two loops is: " << time_interval;
    loop_start_time = clock();

#endif

#define DEBUG_LOOP_TIME
#ifndef DEBUG_LOOP_TIME
    // 在没有让电机运动的情况下，计算一个loop的时间，发现也是在4-6ms，没有超过预设的TS
    clock_t loop_start = clock();
#endif

#define DEBUG_READ_TIME
#ifndef DEBUG_READ_TIME
    // 统计一个控制周期中，传感器读取数据所花费的时间
    // 大概会花费4-5ms的时间，该时间包括了： 读取编码器数据的时间 + 读取力传感器数据的时间 + 读取电机转速的时间
    int read_time = clock();
#endif

#define DEBUG_ANGLE
#ifndef DEBUG_ANGLE
    // 检查关节角度是否正确，并且正常更新
    {
        Eigen::Array<double, 6, 1> jointAngle = _robot->getJointAngle();
        // std::cout << "joint angle is: " << jointAngle.transpose() << std::endl;
        // 输出4关节的关节角度
        //        std::cout << "joint " << test_joint_number << " angle is: "
        //                  << std::setw(16) << jointAngle[test_joint_number-1] << std::endl;
        _angle_t = jointAngle[test_joint_number - 1];
        // double velocity = (_angle_t - _angle_t_1) / 180 * PI / time_interval * 1000;
        double velocity = (_angle_t - _angle_t_1) / 180 * PI / TS * 1000;
        _angle_t_1 = _angle_t;
        qDebug() << "velocity is: " << velocity;
    }
#endif

#define DEBUG_MOTOR_ENCODER_READ_ANGLE_VELOCITY
#ifndef DEBUG_MOTOR_ENCODER_READ_ANGLE_VELOCITY
    // 使用电机读取速度的API接口进行读取速度
    double omega_motor = 0;
    if(!_motorDevice->getQMotorObject()->ePOS_GetVelocityIsAveraged(test_joint_number,omega_motor,-3))
    {
        qDebug() << "Fail to read motor velocity";
    }else{
        double motor_velocity = omega_motor * 2 / PI / 60;
        // qDebug() << "omega of motor is: " << motor_velocity;
        qDebug() << "omega of joint is: " << motor_velocity / _robot->transRatio[test_joint_number-1];
    }
#endif

    // 只有机器人处于Enable状态才能执行下面代码
    if(_robot->_robotState < RobotStates::Enabled){return;}
    // if(_isSriStartFlag == false){return;}

#define DEBUG_FORCE
#ifndef DEBUG_FORCE
    {
        // 检查力信息
        double* forceinsensor = _robot->getForceInSensor();

        static int cnt = 0;
        if(cnt++ % 30 == 0)
        {
            for(int i = 0; i< 6; ++i)
            {
                qDebug() << tr("channel %1 is %2").arg(i+1).arg(forceinsensor[i]);
            }
        }
    }
#endif

#if OPERATE_MODE == 1
    if(_isInAdmittanceFlag == false)
    {
        return;
    }

    // 计算导纳速度
    if(!_robot->updateAdmMotionPara())
    {
        return;
    }

    // 进行导纳运动
    startRunning();
#endif

#if OPERATE_MODE == 2

#define DEBUG_EPISODE_TIME
#ifndef DEBUG_EPISODE_TIME
    static int time_start_episode = clock();      // 对episode进行计时
#endif

    // 强化学习任务中，只有1关节和4关节可能到达限位，此为安全性检查
    Eigen::Array<double, 6, 1> jointAngle = _robot->getJointAngle();
    if(jointAngle[0] >= 110 || jointAngle[3] >= 44){
        _isDangerFlag = TRUE;
    }

    if(!_isTraingFlag){return;}     // 判断是否处于训练过程
    // readJointVelocity();            // 通过读取到的电机速度计算关节速度
    // _robot->updateCartesianVel();   // 计算jacobian，并更新末端的速度/加速度和jerk

    double velocity = _robot->getVelocity();
    // double acceleration = _robot->getAccel();
    // double jerk = _robot->getJerk();

    // 更新速度参数
    _vel_queue.erase(_vel_queue.begin());
    _vel_queue.push_back(velocity);

    double distance = _robot->getDistance();      // 获取走过的位移，以m为单位

    double acceleration = getAcceleration();
    double jerk = getJerk();


    // 暂时这么写，后期待优化
    if(_isDangerFlag){
        _virtualDamp = _pyHandler->trainOneStep(404, acceleration, qAbs(jerk) * qAbs(jerk) , distance);
    }else{
        _virtualDamp = _pyHandler->trainOneStep(velocity, acceleration, qAbs(jerk) * qAbs(jerk), distance);
    }

#ifndef DEBUG_READ_TIME
    // 1次数据库操作的时间小于1ms
    qDebug() << "read sensor info time is: " << clock() - read_time;
#endif


#define DEBUG_STEP_TIME
#ifndef DEBUG_STEP_TIME
    // 强化学习一次step时间小于1ms
    clock_t step_start_time = clock();
#endif

    // 此时Robot里面已经存储了正确的::<关节角度>---<关节速度>---<力传感器>::信息
    // _virtualDamp = _pyHandler->trainOneStep(velocity, acceleration, jerk);

#ifndef DEBUG_STEP_TIME
    qDebug() << "RL step time is: " << clock() - step_start_time;
#endif


#define DEBUG_DATABASE_TIME
#ifndef DEBUG_DATABASE_TIME
    clock_t dataBase_time = clock();
#endif

    // 数据库操作
    _db.transaction();  // 开启事务
    static int master_key = 0;
    createConnetion(master_key++, velocity, acceleration, jerk, _virtualDamp);

#ifndef DEBUG_DATABASE_TIME
    qDebug() << "DataBase time is: " << clock() - dataBase_time;
#endif

    // 根据_virtualDamp的值判断episode是否已经完成，一个episode结束到另一个episode开始的状态重置代码都在下面
    // 重置状态主要指的是机器人要回零
    if (_virtualDamp < 0)
    {
        qDebug() << "distance is: " << distance;
#ifndef DEBUG_EPISODE_TIME
        qDebug() << "episode time is: " << clock() - time_start;
#endif
        _db.commit();   // 提交数据
        for(WORD i = 0; i < JRR_JOINT_NUMBER; ++i){     // 每个episode结束时，设置速度为0
            _motorDevice->getQMotorObject()->ePOS_MoveWithVelocity(i+1, 0);     // 减速过程的加速度应该是以上一次配置的参数为准
        }

        QThread::sleep(1);      // 暂停1s
        if(moveToHome()){
            QThread::msleep(750);               // 线程睡眠750s
            _robot->resetMotionInfo();          // 重置jerk为0，改为重置状态，设置速度/加速度和jerk都是0
            resetMotionInfo();                  // 重置运动信息为0
            _pyHandler->trainOneStep(0, 0, 0, 0);  // 最后一次调用step
            createConnetion(master_key++, 99.0, 99.0, 99.0, _virtualDamp);  // 加入数据库的episode结束的标志位
            _isDangerFlag = FALSE;              // 重置标志变量为false
        }
        qDebug() << "-------------------episode start----------------------------";

#ifndef DEBUG_EPISODE_TIME
        time_start = clock();
#endif
        return;
    }


#define DEBUG_MOTOR_TIME
#ifndef DEBUG_MOTOR_TIME
    // 电机的运动时间大概是4-6ms左右
    clock_t run_time_motor = clock();
#endif

    // 更新电机运动参数
    if(!_robot->updateAdmMotionPara(_virtualDamp))
        //    if(!_robot->updateAdmMotionPara(17.5))  // 定导纳实验
    {
        return;
    }
    startRunning();     // 电机运动，等测试完之后再打开相应代码

#ifndef DEBUG_MOTOR_TIME
    qDebug() << "running time is: " << clock() - run_time_motor;
#endif

#ifndef DEBUG_LOOP_TIME
    qDebug() << "single loop time is: " << clock() - loop_start;
#endif

#endif


#if OPERATE_MODE == 3
    // 很危险,因为没有运动停止的代码，所以一般不要打开，该代码只是为了测试末端速度
    if(!_isTestVelFlag)
    {
        return;
    }



    // 更新电机速度
    _robot->testTermVel();

    // 电机开始运动
    startRunning();

#endif

#if OPERATE_MODE == 4
    if(!_isTraingFlag){return;}     // 判断是否处于训练过程
    double omega = _robot->get_omega_joint(); // 读取平均角速度

    // 更新速度参数 rad/s
    _omega_queue.erase(_omega_queue.begin());
    _omega_queue.push_back(omega);

    // 更新加速度
    double alpha = (_omega_queue[1] - _omega_queue[0]) / TS * 1000;
    _alpha_queue.erase(_alpha_queue.begin());
    _alpha_queue.push_back(alpha);

    // 更新jerk
    double jerk = (_alpha_queue[1] - _alpha_queue[0]) / TS * 1000;

    // 更新角度
    double angle = 0;
    _robot->get_angle(static_cast<int>( test_joint_number), angle);

    // 传入到神经网络中
    _virtualDamp = _pyHandler->trainOneStep(omega, alpha, jerk*jerk, angle);

    _db.transaction();  // 开启事务
    static int db_master_key = 0;
    if(_virtualDamp < 0)
    {
        _isTraingFlag = false;      // 暂停高速读取的定时器事件

          // 发送给电机的指令也都是以rpm为单位
        _motorDevice->getQMotorObject()->ePOS_SetVelocityMust(test_joint_number,0,-3);

        clock_t time_db_start = clock();
        _db.commit();   // 提交数据
        qDebug() << "commit time is: " << clock() - time_db_start;

        QThread::sleep(1);
        if(joint_homing())
        {
            QThread::msleep(750);
            updateJointAngle();             // 更新关节角
            _robot->resetMotionInfo(test_joint_number);     // 保证关节角每次的初始条件是从零位开始
            resetMotionInfo();
            _pyHandler->trainOneStep(0,0,0,0);

            // 加入数据库的episode结束的标志位
            createConnetion(db_master_key++, 99.0, 99.0, 99.0, _virtualDamp, angle, 0);
        }

        qDebug() << "------------episode start-----------------";

        _isTraingFlag = true;   // 再次打开高速读取数据的定时器事件
        return;
    }

    long omega_m5 = _robot->update_omega_motor(_virtualDamp, test_joint_number);
    //    long omega_m5 = _robot->update_omega_m5(18,test_joint_number);    // 定导纳实验
    double omega_exp = _robot->get_omega_exp();
    createConnetion(db_master_key++, omega, alpha, jerk, _virtualDamp, angle, omega_exp);

    //    _motorDevice->getQMotorObject()->ePOS_SetVelocityProfile(
    //                5,
    //                static_cast<DWORD>(40 * omega_m5),
    //                static_cast<DWORD>(40 * omega_m5));
    //    qDebug() << "guanjie 5 sudu :"<<omega_m5 ;
    //    _motorDevice->getQMotorObject()->ePOS_MoveWithVelocity(5, omega_m5, -3);

    // 使用VM做定导纳实验
    if(!_motorDevice->getQMotorObject()->ePOS_ActiveVelocityMode(test_joint_number)){
        qDebug() << "Fail to active velocity mode";
    }
    if(!_motorDevice->getQMotorObject()->ePOS_SetVelocityMust(test_joint_number, omega_m5, -3)){
        qDebug() << "Fail to exacute PM";
    }

#endif

    //----------------------------------------------------------------------
}

// ******************************************************************************












void MainWindow::showSriConfigState()
{
    ui->textEdit_screen->append(tr("Sri is ready"));
}

bool MainWindow::openDataBase()
{
    //--------------------------DataBase--------------------------------
    _db = QSqlDatabase::addDatabase("QSQLITE");     // 添加驱动
    _db.setDatabaseName("episode.db");              // 设置别名

    if(!_db.open())
    {
        qDebug() << _db.lastError().text();
        return false;
    }
    qDebug() << "DataBase has been opened";

    QSqlQuery query;
    _query = query;
    _query.exec("create table episode_info (time_stamp int primary key, "
                "velocity float, acceleration float, jerk float, B float, distance float, vel_exp float)");
    qDebug() << "Query SUCCESSFUL";

    return true;
}

void MainWindow::on_pushButton_buildConnect_clicked()
{

    if(ui->pushButton_buildConnect->text() == "BuildConnect")
    {
        if(!_motorDevice->getQMotorObject()->ePOS_OpendeviceByCAN())
        {
            ui->textEdit_screen->append(tr("##Connect Failed##"));
            QString str = QString(_motorDevice->getQMotorObject()->ePOS_ReadErrorMsg());
            emit errorOccur(str.toStdString());
            _robot->_robotState = RobotStates::ErrorOccured;
            return;
        }
        ui->textEdit_screen->append(tr("##Connection Succeeded##"));
        _robot->_robotState = RobotStates::Connected;
        ui->pushButton_buildConnect->setText("DestroyConnect");

    }else{
        if(!_motorDevice->getQMotorObject()->ePOS_CloseSystem())
        {
            ui->textEdit_screen->append(tr("##Disconnection Failed##"));
            QString str = QString(_motorDevice->getQMotorObject()->ePOS_ReadErrorMsg());
            emit errorOccur(str.toStdString());
            _robot->_robotState = RobotStates::ErrorOccured;
            return;
        }
        ui->textEdit_screen->append(tr("##Disconnection Succeeded##"));
        _robot->_robotState = RobotStates::JustStarted;
        ui->pushButton_buildConnect->setText("BuildConnect");
    }
}

void MainWindow::on_pushButton_enable_clicked()
{
    if(ui->pushButton_enable->text() == "Enable")
    {
        for (WORD i = JRR_JOINT_NUMBER; i >= 1; --i) {
            if(_motorDevice->getQMotorObject()->ePOS_ActivePositionMode(i)      // 使用位置模式做位置保持
                    && _motorDevice->getQMotorObject()->ePOS_SetEnableState(i))
            {
                // 只要一个执行成功就应该转换功能
                ui->textEdit_screen->append(tr("##Joint %1 enabled##").arg(QString::number(i)));
                ui->pushButton_enable->setText("Disable");
                ui->pushButton_release->setEnabled(false);

                // 阻塞motor线程 0.75s
                _motorThread->msleep(750);
            }
            else {
                QString str = tr("Error occur in enabling motor %1").arg(QString::number(i));
                ui->textEdit_screen->append(tr("##%1##").arg(str));

                str = QString(_motorDevice->getQMotorObject()->ePOS_ReadErrorMsg());
                emit errorOccur(str.toStdString());
                _robot->_robotState = RobotStates::ErrorOccured;
                return;
            }
        }

        // 只有当全部电机都正常启动的enable的情况下才置机器人状态为RobotStates::Enabled;
        _robot->_robotState = RobotStates::Enabled;
    }else{
        for(WORD i = JRR_JOINT_NUMBER; i >= 1 ; --i)
        {
            if(_motorDevice->getQMotorObject()->ePOS_SetDisableState(i))
            {
                ui->textEdit_screen->append(tr("##Joint %1 has been disabled##").arg(QString::number(i)));
                ui->pushButton_enable->setText("Enable");
                ui->pushButton_release->setEnabled(true);

                // 不应该延时关闭，三关节制动偏慢
                // _motorThread->msleep(500);

            }else{
                QString str = tr("Error occur in disabling motor %1").arg(QString::number(i));
                ui->textEdit_screen->append(tr("##%1##").arg(str));

                str = QString(_motorDevice->getQMotorObject()->ePOS_ReadErrorMsg());
                emit errorOccur(str.toStdString());

                _robot->_robotState = RobotStates::ErrorOccured;
                return;
            }
        }
        // 只有所有的电机全部都disable的情况下机器人的状态才是Initialized
        _robot->_robotState = RobotStates::Initialized;
    }
}

void MainWindow::on_pushButton_clearFault_clicked()
{
    for(WORD i = 1; i < JRR_JOINT_NUMBER; ++i)
    {
        if(_motorDevice->getQMotorObject()->ePOS_ReadMotorState(i) != MotorState::Fault)
        {
            continue;   // 执行下一次循环
        }
        QString str = tr("##Motor %1 is in Fault state##").arg(QString::number(i));
        ui->textEdit_screen->append(str);
        emit errorOccur(str.toStdString());

        if(!_motorDevice->getQMotorObject()->ePOS_ClearFault(i))
        {
            QString str = QString(_motorDevice->getQMotorObject()->ePOS_ReadErrorMsg());
            emit errorOccur(str.toStdString());
            return;
        }
        ui->textEdit_screen->append(tr("Motor %1 has been cleaned").arg(QString::number(i)));
    }

    // 执行完clearFault之后，机器人的状态置为Initialized
    _robot->_robotState = RobotStates::Initialized;
    ui->textEdit_screen->append(tr("There is no motor in fault"));
}

void MainWindow::on_pushButton_release_pressed()
{
    WORD currentJointSel = static_cast<WORD>(ui->comboBox_jointSelection->currentIndex()) + 1;
    if(_motorDevice->getQMotorObject()->ePOS_ActiveCurrentMode(currentJointSel)             // 激活电流模式
            && _motorDevice->getQMotorObject()->ePOS_SetCurrentMust(currentJointSel,0)      // 设置电流值
            && _motorDevice->getQMotorObject()->ePOS_SetEnableState(currentJointSel))       // 电机使能
    {
        ui->textEdit_screen->append(tr("Joint %1 has been released").arg(currentJointSel));
        _robot->_robotState = RobotStates::Released;
        return;
    }
    _robot->_robotState = RobotStates::ErrorOccured;
    QString str = QString(_motorDevice->getQMotorObject()->ePOS_ReadErrorMsg());
    emit errorOccur(str.toStdString());
    ui->textEdit_screen->append(tr("Fail to release Brake %1").arg(QString::number(currentJointSel)));
}

void MainWindow::on_pushButton_release_released()
{
    WORD currentJointSel = static_cast<WORD>(ui->comboBox_jointSelection->currentIndex()) + 1;
    if(_robot->_robotState != RobotStates::Released)
    {
        return;
    }
    if(!_motorDevice->getQMotorObject()->ePOS_SetDisableState(currentJointSel))
    {
        _robot->_robotState = RobotStates::ErrorOccured;
        QString str = QString(_motorDevice->getQMotorObject()->ePOS_ReadErrorMsg());
        emit errorOccur(str.toStdString());
        ui->textEdit_screen->append(tr("Fail to brake joint %1").arg(QString::number(currentJointSel)));
        return;
    }
    ui->textEdit_screen->append(tr("Joint %1 has been blocked").arg(currentJointSel));
    _robot->_robotState = RobotStates::Initialized;
}

void MainWindow::on_pushButton_robotState_clicked()
{
    int state = _robot->_robotState;

    switch (state) {
    case 0:
        ui->textEdit_screen->append(tr("Robot State: JustStarted"));
        break;
    case 1:
        ui->textEdit_screen->append(tr("Robot State: Connected"));
        break;
    case 2:
        ui->textEdit_screen->append(tr("Robot State: ErrorOccured"));
        break;
    case 3:
        ui->textEdit_screen->append(tr("Robot State: Initialized"));
        break;
    case 4:
        ui->textEdit_screen->append(tr("Robot State: Enabled"));
        break;
    case 5:
        ui->textEdit_screen->append(tr("Robot State: Released"));
        break;
    case 6:
        ui->textEdit_screen->append(tr("Robot State: Teach"));
        break;
    case 7:
        ui->textEdit_screen->append(tr("Robot State: AdmittanceMode"));
        break;
    default:
        break;
    }
}

void MainWindow::on_pushButton_moveForward_pressed()
{
    WORD currentJointSel = static_cast<WORD>(ui->comboBox_jointSelection->currentIndex()) + 1;
    //    double jointVelocity = ui->spinBox_velocity->value() / 10.0;            // 设置为0-12rpm
    double jointVelocity = ui->spinBox_velocity->value();            // 设置为0-12rpm
    double motorVelocity = jointVelocity * _robot->transRatio[currentJointSel-1];           // 关节速度映射到电机转速



    // 以下为使用PVM做速度控制

    DWORD ProfileAcceleration = static_cast<DWORD>(5 * motorVelocity);                      // 5倍准则
    DWORD ProfileDeceleration = static_cast<DWORD>(5 * motorVelocity);

    if(_motorDevice->getQMotorObject()->ePOS_ActiveProfileVelocityMode(currentJointSel)     // 激活PVM模式
            && _motorDevice->getQMotorObject()->ePOS_SetVelocityProfile(
                currentJointSel,ProfileAcceleration,ProfileDeceleration)        // 设置PVM参数
            && _motorDevice->getQMotorObject()->ePOS_MoveWithVelocity(
                currentJointSel,static_cast<long>(
                    motorVelocity * _robot->directions[currentJointSel-1])))    // 开始运动
    {
        _robot->_robotState = RobotStates::Released;
        return;
    }


    _robot->_robotState = RobotStates::ErrorOccured;
    QString str = QString(_motorDevice->getQMotorObject()->ePOS_ReadErrorMsg());
    emit errorOccur(str.toStdString());
}

void MainWindow::on_pushButton_moveForward_released()
{
    WORD currentJointSel = static_cast<WORD>(ui->comboBox_jointSelection->currentIndex()) + 1;

    if(_motorDevice->getQMotorObject()->ePOS_HaltVelocityMovement(currentJointSel)          // 停止运动
            && _motorDevice->getQMotorObject()->ePOS_ActivePositionMode(currentJointSel))   // 启用位置模式
    {
        _robot->_robotState = RobotStates::Released;
        return;
    }
    _robot->_robotState = RobotStates::ErrorOccured;
    QString str = QString(_motorDevice->getQMotorObject()->ePOS_ReadErrorMsg());
    emit errorOccur(str.toStdString());
}


void MainWindow::on_pushButton_moveBackward_pressed()
{
    WORD currentJointSel = static_cast<WORD>(ui->comboBox_jointSelection->currentIndex()) + 1;
    double jointVelocity = ui->spinBox_velocity->value() / 10.0;            // 设置为0-12rpm

    // 关节速度映射到电机转速
    double motorVelocity = jointVelocity * _robot->transRatio[currentJointSel-1];
    DWORD ProfileAcceleration = static_cast<DWORD>(5 * motorVelocity);      // 5倍准则
    DWORD ProfileDeceleration = static_cast<DWORD>(5 * motorVelocity);

    if(_motorDevice->getQMotorObject()->ePOS_ActiveProfileVelocityMode(currentJointSel)     // 启用PVM
            && _motorDevice->getQMotorObject()->ePOS_SetVelocityProfile(
                currentJointSel,ProfileAcceleration,ProfileDeceleration)    // 设置参数
            && _motorDevice->getQMotorObject()->ePOS_MoveWithVelocity(
                currentJointSel,static_cast<long>
                (-1 * motorVelocity * _robot->directions[currentJointSel-1])))   // 开始运动
    {
        _robot->_robotState = RobotStates::Released;
        return;
    }
    _robot->_robotState = RobotStates::ErrorOccured;
    QString str = QString(_motorDevice->getQMotorObject()->ePOS_ReadErrorMsg());
    emit errorOccur(str.toStdString());
}

void MainWindow::on_pushButton_moveBackward_released()
{
    WORD currentJointSel = static_cast<WORD>(ui->comboBox_jointSelection->currentIndex()) + 1;

    if(_motorDevice->getQMotorObject()->ePOS_HaltVelocityMovement(currentJointSel)    // 停止运动
            && _motorDevice->getQMotorObject()->ePOS_ActivePositionMode(currentJointSel))    // 激活PM
    {
        _robot->_robotState = RobotStates::Released;
        return;
    }
    _robot->_robotState = RobotStates::ErrorOccured;
    QString str = QString(_motorDevice->getQMotorObject()->ePOS_ReadErrorMsg());
    emit errorOccur(str.toStdString());
}


void MainWindow::on_pushButton_configSri_clicked()
{
    _sriDevice->ConfigSystem();
}


void MainWindow::on_pushButton_startSri_clicked()
{
    if(ui->pushButton_startSri->text() == "StartSri")
    {
        _isSriStartFlag = true;
        ui->pushButton_startSri->setText("StopSri");
        _sriDevice->startRealTimeSample();
        ui->textEdit_screen->append(tr("Sri is started"));
    }else {
        _isSriStartFlag = false;
        ui->pushButton_startSri->setText("StartSri");
        _sriDevice->stopRealTimeSample();
        ui->textEdit_screen->append(tr("Sri is stopped"));
    }
}

void MainWindow::on_pushButton_forceInit_clicked()
{

    // 采用均值滤波的形式对传感器的初始值进行滤波,能不能写到传感器里面
    Eigen::Matrix<double, SRI_CHANNEL_NUMBER, 1> forceTmp =
            Eigen::Matrix<double,SRI_CHANNEL_NUMBER,1>::Zero();

    // 有个小问题，这里没有延迟函数，5000次中可能重复了很多次
    for(int i = 0; i < 5000; ++i)
    {
        forceTmp += _sriDevice->readForceData();
        //        _sriThread->msleep(1);
    }
    forceTmp /= 5000;

    _robot->setForceBias(forceTmp);
    ui->textEdit_screen->append(tr("----Force bias has been logged----"));

    double* tmpForceBias = _robot->getForceBias();
    for(int i = 0; i < SRI_CHANNEL_NUMBER; ++i)
    {
        qDebug() << tr("channel %1: %2").arg(i+1).arg(tmpForceBias[i]);
    }
    //    ui->pushButton_forceInit->setEnabled(false);
}

void MainWindow::on_pushButton_startAdmittanceMode_clicked()
{
    if(ui->pushButton_startAdmittanceMode->text() == "START")
    {
        _isInAdmittanceFlag = TRUE;

#define PVM
#ifndef PVM
        // 现在使用的是VM做导纳控制，所以下面的代码不再需要
        // 使用PVM 做导纳运动,
        for(WORD i = 1; i <= JRR_JOINT_NUMBER; ++i)
        {
            _motorDevice->getQMotorObject()->ePOS_ActiveProfileVelocityMode(i);
        }

#endif

        // 使用VM模式做导纳控制
        for(WORD i = 1; i <= JRR_JOINT_NUMBER; ++i)
        {
            _motorDevice->getQMotorObject()->ePOS_ActiveVelocityMode(i);
        }

        ui->pushButton_startAdmittanceMode->setText("STOP");

    }else{
        _isInAdmittanceFlag = FALSE;

        // 重新设置为PM,做位置保持
        for(WORD i = 1; i <= JRR_JOINT_NUMBER; ++i)
        {
            _motorDevice->getQMotorObject()->ePOS_HaltVelocityMovement(i);  // 停止运动
            _motorDevice->getQMotorObject()->ePOS_ActivePositionMode(i);    // 使用位置模式做位置保持
        }
        ui->pushButton_startAdmittanceMode->setText("START");
    }
}

void MainWindow::on_pushButton_quickStop_clicked()
{
    for(WORD i = 0; i < JRR_JOINT_NUMBER; ++i)
    {
        _motorDevice->getQMotorObject()->ePOS_SetQuickStopState(i+1);
    }
}

void MainWindow::on_pushButton_startMyo_clicked()
{
    if(ui->pushButton_startMyo->text() == "startMyo")
    {
        _myoThread->start();
        _isMyoStartFlag = TRUE;
        ui->pushButton_startMyo->setText("stopMyo");

        // 使用Myo对机器人的导纳参数进行赋值
        connect(_myoDevice->getMyoObject(),SIGNAL(changeToHighPara()),_robot,SLOT(setHighAdmittancePara()));
        connect(_myoDevice->getMyoObject(),SIGNAL(changeToLowPara()),_robot,SLOT(setLowAdmittancePara()));

    }else{
        _myoDevice->stopEmgSample();
        ui->pushButton_startMyo->setEnabled(false);
        _isMyoStartFlag = FALSE;
    }
}

// 下面是我新加的程序块
void MainWindow::on_setAdmittancePara_clicked()
{
    // 还未添加任何函数体，是不是谁动过我的程序
    Eigen::Array<double, 6, 1> massTmp;
    massTmp << ui->spinBox_m1->value(), ui->spinBox_m2->value(),
            ui->spinBox_m3->value(), ui->spinBox_m4->value(),
            ui->spinBox_m5->value(), ui->spinBox_m6->value();
    _robot->setVirtualMassValue(massTmp);

    Eigen::Array<double, 6, 1> dampTmp;
    dampTmp << ui->spinBox_d1->value(), ui->spinBox_d2->value(),
            ui->spinBox_d3->value(), ui->spinBox_d4->value(),
            ui->spinBox_d5->value(), ui->spinBox_d6->value();
    _robot->setVirtualDampValue(dampTmp);
}

void MainWindow::on_moveToHome_clicked()
{
    //        runToHome();

    // 使用的是VM做回零动作
    for(WORD i = 0; i < JRR_JOINT_NUMBER; ++i)
    {
        _motorDevice->getQMotorObject()->ePOS_ActiveProfileVelocityMode(i+1);
    }
    moveToHome();
}

void MainWindow::on_ptn_pythonInit_clicked()
{
    // 初始化只能执行一次
    ui->ptn_pythonInit->setEnabled(false);

    // 初始化Python
    _pyHandler->initPython();

    // 获取python脚本
    _pyHandler->getPyScript();

    // 测试部分代码，真正运行时需要关闭
    // _pyHandler->testCallMethod();

    //运行
    _pyHandler->startRun();

}

void MainWindow::on_ptn_suspendTrain_clicked()
{
    // 置标志变量为False，停止导纳运动
    _isTraingFlag = FALSE;
}

void MainWindow::on_ptn_startTrain_clicked()
{  

#define CARTESSIAN_OPERATION_MODE
#ifndef CARTESSIAN_OPERATION_MODE
    // 设置电机的工作模式为PVN，进行导纳训练，全关节运动时用的代码
    for(WORD i = 1; i <= JRR_JOINT_NUMBER; ++i)
    {
        _motorDevice->getQMotorObject()->ePOS_ActiveProfileVelocityMode(i);
    }
#endif

#define VM
#ifndef VM
    // 设置电机的工作模式为VM，进行导纳运动
    for(WORD i = 1; i <= JRR_JOINT_NUMBER; ++i)
    {
        // 激活VM
        _motorDevice->getQMotorObject()->ePOS_ActiveVelocityMode(i);
    }
#endif

    // 置标志变量为True
    _isTraingFlag = TRUE;

    // 打开数据库
    openDataBase();
}


void MainWindow::on_ptn_testVel_clicked()
{
    _isTestVelFlag = true;

    for(WORD i = 0; i < JRR_JOINT_NUMBER; ++i)
    {
        _motorDevice->getQMotorObject()->ePOS_ActiveProfileVelocityMode(i+1);
    }
}


void MainWindow::on_ptn_joint5_enable_clicked()
{
    if(ui->ptn_joint5_enable->text() == "joint5_Enable")
    {
        // 设置五关节使能
        if(!_motorDevice->getQMotorObject()->ePOS_SetEnableState(test_joint_number))
        {
            qDebug() << "Fail to enable joint 5";
            _robot->_robotState = RobotStates::ErrorOccured;
            return;
        }
        if(!_motorDevice->getQMotorObject()->ePOS_ActiveProfileVelocityMode(test_joint_number)
                || !_motorDevice->getQMotorObject()->ePOS_MoveWithVelocity(test_joint_number,0,-3))
        {
            qDebug() << "Fail to active PVM";
            _robot->_robotState = RobotStates::ErrorOccured;
            return;
        }

        ui->textEdit_screen->append(tr("###Joint 5 has been enabled"));
        ui->ptn_joint5_enable->setText("Disable");
        _robot->_robotState = RobotStates::Enabled;
    }else{
        if(!_motorDevice->getQMotorObject()->ePOS_SetDisableState(test_joint_number))
        {
            qDebug() << "Fail to disable joint 5";
            _robot->_robotState = RobotStates::ErrorOccured;
            return;
        }
        ui->textEdit_screen->append(tr("###Joint 5 has been disabled"));
        ui->ptn_joint5_enable->setText("joint5_Enable");
        _robot->_robotState = RobotStates::Initialized;
    }
}

void MainWindow::on_ptn_joint5_home_clicked()
{
    // 关节5执行回零程序
    joint_homing();
}

void MainWindow::on_ptn_change_units_clicked()
{
    // 2 读取
    char type;
    qDebug() << "is here";
    if(!_motorDevice->getQMotorObject()->ePOS_GetVelocityUnits(4, type))
    {
        qDebug() << "Fail to get units";
        return;
    }
    std::cout << "units is: "<< type << std::endl;
    qDebug() <<  "units is: "<< type;

}

void MainWindow::on_ptn_set_units_clicked()
{
    if(!_motorDevice->getQMotorObject()->ePOS_SetMilliVelocityUnits()){
        qDebug() << "Fail to set velocity units";
    }else{
        qDebug() << "Velocity units has been set";
    }
}


void MainWindow::on_pushButton_2_clicked()
{
    if(!_motorDevice->getQMotorObject()->ePOS_ActiveVelocityMode(test_joint_number))
    {
        qDebug() << "Fail to active VM";
    }else{
        qDebug() << "active VM";
    }

}

void MainWindow::on_pushButton_3_clicked()
{
    if(!_motorDevice->getQMotorObject()->ePOS_SetVelocityMust(test_joint_number, 10,-3))
    {
        qDebug() << "Fail to set velocity must";
    }else{
        qDebug() << "set velocity must 10rmp";
    }
}
