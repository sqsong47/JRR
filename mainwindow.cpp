#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QStringList>
#include <QDebug>
#include <iostream>
#include <QThread>
#include <time.h>


/* 条件编译指令，可进行扩展
 * OPERATE_MODE = 1 代表处于普通的定导纳运动
 * OPERATE_MODE = 2 代表处于强化学习训练的变导纳过程
 * 使用时可以通过修改该宏的值进行操作模式选择
*/
#define OPERATE_MODE 2

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
    _timer = new QTimer(this);
    connect(_timer,SIGNAL(timeout()),this,SLOT(mainControlLoop()));
    _timer->start(TS); // 控制周期为30ms


    //----------------------------ERROR---------------------------------
    connect(_myoDevice,SIGNAL(errorOccur(std::string)),this,SLOT(showError(std::string)));
    connect(this,SIGNAL(errorOccur(std::string)),this,SLOT(showError(std::string)));

    // ------------------------------------------------------------------
    _isSriStartFlag = FALSE;
    _isInAdmittanceFlag = FALSE;
    _isMyoStartFlag = FALSE;
    _isTraingFlag = FALSE;

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
    delete _timer;

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

#define PVM
#ifndef PVM
    // 以下代码为设置PVM的参数和调用PVM的运动函数
    for (WORD i = 0; i < JRR_JOINT_NUMBER; ++i)
    {
        _motorDevice->getQMotorObject()->ePOS_SetVelocityProfile(
                    i+1, static_cast<DWORD>(5 * tmpVel[i]),
                    static_cast<DWORD>(5 * tmpVel[i]));
        _motorDevice->getQMotorObject()->ePOS_MoveWithVelocity(i+1, tmpVel[i]);
    }
#endif

    // 现在直接改成VM的运动模式
    for(WORD i = 0; i < JRR_JOINT_NUMBER; ++i)
    {
        // 使用VM进行机器人运动
        _motorDevice->getQMotorObject()->ePOS_SetVelocityMust(i+1, tmpVel[i]);
    }
}

void MainWindow::readJointVelocity()
{
    // 注意这里的速度是电机的速度，需要进行传动比的转换才能映射到关节速度上
    long joint1, joint2, joint3, joint4, joint5, joint6;
    if(!_motorDevice->getQMotorObject()->ePOS_GetVelocityIs(1, joint1)
            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIs(2, joint2)
            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIs(3, joint3)
            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIs(4, joint4)
            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIs(5, joint5)
            || !_motorDevice->getQMotorObject()->ePOS_GetVelocityIs(6, joint6))
    {
        // qDebug() << "Fail to read joint velocity";
        return;
    }

    // setJointVelocity函数中应该包含正确的速度映射公式，传动比好说，关键是方向的转换
    _robot->setJointVelocity(joint1, joint2, joint3, joint4, joint5, joint6);
}

bool MainWindow::moveToHome()
{
    // 获取电机需要的运动行程,motorDisplacement是电机需要运动的行程
    // 有一个问题calMotorStroke()是不是实时更新的
    // 虽然是可以每次进入主循环都更新，但是由于执行重置状态操作时，主循环会卡死，还是只能读取一次，不可以实时读取
    Eigen::Array<long, JRR_JOINT_NUMBER, 1> motorStroke = _robot->calMotorStroke();

    // 设置机器人的回零时间为3s，即0.05min
    int homingTime = 3;

    // 每个电机需要执行的速度,rpm
    long motorVel[6] = {0};
    for(int i = 0; i < 6; i++)
    {
        // motorStroke存储的counts，先要转化为转数,再转化为rpm
        motorVel[i] = static_cast<long>(
                    motorStroke[i] / _robot->countsPerTurn[i] / homingTime / 60);
    }

    // 接下来应该考虑让电机以速度motorVel，运行3s
    for(WORD i = 0; i < JRR_JOINT_NUMBER; ++i)
    {
        // 开始运动
        _motorDevice->getQMotorObject()->ePOS_SetVelocityMust(i+1, motorVel[i]);
    }

    QThread::sleep(3);

    // 重置为0
    for(WORD i = 0; i < JRR_JOINT_NUMBER; ++i)
    {
        // 开始运动
        _motorDevice->getQMotorObject()->ePOS_SetVelocityMust(i+1, 0);
    }

    ui->textEdit_screen->append("Homing Finished");
    return true;

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
    // 通过编码器更新关节角度，已证实执行成功
    updateJointAngle();

#ifndef DEBUG
    // 检查关节角度是否正确
    Eigen::Array<double, 6, 1> jointAngle = _robot->getJointAngle();
    for(int i = 0; i < 6; ++i)
    {
        qDebug() << tr("joint %1 angle is : %2").arg(i+1).arg(jointAngle[i]);
    }
#endif

    // 只有机器人处于Enable状态才能执行下面代码
    if(_robot->_robotState < RobotStates::Enabled){return;}
    if(_isSriStartFlag == false){return;}

    // 更新力信息，已证实执行成功
    updateForceValue();

#ifndef DEBUG
    // 检查力信息
    double* forceinsensor = _robot->getForceInSensor();
    for(int i = 0; i< 6; ++i)
    {
        qDebug() << tr("channel %1 is %2").arg(i+1).arg(forceinsensor[i]);
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

    // 判断是否处于训练过程
    if(!_isTraingFlag){return;}

    // 更新关节的速度值,该函数读取电机的转速进而进行关节速度的换算
    readJointVelocity();

    // 此时Robot里面已经存储了正确的::<关节角度>---<关节速度>---<力传感器>::信息
    _virtualDamp = _pyHandler->trainOneStep(_robot->getVelocity(),
                                            _robot->getAccel(),
                                            _robot->getReward());

#define DEBUG3
#ifndef DEBUG3
    static int i = 0;
    i++;
    _virtualDamp = _pyHandler->trainOneStep(1 + i, 2 + i, 3 + i);
#endif


#define DEBUG1
#ifndef DEBUG1
    // 输出强化学习的相关信息
    static int i = 0;
    i++;
    if(i % 30 == 0)
    {
        qDebug()<< i
                << "  Admittance: " << _virtualDamp
                << " vel: " << _robot->getVelocity()
                << " accel: " << _robot->getAccel()
                << " reword: " <<  _robot->getReward();
    }
#endif


#define DEBUG2
#ifndef DEBUG2
    // 输出导纳值
    qDebug() << "Admittance: " << _virtualDamp;
#endif

    // 根据_virtualDamp的值判断episode是否已经完成，一个episode结束到另一个episode开始的状态重置代码都在下面
    // 重置状态主要指的是机器人要回零
    if (_virtualDamp < 0)
    {
        // 暂停训练，从主循环退出
        _isTraingFlag = false;
        qDebug() << "episode finished" ;

        // QThread::msleep(750);
        // 添加机器人回零代码，似乎有点问题，step函数此时是不是一定暂停了？

        //设置速度为0
        for(WORD i = 0; i < JRR_JOINT_NUMBER; ++i)
        {
            _motorDevice->getQMotorObject()->ePOS_SetVelocityMust(i+1, 0);
        }

        // 暂停1s
        QThread::sleep(1);

        // runToHome()需要修改，此时用的模式PPM，需要改成VM
        // if (runToHome())
        if(moveToHome())
        {
            qDebug() << "episode wait " ;

#define PVM
#ifndef PVM
            // 由于回零程序还有导纳控制的程序都采用的是VM的模式，现在程序中不需要再额外的添加状态切换代码
            // 设置电机的工作模式为PVM
            for(WORD i = 1; i <= 6; ++i)
            {
                _motorDevice->getQMotorObject()->ePOS_ActiveProfileVelocityMode(i);
            }
#endif


#define DEBUG4
#ifndef DEBUG4
            // 置标志变量为true
            qDebug()<< "episode: "
                    << " Admittance: " << _virtualDamp
                    << " vel: " << _robot->getVelocity()
                    << " accel: " << _robot->getAccel()
                    << " reword: " <<  _robot->getReward();
#endif

            // 重置jerk为0
            _robot->resetJerk();

            // 最后一次调用step
            _pyHandler->trainOneStep(0, 0, 0);

            // 暂停1s用于速度的切换
            QThread::sleep(1);

            // 置标志量为true，开启下次训练
            _isTraingFlag = true;
        }
        return;
    }

    // 更新电机运动参数
    if(!_robot->updateAdmMotionPara(_virtualDamp))
    {
        return;
    }

    // 电机运动，等测试完之后再打开相应代码
    startRunning();
#endif

    //----------------------------------------------------------------------
}

// ******************************************************************************






void MainWindow::showSriConfigState()
{
    ui->textEdit_screen->append(tr("Sri is ready"));
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
    double jointVelocity = ui->spinBox_velocity->value() / 10.0;            // 设置为0-12rpm
    double motorVelocity = jointVelocity * _robot->transRatio[currentJointSel-1];           // 关节速度映射到电机转速

    // 使用VM模式做匀速运动，因为感觉VM的PID应该更好一点
    /*
    if(_motorDevice->getQMotorObject()->ePOS_ActiveVelocityMode(currentJointSel)            // 激活PV
            && _motorDevice->getQMotorObject()->ePOS_SetVelocityMust(currentJointSel, static_cast<long>(motorVelocity * _robot->directions[currentJointSel-1])))
    {
        _robot->_robotState = RobotStates::Released;
        return;
    }
    */


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
    runToHome();
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

    // 观测状态，进行最后一步训练训练
    ;

    // 调用停止训练的接口函数
    // _pyHandler->suspendTrain();

    // 置标志变量为False，停止导纳运动
    _isTraingFlag = FALSE;

    // 可能需要一个延时函数
    _motorThread->msleep(1000);

    // 执行机器人的归零程序
    runToHome();


}

void MainWindow::on_ptn_startTrain_clicked()
{

#define PVM
#ifndef PVM
    // 设置电机的工作模式为PVN，进行导纳训练
    for(WORD i = 1; i <= JRR_JOINT_NUMBER; ++i)
    {
        _motorDevice->getQMotorObject()->ePOS_ActiveProfileVelocityMode(i);
    }
#endif

    // 设置电机的工作模式为VM，进行导纳运动
    for(WORD i = 1; i <= JRR_JOINT_NUMBER; ++i)
    {
        // 激活VM
        _motorDevice->getQMotorObject()->ePOS_ActiveVelocityMode(i);
    }

    // 置标志变量为True
    _isTraingFlag = TRUE;
}
