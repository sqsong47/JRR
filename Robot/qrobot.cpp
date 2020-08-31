#include "qrobot.h"
#include <QtMath>
#include <iostream>
#include <QDebug>
#include <qmath.h>

#define DEBUG

static const double PI = 3.141592653898;

/*********************************************************************
 * 2020.06.11   宋世奇
 * 1. 如果以速度模式进行导纳控制，根本不需要进行位置零点记录，但是由于要切换电机的
 *    运动模式，实际效果并不理想
 * 2. 采用位置模式进行导纳控制一定要注意安全，因为位置算错的话，电机会非常危险
 * 3. 位置初始化的目的在于确定关节转角，用于雅可比矩阵的运算
 * 4. 机器人的控制周期 TS 在运行之前一定要仔细检查，太小的话CAN通信的通信速率达
 *    不到，太大的话误差就会很大，因为加速度是采用的速度差分计算的。
*********************************************************************/

QRobot::QRobot(QObject *parent) : QObject(parent)
{
    _robotState = JustStarted;

    _forceX = 0;    _forceY = 0;    _forceZ = 0;
    _torX = 0;      _torY = 0;      _torZ = 0;

    _c1 = 0; _c2 = 0; _c3 = 0; _c4 = 0; _c5 = 0; _c6 = 0;
    _s1 = 0; _s2 = 0; _s3 = 0; _s4 = 0; _s5 = 0; _s6 = 0;

    _velocity           = Eigen::Array<double, 6, 1>::Zero();
    _lastVelocity       = Eigen::Array<double, 6, 1>::Zero();
    _forceInCartesian   = Eigen::Array<double, 6, 1>::Zero();
    _jacob              = Eigen::Matrix<double, 6, 6>::Zero();
    _invJacob           = Eigen::Matrix<double, 6, 6>::Zero();
    _d_qd_j             = Eigen::Array<double, 6, 1>::Zero();

    // 训练时候用于计算jerk的变量，初始化为0
    _rlJointVelocity    = Eigen::Array<double, 6, 1>::Zero();
    _rlCarVel_t         = Eigen::Array<double, 6, 1>::Zero();
    _rlCarVel_t_1       = Eigen::Array<double, 6, 1>::Zero();
    _rlAcce_t           = Eigen::Array<double, 6, 1>::Zero();
    _rlAcce_t_1         = Eigen::Array<double, 6, 1>::Zero();
    _jerk               = Eigen::Array<double, 6, 1>::Zero();


    // 以下代码为机器人的归零程序使用的代码
    _q_z << 220.316, 90.342, 269.614, 327.452, 303.712, 346.535;

    _q_e = Eigen::Array<double, 6, 1>::Zero();
    _q_j = Eigen::Array<double, 6, 1>::Zero();

    _offsetInit << 0, PI/2, PI/2, PI/2, 0, PI;

    memset(_d_qd_m, 0, sizeof (_d_qd_m));
    memset(_rawForce, 0, sizeof (_rawForce));
    memset(_forceBias, 0, sizeof (_forceBias));


    // 以下为梁家伟使用的原始数据
    //    M << 10, 10, 10, 1000, 1000, 1000;          // 初始化质量， 旋转轴非常大，暂定
    //    D << 100.0, 100.0, 100.0, 5.0, 5.0, 5.0;    // 初始化阻尼， 正数,旋转轴非常大，暂定

    //    M << 5, 5, 5, 250, 250, 250;                        // 初始化质量， 旋转轴非常大，暂定
    //    D << 25.0, 25.0, 50.0, 2.5, 2.5, 2.5;               // 初始化阻尼， 正数,旋转轴非常大，暂定

    _virtualMass << 5, 5, 5, 5, 5, 5;                        // 初始化质量， 旋转轴非常大，暂定
    _virtualDamp << 25.0, 25.0, 50.0, 5, 5, 5;               // 初始化阻尼， 正数,旋转轴非常大，暂定
}



// ------------------------------------------------------------------------------------------------------
// Just do sum calculation, sampling operations are performed in the mainwindow
// Calculate the admittance output speed under the current external force and convert it to joint space

bool QRobot::updateAdmMotionPara()  // 主要更新的还是下个时间片的速度
{
    // 计算所用的外力 = 采样值 - 零点力偏移
    _forceX = _rawForce[0] - _forceBias[0];
    _forceY = _rawForce[1] - _forceBias[1];
    _forceZ = _rawForce[2] - _forceBias[2];
    _torX = _rawForce[3] - _forceBias[3];
    _torY = _rawForce[4] - _forceBias[4];
    _torZ = _rawForce[5] - _forceBias[5];


#ifndef DEBUG
    qDebug() << tr("参与运算的力和力矩值为： ");
    qDebug() << _forceInDireX << " " << _forceInDireY << " " << _forceInDireZ << " "
             << _torAroundX << "" << _torAroundY << " " << _torAroundZ;
#endif

    // 实际参与运算的角度应该是关节角加上初始偏置角，由DH坐标系确定
    // _q_j是角度值，应该转成弧度制参与运算
    _theta1 = qDegreesToRadians(_q_j[0]) + _offsetInit[0];
    _theta2 = qDegreesToRadians(_q_j[1]) + _offsetInit[1];
    _theta3 = qDegreesToRadians(_q_j[2]) + _offsetInit[2];
    _theta4 = qDegreesToRadians(_q_j[3]) + _offsetInit[3];
    _theta5 = qDegreesToRadians(_q_j[4]) + _offsetInit[4];
    _theta6 = qDegreesToRadians(_q_j[5]) + _offsetInit[5];


#ifndef DEBUG
    qDebug() << tr("theta 1-6 is: ");
    qDebug() << _theta1 << " " << _theta2 << " " << _theta3 << " " <<
                _theta4 << " " << _theta5 << " " << _theta6;
#endif


    // 计算当前转角下的cos和sin,数学计算只进行一次,这样可以提高运算速度
    _s1 = qSin(_theta1);   _c1 = qCos(_theta1);
    _s2 = qSin(_theta2);   _c2 = qCos(_theta2);
    _s3 = qSin(_theta3);   _c3 = qCos(_theta3);
    _s4 = qSin(_theta4);   _c4 = qCos(_theta4);
    _s5 = qSin(_theta5);   _c5 = qCos(_theta5);
    _s6 = qSin(_theta6);   _c6 = qCos(_theta6);


    // 计算jacobian
    // 使用的其实是Jacob0;
    _jacob << (429*_s1*_s2)/1000 - (117*_c4*(_c1*_s3 + _c2*_c3*_s1))/1000 + (697*_s4*(_c1*_s3 + _c2*_c3*_s1))/1250 + (697*_c4*_s1*_s2)/1250 + (117*_s1*_s2*_s4)/1000,
            -_c1*(_c2*((697*_c4)/1250 + (117*_s4)/1000 + 429/1000.0) + _c3*_s2*((117*_c4)/1000 - (697*_s4)/1250)),
            -((_c3*_s1 + _c1*_c2*_s3)*(585*_c4 - 2788*_s4))/5000,
            - (_c1*_c3 - _c2*_s1*_s3)*(_c2*((697*_c4)/1250 + (117*_s4)/1000) + _c3*_s2*((117*_c4)/1000 - (697*_s4)/1250)) - _s2*_s3*((_c1*_s3 + _c2*_c3*_s1)*((117*_c4)/1000 - (697*_s4)/1250) - _s1*_s2*((697*_c4)/1250 + (117*_s4)/1000)),
            0,
            0,
            (697*_s4*(_s1*_s3 - _c1*_c2*_c3))/1250 - (117*_c4*(_s1*_s3 - _c1*_c2*_c3))/1000 - (429*_c1*_s2)/1000 - (697*_c1*_c4*_s2)/1250 - (117*_c1*_s2*_s4)/1000,
            -_s1*(_c2*((697*_c4)/1250 + (117*_s4)/1000 + 429/1000.0) + _c3*_s2*((117*_c4)/1000 - (697*_s4)/1250)),
            ((_c1*_c3 - _c2*_s1*_s3)*(585*_c4 - 2788*_s4))/5000,
            - (_c3*_s1 + _c1*_c2*_s3)*(_c2*((697*_c4)/1250 + (117*_s4)/1000) + _c3*_s2*((117*_c4)/1000 - (697*_s4)/1250)) - _s2*_s3*((_s1*_s3 - _c1*_c2*_c3)*((117*_c4)/1000 - (697*_s4)/1250) + _c1*_s2*((697*_c4)/1250 + (117*_s4)/1000)),
            0,
            0,
            0,
            (117*_c2*_c3*_c4)/1000 - (697*_c4*_s2)/1250 - (117*_s2*_s4)/1000 - (429*_s2)/1000 - (697*_c2*_c3*_s4)/1250,
            -(_s2*_s3*(585*_c4 - 2788*_s4))/5000,
            (117*_c2*_c4)/1000 - (697*_c2*_s4)/1250 - (697*_c3*_c4*_s2)/1250 - (117*_c3*_s2*_s4)/1000,
            0,
            0,
            0,
            _s1,
            -_c1*_s2,
            _c3*_s1 + _c1*_c2*_s3,
            _s4*(_s1*_s3 - _c1*_c2*_c3) - _c1*_c4*_s2,
            _c5*(_c3*_s1 + _c1*_c2*_s3) - _s5*(_c4*(_s1*_s3 - _c1*_c2*_c3) + _c1*_s2*_s4),
            0,
            -_c1,
            -_s1*_s2,
            _c2*_s1*_s3 - _c1*_c3,
            - _s4*(_c1*_s3 + _c2*_c3*_s1) - _c4*_s1*_s2,
            _s5*(_c4*(_c1*_s3 + _c2*_c3*_s1) - _s1*_s2*_s4) - _c5*(_c1*_c3 - _c2*_s1*_s3),
            1,
            0,
            _c2,
            _s2*_s3,
            _c2*_c4 - _c3*_s2*_s4,
            _s5*(_c2*_s4 + _c3*_c4*_s2) + _c5*_s2*_s3;


    // 雅克比矩阵求逆，用作奇异性检查
    _invJacob = _jacob.inverse();

    if (_invJacob.norm() > 200)
    {
        return false;
    }

    // 这里导纳运算的质点在frame5和frame6 z轴的交点处。
    // 使用jacob0时用的是下面的代码
    // 对比一下转换之后的差别
    _forceInCartesian << _forceX*(_s6*(_s4*(_s1*_s3 - _c1*_c2*_c3) - _c1*_c4*_s2) - _c6*(_c5*(_c4*(_s1*_s3 - _c1*_c2*_c3) + _c1*_s2*_s4) + _s5*(_c3*_s1 + _c1*_c2*_s3)))
                         + _forceY*(_c6*(_s4*(_s1*_s3 - _c1*_c2*_c3) - _c1*_c4*_s2) + _s6*(_c5*(_c4*(_s1*_s3 - _c1*_c2*_c3) + _c1*_s2*_s4) + _s5*(_c3*_s1 + _c1*_c2*_s3)))
                         - _forceZ*(_s5*(_c4*(_s1*_s3 - _c1*_c2*_c3) + _c1*_s2*_s4) - _c5*(_c3*_s1 + _c1*_c2*_s3)),

            _forceZ*(_s5*(_c4*(_c1*_s3 + _c2*_c3*_s1) - _s1*_s2*_s4) - _c5*(_c1*_c3 - _c2*_s1*_s3))
            - _forceX*(_s6*(_s4*(_c1*_s3 + _c2*_c3*_s1) + _c4*_s1*_s2) - _c6*(_c5*(_c4*(_c1*_s3 + _c2*_c3*_s1) - _s1*_s2*_s4) + _s5*(_c1*_c3 - _c2*_s1*_s3)))
            - _forceY*(_c6*(_s4*(_c1*_s3 + _c2*_c3*_s1) + _c4*_s1*_s2) + _s6*(_c5*(_c4*(_c1*_s3 + _c2*_c3*_s1) - _s1*_s2*_s4) + _s5*(_c1*_c3 - _c2*_s1*_s3))),

            _forceX*(_c6*(_c5*(_c2*_s4 + _c3*_c4*_s2) - _s2*_s3*_s5) + _s6*(_c2*_c4 - _c3*_s2*_s4))
            - _forceY*(_s6*(_c5*(_c2*_s4 + _c3*_c4*_s2) - _s2*_s3*_s5) - _c6*(_c2*_c4 - _c3*_s2*_s4)) + _forceZ*(_s5*(_c2*_s4 + _c3*_c4*_s2) + _c5*_s2*_s3),

            _torX*(_s6*(_s4*(_s1*_s3 - _c1*_c2*_c3) - _c1*_c4*_s2) - _c6*(_c5*(_c4*(_s1*_s3 - _c1*_c2*_c3) + _c1*_s2*_s4) + _s5*(_c3*_s1 + _c1*_c2*_s3)))
            + _torY*(_c6*(_s4*(_s1*_s3 - _c1*_c2*_c3) - _c1*_c4*_s2) + _s6*(_c5*(_c4*(_s1*_s3 - _c1*_c2*_c3) + _c1*_s2*_s4) + _s5*(_c3*_s1 + _c1*_c2*_s3)))
            - _torZ*(_s5*(_c4*(_s1*_s3 - _c1*_c2*_c3) + _c1*_s2*_s4) - _c5*(_c3*_s1 + _c1*_c2*_s3)),

            _torZ*(_s5*(_c4*(_c1*_s3 + _c2*_c3*_s1) - _s1*_s2*_s4) - _c5*(_c1*_c3 - _c2*_s1*_s3))
            - _torX*(_s6*(_s4*(_c1*_s3 + _c2*_c3*_s1) + _c4*_s1*_s2) - _c6*(_c5*(_c4*(_c1*_s3 + _c2*_c3*_s1) - _s1*_s2*_s4) + _s5*(_c1*_c3 - _c2*_s1*_s3)))
            - _torY*(_c6*(_s4*(_c1*_s3 + _c2*_c3*_s1) + _c4*_s1*_s2) + _s6*(_c5*(_c4*(_c1*_s3 + _c2*_c3*_s1) - _s1*_s2*_s4) + _s5*(_c1*_c3 - _c2*_s1*_s3))),

            _torX*(_c6*(_c5*(_c2*_s4 + _c3*_c4*_s2) - _s2*_s3*_s5) + _s6*(_c2*_c4 - _c3*_s2*_s4))
            - _torY*(_s6*(_c5*(_c2*_s4 + _c3*_c4*_s2) - _s2*_s3*_s5) - _c6*(_c2*_c4 - _c3*_s2*_s4)) + _torZ*(_s5*(_c2*_s4 + _c3*_c4*_s2) + _c5*_s2*_s3);

    // 此行代码会通过修改F矩阵，隐藏导纳运动三维的旋转信息
    // 但是如果一开始就设计只做平移的运动，还是直接设置上面的_ForceInCartesian后三个数为0比较好，因为少了三维的计算
    // 不过即使做了计算应该也耗时不多，计算机只做加减乘除是很快的
    dimMask(_forceInCartesian, 3);

    // 迭代计算输出导纳方程输出的速度
    _velocity = (TS * 0.001 * _forceInCartesian + _virtualMass * _lastVelocity) / (_virtualMass + _virtualDamp * TS * 0.001);
    _lastVelocity = _velocity;

    // 反算期望的关节速度，rad/s
    _d_qd_j = _invJacob * (_velocity.matrix());

    for(int i = 0; i < JRR_JOINT_NUMBER; i++)
    {
        // 反算关节电机的速度，单位是rpm  经实验验证是对的
        _d_qd_m[i] = static_cast<long>(_d_qd_j(i,0) * 30 * transRatio[i] * directions[i] / PI);
    }
    return true;
}


// 利用C++的函数重载功能，带导纳参数的代表用于强化学习的训练阶段
bool QRobot::updateAdmMotionPara(double virtualDamp)
{
    /*
 * 函数应该包含以下几个功能
 * 1. 能够根据导纳值进行导纳速度的输出
*/

    // 1. 设置导纳参数，先预设各个方向的阻尼值都是一样的，就是传入的阻抗参数
    Eigen::Array<double, 6, 1> dampArr = virtualDamp * Eigen::Array<double, 6, 1>::Ones();

    // 计算所用的外力 = 采样值 - 零点力偏移
    _forceX = _rawForce[0] - _forceBias[0];
    _forceY = _rawForce[1] - _forceBias[1];
    _forceZ = _rawForce[2] - _forceBias[2];
    _torX = _rawForce[3] - _forceBias[3];
    _torY = _rawForce[4] - _forceBias[4];
    _torZ = _rawForce[5] - _forceBias[5];

    // 实际参与运算的角度应该是关节角加上初始偏置角，由DH坐标系确定
    _theta1 = qDegreesToRadians(_q_j[0]) + _offsetInit[0];
    _theta2 = qDegreesToRadians(_q_j[1]) + _offsetInit[1];
    _theta3 = qDegreesToRadians(_q_j[2]) + _offsetInit[2];
    _theta4 = qDegreesToRadians(_q_j[3]) + _offsetInit[3];
    _theta5 = qDegreesToRadians(_q_j[4]) + _offsetInit[4];
    _theta6 = qDegreesToRadians(_q_j[5]) + _offsetInit[5];

    // 计算当前转角下的cos和sin,数学计算只进行一次,这样可以提高运算速度
    _s1 = qSin(_theta1);   _c1 = qCos(_theta1);
    _s2 = qSin(_theta2);   _c2 = qCos(_theta2);
    _s3 = qSin(_theta3);   _c3 = qCos(_theta3);
    _s4 = qSin(_theta4);   _c4 = qCos(_theta4);
    _s5 = qSin(_theta5);   _c5 = qCos(_theta5);
    _s6 = qSin(_theta6);   _c6 = qCos(_theta6);


    // 计算jacobian
    // 使用的其实是Jacob0;
    _jacob << (429*_s1*_s2)/1000 - (117*_c4*(_c1*_s3 + _c2*_c3*_s1))/1000 + (697*_s4*(_c1*_s3 + _c2*_c3*_s1))/1250 + (697*_c4*_s1*_s2)/1250 + (117*_s1*_s2*_s4)/1000,
            -_c1*(_c2*((697*_c4)/1250 + (117*_s4)/1000 + 429/1000.0) + _c3*_s2*((117*_c4)/1000 - (697*_s4)/1250)),
            -((_c3*_s1 + _c1*_c2*_s3)*(585*_c4 - 2788*_s4))/5000,
            - (_c1*_c3 - _c2*_s1*_s3)*(_c2*((697*_c4)/1250 + (117*_s4)/1000) + _c3*_s2*((117*_c4)/1000 - (697*_s4)/1250)) - _s2*_s3*((_c1*_s3 + _c2*_c3*_s1)*((117*_c4)/1000 - (697*_s4)/1250) - _s1*_s2*((697*_c4)/1250 + (117*_s4)/1000)),
            0,
            0,
            (697*_s4*(_s1*_s3 - _c1*_c2*_c3))/1250 - (117*_c4*(_s1*_s3 - _c1*_c2*_c3))/1000 - (429*_c1*_s2)/1000 - (697*_c1*_c4*_s2)/1250 - (117*_c1*_s2*_s4)/1000,
            -_s1*(_c2*((697*_c4)/1250 + (117*_s4)/1000 + 429/1000.0) + _c3*_s2*((117*_c4)/1000 - (697*_s4)/1250)),
            ((_c1*_c3 - _c2*_s1*_s3)*(585*_c4 - 2788*_s4))/5000,
            - (_c3*_s1 + _c1*_c2*_s3)*(_c2*((697*_c4)/1250 + (117*_s4)/1000) + _c3*_s2*((117*_c4)/1000 - (697*_s4)/1250)) - _s2*_s3*((_s1*_s3 - _c1*_c2*_c3)*((117*_c4)/1000 - (697*_s4)/1250) + _c1*_s2*((697*_c4)/1250 + (117*_s4)/1000)),
            0,
            0,
            0,
            (117*_c2*_c3*_c4)/1000 - (697*_c4*_s2)/1250 - (117*_s2*_s4)/1000 - (429*_s2)/1000 - (697*_c2*_c3*_s4)/1250,
            -(_s2*_s3*(585*_c4 - 2788*_s4))/5000,
            (117*_c2*_c4)/1000 - (697*_c2*_s4)/1250 - (697*_c3*_c4*_s2)/1250 - (117*_c3*_s2*_s4)/1000,
            0,
            0,
            0,
            _s1,
            -_c1*_s2,
            _c3*_s1 + _c1*_c2*_s3,
            _s4*(_s1*_s3 - _c1*_c2*_c3) - _c1*_c4*_s2,
            _c5*(_c3*_s1 + _c1*_c2*_s3) - _s5*(_c4*(_s1*_s3 - _c1*_c2*_c3) + _c1*_s2*_s4),
            0,
            -_c1,
            -_s1*_s2,
            _c2*_s1*_s3 - _c1*_c3,
            - _s4*(_c1*_s3 + _c2*_c3*_s1) - _c4*_s1*_s2,
            _s5*(_c4*(_c1*_s3 + _c2*_c3*_s1) - _s1*_s2*_s4) - _c5*(_c1*_c3 - _c2*_s1*_s3),
            1,
            0,
            _c2,
            _s2*_s3,
            _c2*_c4 - _c3*_s2*_s4,
            _s5*(_c2*_s4 + _c3*_c4*_s2) + _c5*_s2*_s3;

    // 以下代码分别为计算观测值以及更新观测值
    solveEnv();
    updateState();

    // 雅克比矩阵求逆，用作奇异性检查
    _invJacob = _jacob.inverse();
    if (_invJacob.norm() > 200)
    {
        qDebug() << "Jacobian matrix is singular";
        return false;
    }


    // 这里导纳运算的质点在frame5和frame6 z轴的交点处。
    // 使用jacob0时用的是下面的代码
    // 对比一下转换之后的差别
    /*
    _forceInCartesian << _forceX*(_s6*(_s4*(_s1*_s3 - _c1*_c2*_c3) - _c1*_c4*_s2) - _c6*(_c5*(_c4*(_s1*_s3 - _c1*_c2*_c3) + _c1*_s2*_s4) + _s5*(_c3*_s1 + _c1*_c2*_s3)))
                         + _forceY*(_c6*(_s4*(_s1*_s3 - _c1*_c2*_c3) - _c1*_c4*_s2) + _s6*(_c5*(_c4*(_s1*_s3 - _c1*_c2*_c3) + _c1*_s2*_s4) + _s5*(_c3*_s1 + _c1*_c2*_s3)))
                         - _forceZ*(_s5*(_c4*(_s1*_s3 - _c1*_c2*_c3) + _c1*_s2*_s4) - _c5*(_c3*_s1 + _c1*_c2*_s3)),

            _forceZ*(_s5*(_c4*(_c1*_s3 + _c2*_c3*_s1) - _s1*_s2*_s4) - _c5*(_c1*_c3 - _c2*_s1*_s3))
            - _forceX*(_s6*(_s4*(_c1*_s3 + _c2*_c3*_s1) + _c4*_s1*_s2) - _c6*(_c5*(_c4*(_c1*_s3 + _c2*_c3*_s1) - _s1*_s2*_s4) + _s5*(_c1*_c3 - _c2*_s1*_s3)))
            - _forceY*(_c6*(_s4*(_c1*_s3 + _c2*_c3*_s1) + _c4*_s1*_s2) + _s6*(_c5*(_c4*(_c1*_s3 + _c2*_c3*_s1) - _s1*_s2*_s4) + _s5*(_c1*_c3 - _c2*_s1*_s3))),

            _forceX*(_c6*(_c5*(_c2*_s4 + _c3*_c4*_s2) - _s2*_s3*_s5) + _s6*(_c2*_c4 - _c3*_s2*_s4))
            - _forceY*(_s6*(_c5*(_c2*_s4 + _c3*_c4*_s2) - _s2*_s3*_s5) - _c6*(_c2*_c4 - _c3*_s2*_s4)) + _forceZ*(_s5*(_c2*_s4 + _c3*_c4*_s2) + _c5*_s2*_s3),

            _torX*(_s6*(_s4*(_s1*_s3 - _c1*_c2*_c3) - _c1*_c4*_s2) - _c6*(_c5*(_c4*(_s1*_s3 - _c1*_c2*_c3) + _c1*_s2*_s4) + _s5*(_c3*_s1 + _c1*_c2*_s3)))
            + _torY*(_c6*(_s4*(_s1*_s3 - _c1*_c2*_c3) - _c1*_c4*_s2) + _s6*(_c5*(_c4*(_s1*_s3 - _c1*_c2*_c3) + _c1*_s2*_s4) + _s5*(_c3*_s1 + _c1*_c2*_s3)))
            - _torZ*(_s5*(_c4*(_s1*_s3 - _c1*_c2*_c3) + _c1*_s2*_s4) - _c5*(_c3*_s1 + _c1*_c2*_s3)),

            _torZ*(_s5*(_c4*(_c1*_s3 + _c2*_c3*_s1) - _s1*_s2*_s4) - _c5*(_c1*_c3 - _c2*_s1*_s3))
            - _torX*(_s6*(_s4*(_c1*_s3 + _c2*_c3*_s1) + _c4*_s1*_s2) - _c6*(_c5*(_c4*(_c1*_s3 + _c2*_c3*_s1) - _s1*_s2*_s4) + _s5*(_c1*_c3 - _c2*_s1*_s3)))
            - _torY*(_c6*(_s4*(_c1*_s3 + _c2*_c3*_s1) + _c4*_s1*_s2) + _s6*(_c5*(_c4*(_c1*_s3 + _c2*_c3*_s1) - _s1*_s2*_s4) + _s5*(_c1*_c3 - _c2*_s1*_s3))),

            _torX*(_c6*(_c5*(_c2*_s4 + _c3*_c4*_s2) - _s2*_s3*_s5) + _s6*(_c2*_c4 - _c3*_s2*_s4))
            - _torY*(_s6*(_c5*(_c2*_s4 + _c3*_c4*_s2) - _s2*_s3*_s5) - _c6*(_c2*_c4 - _c3*_s2*_s4)) + _torZ*(_s5*(_c2*_s4 + _c3*_c4*_s2) + _c5*_s2*_s3);
    */

    // 只做x轴的平移运动
    // dimMask(_forceInCartesian, 1);

    // 训练过程需要将其置为5
    //    _forceInCartesian[0] = 5;

    _forceInCartesian << 5, 0, 0, 0, 0, 0;

    // 根据传入的阻抗进行导纳速度的更新
    _velocity = (TS * 0.001 * _forceInCartesian + _virtualMass * _lastVelocity) / (_virtualMass + dampArr * TS * 0.001);
    _lastVelocity = _velocity;

    // 反算期望的关节速度，rad/s
    _d_qd_j = _invJacob * (_velocity.matrix());

    for(int i = 0; i < JRR_JOINT_NUMBER; i++)
    {
        // 反算关节电机的速度，单位是rpm  经实验验证是对的
        _d_qd_m[i] = static_cast<long>(_d_qd_j(i,0) * 30 * transRatio[i] * directions[i] / PI);
    }

    // 至此已经计算出来电机应该执行的速度
    return true;
}

Eigen::Array<long, 6, 1> QRobot::calMotorStroke()
{
    // 计算电机需要转动的角度,以及需要运动的行程
    Eigen::Array<double, 6, 1> q_m;
    for (int i = 0; i < 6; i++)
    {
        // 电机应该反向运动，使得各个关节角归零
        q_m[i] = -1 * _q_j[i] * transRatio[i] * directions[i];

        // 计算电机的行程
        _p_m[i] = static_cast<long>(q_m[i] / 360 * countsPerTurn[i]);
    }
    return _p_m;
}

// --------------------------------------------------------------------------------------------------------

// 初始化F_bias
void QRobot::setForceBias(const Eigen::Matrix<double, SRI_CHANNEL_NUMBER, 1> &forceTmp, const int size)
{
    for(int i = 0; i < size; ++i)
    {
        _forceBias[i] = forceTmp[i];
    }

}

double *QRobot::getForceBias()
{
    return _forceBias;
}

double *QRobot::getForceInSensor()
{
    return _rawForce;
}

long *QRobot::getAdmMotorVel()
{
    return _d_qd_m;
}

double QRobot::getForceIn_X_Direction()
{
    return _forceX;
}

double QRobot::getForceIn_Y_Direction()
{
    return _forceY;
}

double QRobot::getForceIn_Z_Direction()
{
    return _forceZ;
}

double QRobot::getTorque_X_Direction()
{
    return _torX;
}

double QRobot::getTorque_Y_Direction()
{
    return _torY;
}

double QRobot::getTorque_Z_Direction()
{
    return _torZ;
}

Eigen::Array<double, JRR_JOINT_NUMBER, 1> QRobot::getCurrentJointVelocity()
{
    return _d_qd_j;
}

Eigen::Array<double, 6, 1> QRobot::getVirtualMassVector()
{
    return _virtualMass;
}

Eigen::Array<double, 6, 1> QRobot::getVirtualDampVector()
{
    return _virtualDamp;
}

Eigen::Array<double, JRR_JOINT_NUMBER, 1> QRobot::getJointAngle()
{
    return _q_j;

}

void QRobot::resetJerk()
{
    _jerk = 0;
}

Eigen::Array<double, 6, 1> QRobot::getJointVelocity()
{
    return _rlJointVelocity;
}

double QRobot::getReward()
{
    // 返回jerk的绝对值作为奖励
    return qFabs( _jerk[0]);
}

double QRobot::getAccel()
{
    return _rlAcce_t[0];
}

double QRobot::getVelocity()
{
    return _rlCarVel_t[0];
}



void QRobot::dimMask(Eigen::Array<double, 6, 1> array, int dimension)
{
    if(dimension > 6 || dimension < 0)
    {
        qDebug() << "Please input correct dimension";
        return;
    }

    for (int i = 6; i > dimension; )
    {
        array[i--] = 0;
    }
}

void QRobot::setHighAdmittancePara()
{
    _virtualMass << 10, 10, 10, 10, 10, 10;
    _virtualDamp << 50, 50, 100, 5, 5, 5;
}

void QRobot::setLowAdmittancePara()
{
    _virtualMass << 5, 5, 5, 5, 5, 5;
    _virtualDamp << 25, 25, 50, 5, 5, 5;
}


void QRobot::setCurrentForceValue(const Eigen::Matrix<double, SRI_CHANNEL_NUMBER, 1> &forceTmp, const int size)
{
    for(int i = 0; i < size; ++i)
    {
        _rawForce[i] = forceTmp[i];
    }
}

void QRobot::setVirtualMassValue(const Eigen::Array<double, 6, 1> massTmp)
{
    _virtualMass = massTmp;
}

void QRobot::setVirtualDampValue(const Eigen::Array<double, 6, 1> dampTmp)
{
    _virtualDamp = dampTmp;
}

void QRobot::setCurrentEncoderAngle(
        const double& theta_1, const double& theta_2, const double& theta_3,
        const double& theta_4, const double& theta_5, const double& theta_6)
{
    // _q_e保存了编码器的读数值
    _q_e << theta_1,theta_2,theta_3,theta_4,theta_5,theta_6;

    // _q_z保存了机器人零位时的编码器读数值，标定修正就是该参数
    // _q_j保存了关节角度值
    _q_j[0] = _q_e[0] - _q_z[0];        // 1关节正常运算
    _q_j[1] = _q_e[1] - _q_z[1];        // 2关节正常运算
    _q_j[2] = -1*(_q_e[2] - _q_z[2]);   // 3关节由于编码器的安装需要乘一个修正系数

    // 4关节由于过了零点，所以需要进行判断
    if(_q_e[3] < 20)  // 其实只能到16左右
    {
        _q_j[3] = 360 - _q_z[3] + _q_e[3];
    }else {
        _q_j[3] = _q_e[3] - _q_z[3];
    }

    _q_j[4] = _q_e[4] - _q_z[4];        // 5关节也是正常运算

    // 由于6关节也是过了零点，所以需要进行判断
    if(_q_e[5] < 25)
    {
        _q_j[5] = 360 - _q_z[5] + _q_e[5];
    }else {
        _q_j[5] = _q_e[5] - _q_z[5];
    }
}

void QRobot::setJointVelocity(long v1, long v2, long v3,
                              long v4, long v5, long v6)
{
    // 电机采集的速度应该进行传动比的转换才能映射到关节速度上
    // rpm转rad/s
    double tmp_v1 = v1 * 2.0 * PI / 60;
    double tmp_v2 = v2 * 2.0 * PI / 60;
    double tmp_v3 = v3 * 2.0 * PI / 60;
    double tmp_v4 = v4 * 2.0 * PI / 60;
    double tmp_v5 = v5 * 2.0 * PI / 60;
    double tmp_v6 = v6 * 2.0 * PI / 60;

    Eigen::Array<double, 6, 1> tmpMotorVel;
    tmpMotorVel << tmp_v1, tmp_v2, tmp_v3, tmp_v4, tmp_v5, tmp_v6;


    // TODO：从电机到关节的速度应该除以传动比，然后再乘以方向修正
    for (int i = 0; i < 6; ++i)
    {
        _rlJointVelocity[i] = tmpMotorVel[i] / transRatio[i] * directions[i];
    }
}

void QRobot::solveEnv()
{
    // 以下代码计算末端笛卡尔速度/加速度/加加速度
    // 注意下面的_velocity和_lastVelocity导纳运动期望输出的速度，和这里真实计算的是有偏差的
    _rlCarVel_t = _jacob * _rlJointVelocity.matrix();
    _rlAcce_t = (_rlCarVel_t - _rlCarVel_t_1) / TS;
    _jerk = (_rlAcce_t - _rlAcce_t_1) / TS;
}

void QRobot::updateState()
{
    // 进行速度的更新
    _rlCarVel_t_1 = _rlCarVel_t;

    // 进行加速度的更新
    _rlAcce_t_1 = _rlAcce_t;
}
