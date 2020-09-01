#ifndef QROBOT_H
#define QROBOT_H

#include <QObject>
#include <Eigen/Eigen/Dense>

/**********************************************************************************
 * 2020.06.12       宋世奇
 * 1. 程序中有很多该加引用和const的地方还没有改完
**********************************************************************************/
// const Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();

// 属于Robot，可以在其他文件中进行引用
const int TS = 30;         // 测试时用100，运行时用30
const unsigned char JRR_JOINT_NUMBER = 6;
const unsigned char SRI_CHANNEL_NUMBER = 6;


typedef enum{
    JustStarted,            // 刚启动，等待connect
    Connected,              // 连接（读取绝对编码器示数）
    ErrorOccured,           // 发生错误
    Initialized,            // 初始化状态（清除错误，去使能）
    Released,               // 打开制动器
    Enabled,                // 使能状态
    Teach,                  // 示教模式
    AdmittanceMode          // 导纳运动模式
}RobotStates;

class QRobot : public QObject
{
    Q_OBJECT
public:
    explicit QRobot(QObject *parent = nullptr);

public:

    // 本来也应该是私有成员
    const double transRatio[JRR_JOINT_NUMBER] = {12.9, 23.6, 11.6, 20.09333333, 3.6, 4.7777777778};        // 电机到关节的传动比
    const int directions[JRR_JOINT_NUMBER] = {-1, 1, 1, -1, -1, -1};                                       // 方向修正，由绕线方向决定
    const int countsPerTurn[JRR_JOINT_NUMBER] = {2000, 2000, 2000, 25600, 20000, 16384};                   // 电机每圈多少脉冲

public:
    // 本来也应该是私有变量，但是为了方便写成了公有的，后期待改进
    RobotStates _robotState;                                // 机器人状态

public:
    bool updateAdmMotionPara();                             // 导纳运动模式
    bool updateAdmMotionPara(double virtualDamp);           // 强化学习使用的导纳代码
    Eigen::Array<long, 6, 1> calMotorStroke();              // 计算机器人的归零行程

public:
    void setForceBias(const Eigen::Matrix<double,SRI_CHANNEL_NUMBER,1>& forceTmp, const int size = 6);                       // 设置力的零点偏移
    void setCurrentForceValue(const Eigen::Matrix<double,SRI_CHANNEL_NUMBER,1>& forceTmp, const int size = 6);               // 输入当前力和力矩
    void setVirtualMassValue(const Eigen::Array<double, 6, 1> massTmp);                                     // 设置虚拟质量
    void setVirtualDampValue(const Eigen::Array<double, 6, 1> dampTmp);                                     // 设置虚拟阻尼
    void setCurrentEncoderAngle(const double& theta_1, const double& theta_2, const double& theta_3,        // 设置关节角参数,也包含了更新机器人关节角的操作
                                const double& theta_4, const double& theta_5, const double& theta_6);
    void setJointVelocity(long v1, long v2, long v3, long v4, long v5, long v6);


    void solveEnv();                            // 求解状态的观测值
    void updateState();                         // 每个时刻读取速度信息后应该更新前两个时刻的速度

    // ------------------------------------------------------------------------------------------------
    double* getForceBias();                                 // 读取零点力偏移
    double* getForceInSensor();                             // 读取当前的力信息
    long* getAdmMotorVel();                                 // 读取导纳运动的电机速度

    // 读取力信息
    double getForceIn_X_Direction();
    double getForceIn_Y_Direction();
    double getForceIn_Z_Direction();
    double getTorque_X_Direction();
    double getTorque_Y_Direction();
    double getTorque_Z_Direction();

    Eigen::Array<double, JRR_JOINT_NUMBER, 1> getCurrentJointVelocity();    // 读取当前计算的关节速度
    Eigen::Array<double, 6, 1> getVirtualMassVector();                      // 读取虚拟质量
    Eigen::Array<double, 6, 1> getVirtualDampVector();                      // 读取虚拟阻尼
    Eigen::Array<double, JRR_JOINT_NUMBER, 1> getJointAngle();              // 读取关节角
    void resetJerk();                                                       // 重置jerk为零

    /*************************************************************************************************/

    Eigen::Array<double, 6, 1> getJointVelocity();                          // 读取实时的关节速度，此函数可能有点多余

    double getReward();                                                     // 返回jerk的第一个元素，作为奖励值
    double getAccel();                                                      // 返回加速度值作为环境状态的观测量
    double getVelocity();                                                   // 返回当前速度值作为环境状态的观测量
    void dimMask(Eigen::Array<double, 6, 1> array, int dimension);          // 隐藏导纳运动多余的空间运动维数


private:
    long _d_qd_m[JRR_JOINT_NUMBER];                         // 经导纳运算后，电机待执行的速度
    double _forceBias[JRR_JOINT_NUMBER];                    // 力和力矩的零点偏移
    double _rawForce[JRR_JOINT_NUMBER];                     // 力传感器的采样值，初始化为0

    double _theta1, _theta2, _theta3,
    _theta4, _theta5, _theta6;                              // 用于计算关节转角，已经加上了关节角偏置

    double _c1, _c2, _c3, _c4, _c5, _c6,
    _s1, _s2, _s3, _s4, _s5, _s6;                           // 为了加快运算速度

    Eigen::Array<double, 6, 1> _virtualMass;                // 导纳质量/惯量
    Eigen::Array<double, 6, 1> _virtualDamp;                // 导纳阻尼
    Eigen::Array<double, 6, 1> _velocity;                   // 当前时刻的速度向量，初始化为0
    Eigen::Array<double, 6, 1> _lastVelocity;               // 上一时刻的速度，用来做差分求解导纳方程
    Eigen::Array<double, 6, 1> _forceInCartesian;           // 虚拟质量块受的力在0系下的表示


    double _forceX, _forceY, _forceZ,
    _torX, _torY, _torZ;                                    // 机器人实际受到的外力

    Eigen::Matrix<double, 6, 6> _jacob;                     // jacobian矩阵，初始化为0
    Eigen::Matrix<double, 6, 6> _invJacob;                  // jacobian的逆矩阵
    Eigen::Array<double, 6, 1> _d_qd_j;                     // 导纳运算输出的关节速度

    // 以下代码与机器人的标定有关
    Eigen::Array<double, 6, 1> _q_z;                        // 机器人处于零位时编码器的读数
    Eigen::Array<double, 6, 1> _q_e;                        // 编码器的实时读数
    Eigen::Array<double, 6, 1> _q_j;                        // 实时的关节转角,如果想要进行雅可比和前向运动学的求解还需要加上偏置
    Eigen::Array<long, 6, 1> _p_m;                          // 电机需要运动的行程
    Eigen::Array<double, 6, 1> _offsetInit;                 // 关节的初始偏置角


    // RL数据
    Eigen::Array<double, 6, 1> _rlJointVelocity;            // 记录电机速度，需要传动比才能映射到关节速度上

    Eigen::Array<double, 6, 1> _rlCarVel_t;                 // 记录末端当前笛卡尔速度
    Eigen::Array<double, 6, 1> _rlCarVel_t_1;               // 记录末端上个时刻的笛卡尔速度
    // Eigen::Array<double, 6, 1> _rlCarVel_t_2;

    Eigen::Array<double, 6, 1> _rlAcce_t;                   // 记录末端当前时刻的加速度值，由速度的一阶向后差分计算得到
    Eigen::Array<double, 6, 1> _rlAcce_t_1;                 // 记录末端上个时刻的加速度值

    Eigen::Array<double, 6, 1> _jerk;                       // 记录奖励值，是加速度的一阶向后差分


signals:

public slots:
    void setHighAdmittancePara();
    void setLowAdmittancePara();
};

#endif // QROBOT_H
