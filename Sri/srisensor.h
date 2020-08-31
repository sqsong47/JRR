#ifndef SRISENSOR_H
#define SRISENSOR_H

#include <QObject>
#include <Eigen/Eigen/Eigen>
#include <string.h>
#include <QtNetwork>
#include <QTcpSocket>
#include <array>
#include <QTimer>


enum SriSensorCommand{
    InitSystem,         // 刚刚初始化之后的状态
    SampleRate,         // 设置采样率
    ChannelConfig,      // 配置参数
    AmpZero,            // 参数1
    ChnGain,            // 参数2
    ChnEx,              // 参数3
    GOD,                // 单次数据采集
    GSD };              // 连续数据采集


#define RX_BUFFER_SIZE	16384
#define PROCESS_BUFFER_SIZE	16384		// 处理缓冲区的大小
#define M812X_CHN_NUMBER	6			// 通道数
#define RECORD_BUFFER_SIZE	5000000		// 记录缓冲区的大小


class SriSensor : public QObject
{
    Q_OBJECT
public:
    explicit SriSensor(QObject *parent = nullptr);

    void sendCommand(const std::string msg);

    // 初始化阶段
    void getChannelPara(Eigen::Matrix<double,M812X_CHN_NUMBER,1> &m_dMatrix);
    void configSystem();
    void setSamplingRate(const int rate=1000);

    // GOD模式
    void getADCounts();
    void dataProcess();
    void startGodMode();

    // GSD模式
    void startGsdMode();
    int realTimeProcess();
    void stopRealTimeSample();


    // 公有访问接口
    Eigen::Matrix<double,M812X_CHN_NUMBER,1> readForceData();

    // 做初始值的滤波,代码待添加


    // 做测试用
    Eigen::Matrix<double,M812X_CHN_NUMBER,1> getAmpZero();
    Eigen::Matrix<double,M812X_CHN_NUMBER,1> getChnGain();
    Eigen::Matrix<double,M812X_CHN_NUMBER,1> getChnEx();

private:
    Eigen::Matrix<double,M812X_CHN_NUMBER,1> m_dResultChValue;
    Eigen::Matrix<double,M812X_CHN_NUMBER,1> m_nADCounts;
    Eigen::Matrix<double,M812X_CHN_NUMBER,1> m_dAmpZero;
    Eigen::Matrix<double,M812X_CHN_NUMBER,1> m_dChnGain;
    Eigen::Matrix<double,M812X_CHN_NUMBER,1> m_dChnEx;
    Eigen::Matrix<double,M812X_CHN_NUMBER,M812X_CHN_NUMBER> m_dDecouplingCoefficient;

    // 最终计算结果
    Eigen::Matrix<double,M812X_CHN_NUMBER,1> m_dDecouplingValue;

    // 力的基线
    // Eigen::Matrix<double,M812X_CHN_NUMBER,1> m_dBaseForce;


    unsigned char mRxBuffer[RX_BUFFER_SIZE];
    unsigned char* pRxBuffer;
    unsigned char* m_pDealDataBuffer;
    unsigned long mRxCounter;

    unsigned short* m_pRecordDataBuffer[M812X_CHN_NUMBER];
    unsigned long m_dwRecordCounter;

    unsigned long Data_Ptr_Out;
    unsigned long m_dwPackageCouter;
    unsigned long m_dwDealDataIndex;

    std::string msg;
    QTcpSocket* serverSocket;

    SriSensorCommand state;

    QTimer* timer;

    bool m_bIsRealTimeFlag = false;

signals:
    void configSystemFinished();

public slots:
    void onReceiverData();
    void writePatameter();
};




// -------------------------------------------------------------
class SriDevice : public QObject
{
    Q_OBJECT
public:
    explicit SriDevice(QObject *parent = nullptr);

    SriSensor* getObject();
    void ConfigSystem();
    void startRealTimeSample();
    void stopRealTimeSample();
    void singleSample();
    Eigen::Matrix<double,M812X_CHN_NUMBER,1> readForceData();

private:
    SriSensor* _sensor;
};


#endif // SRISENSOR_H
