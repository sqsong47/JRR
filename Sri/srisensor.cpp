#include "srisensor.h"
#include <QString>
#include <QByteArray>
#include <cstring>
#include <QVector>
#include <iostream>
#include <QDebug>



/*****************************************************************************
 * 2020.06.10   宋世奇
 * 1. 传感器已经更新为连续数据采集的模式，但是仍然有一些小bug
 * 2. 点击配置之后需要等待1-2s才能做之后的操作，因为要写入参数
 * 3. 配置操作其实包含了设置采样率，但是并没有输出提示信息，可能程序执行太快了
 * 4. TCP的跨线程问题也没有出现，这是个好事
 * 5. 解耦矩阵已经更新为标定报告中的数值了，不需要再进行修改
 * 6. 单次采集操作其实得到的是上一时刻的力和力矩值。
 * 7. 程序中Record矩阵存储了历史数据，但是没有用过
******************************************************************************/


SriSensor::SriSensor(QObject *parent) : QObject(parent)
{
    //----------------------------------Data Init------------------------------------

    m_dDecouplingValue  = Eigen::Matrix<double,M812X_CHN_NUMBER,1>::Zero();
    m_dResultChValue    = Eigen::Matrix<double,M812X_CHN_NUMBER,1>::Zero();
    m_nADCounts         = Eigen::Matrix<double,M812X_CHN_NUMBER,1>::Zero();
    m_dAmpZero          = Eigen::Matrix<double,M812X_CHN_NUMBER,1>::Zero();
    m_dChnGain          = Eigen::Matrix<double,M812X_CHN_NUMBER,1>::Zero();
    m_dChnEx            = Eigen::Matrix<double,M812X_CHN_NUMBER,1>::Zero();
    // m_dBaseForce        = Eigen::Matrix<double,M812X_CHN_NUMBER,1>::Zero();


    // 正确的解耦矩阵
    m_dDecouplingCoefficient << -138.600870, -0.36438000, -1.02861000, -0.61954000, 141.8817000, 1.523230000,
            81.88078000, 1.942600000, -163.773330, -0.35478000, 80.68573000, 0.114680000,
            0.364390000, 88.04022000, 0.215040000, 89.42622000, 1.359540000, 88.40195000,
            -0.11285000, -0.80143000, 0.203010000, -0.03472000, -0.07869000, -0.79145000,
            -0.15607000, 0.579660000, -0.00819000, -0.85580000, 0.204570000, 0.504180000,
            -1.43202000, -0.01931000, -1.30134000, -0.01354000, -1.33984000, -0.02947000;



    memset(mRxBuffer, 0x00, RX_BUFFER_SIZE);
    pRxBuffer = new unsigned char;
    pRxBuffer = nullptr;

    m_pDealDataBuffer = new unsigned char[RX_BUFFER_SIZE];
    memset(m_pDealDataBuffer,0,RX_BUFFER_SIZE);
    m_dwDealDataIndex = 0x00;
    m_dwPackageCouter = 0x00;

    mRxCounter = 0x00;

    state = SriSensorCommand::InitSystem;

    for(int i = 0x00; i < M812X_CHN_NUMBER; ++i)
    {
        m_pRecordDataBuffer[i] = new unsigned short[RECORD_BUFFER_SIZE];
        memset(m_pRecordDataBuffer[i],0,RECORD_BUFFER_SIZE);
    }
    m_dwRecordCounter = 0x00;
    Data_Ptr_Out = 0xFFFFFFFF;

    //----------------------------------Timer---------------------------------------

    timer = new QTimer;
    connect(timer,SIGNAL(timeout()),this,SLOT(writePatameter()));

    //----------------------------------TCP-IP---------------------------------------

    serverSocket = new QTcpSocket;

    QString ip = "192.168.0.108";
    quint16 port = 4008;

    serverSocket->abort();
    serverSocket->connectToHost(ip,port);
    serverSocket->waitForConnected(2000);

    // 接收指令做出回应的响应函数
    connect(serverSocket,SIGNAL(readyRead()),this,SLOT(onReceiverData()));

    // 获取socket的错误信息输出
    connect(serverSocket, QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error),
            [=](QAbstractSocket::SocketError socketError){ qDebug()<<socketError; });

}


void SriSensor::sendCommand(const std::string msg)
{
    // 每次发送指令前都清空mRxBuffer，重置mRxCounter
    memset(mRxBuffer,0,sizeof (mRxBuffer));
    mRxCounter = 0x00;
    serverSocket->write(msg.c_str(),static_cast<qint64>(strlen(msg.c_str())));
}


void SriSensor::getChannelPara(Eigen::Matrix<double, M812X_CHN_NUMBER, 1> &m_dMatrix)
{

    char *pIndexBuffer = nullptr;
    pIndexBuffer = strstr(reinterpret_cast<char*>(mRxBuffer), "=");

    if (pIndexBuffer == nullptr) {
        // 添加错误处理机制
        std::cout<<"There is Error 1"<<std::endl;
        return;
    }

    pIndexBuffer = pIndexBuffer + 1;

    char CharTemp[16] = {0x00};
    int k = 0;
    double dTemp = 0x00;

    for (int i = 0; i < M812X_CHN_NUMBER; ++i)
    {
        memset(CharTemp, 0x00, sizeof(CharTemp));
        k = 0x00;
        while (*pIndexBuffer != ';' && *pIndexBuffer != '\r')
        {
            CharTemp[k++] = *pIndexBuffer++;
        }

        if (sscanf_s(CharTemp, "%lf", &dTemp) != 1) {
            // 添加错误处理机制
            std::cout<<"There is Error 2"<<std::endl;
            return;
        }
        m_dMatrix[i] = dTemp;
        pIndexBuffer++;
    }
}

// 有待改进
void SriSensor::configSystem()
{
    state = SriSensorCommand::InitSystem;
    timer->start(500);
}

void SriSensor::setSamplingRate(const int rate)
{
    state = SriSensorCommand::SampleRate;

    std::string start = "AT+SMPR=";
    std::string end = "\r\n";
    std::string middle = QString::number(rate).toStdString();

    msg = start + middle + end;
    sendCommand(msg);
}


// 有待改进
void SriSensor::getADCounts()
{
    pRxBuffer =reinterpret_cast<unsigned char*>(mRxBuffer);

    if (pRxBuffer[0] == 0xAA && pRxBuffer[1] == 0x55)
    {
        int Index = 6;
        for (unsigned int i = 0; i < 6; ++i)
        {
            m_nADCounts[i] = pRxBuffer[Index] * 256 + pRxBuffer[Index + 1];
            Index = Index + 2;
        }
    }
}

void SriSensor::dataProcess()
{
    for (int i = 0x00; i < M812X_CHN_NUMBER; i++)
    {
        m_dResultChValue[i] = 1000 * ((m_nADCounts[i] - m_dAmpZero[i]) / static_cast<double>(65535)
                                      * static_cast<double>(5)) / m_dChnGain[i] / m_dChnEx[i];
    }
    m_dDecouplingValue = m_dDecouplingCoefficient * m_dResultChValue;
}

void SriSensor::startGodMode()
{
    state = SriSensorCommand::GOD;

    sendCommand("AT+GOD\r\n");
}

void SriSensor::startGsdMode()
{
    state = SriSensorCommand::GSD;
    m_bIsRealTimeFlag = true;

    Data_Ptr_Out = mRxCounter;
    m_dwPackageCouter= 0x00;
    m_dwDealDataIndex = 0x00;

    sendCommand("AT+GSD\r\n");
}

int SriSensor::realTimeProcess()
{

    if(Data_Ptr_Out == 0xFFFFFFFF)
    {
        return 0;
    }

    bool  DataHeaderFlag = false;
    unsigned long i = 0x00,j = 0x00;

    unsigned long PackageLength= 0xFFFFFFFF;
    unsigned long HeaderIndex= 0x00;
    unsigned long HighIndex,LowIndex;
    unsigned long PointCounter = 0x00;
    unsigned long RxLengthTemp = 0x00;

    unsigned long RxCounterCurrent = mRxCounter;

    //------------------
    while(1)
    {

        //--------------------------------------------------------------------------------
        //Data length
        HeaderIndex = Data_Ptr_Out;
        if(RxCounterCurrent >= HeaderIndex)
        {
            RxLengthTemp = RxCounterCurrent - HeaderIndex;
        }else
        {
            RxLengthTemp = RX_BUFFER_SIZE - HeaderIndex + RxCounterCurrent;
        }
        if(RxLengthTemp <= 9)
        {
            break;
        }
        //--------------------------------------------------------------------------------
        //Data head fram
        // 在前10个字节中寻找HeaderIndex
        for(i = 0x00;i < RxLengthTemp; i++)
        {

            if(HeaderIndex == RX_BUFFER_SIZE )
            {
                HeaderIndex = 0x00;
            }
            if(HeaderIndex == RX_BUFFER_SIZE-1)
            {
                if((mRxBuffer[HeaderIndex] == 0xAA) && (mRxBuffer[0] == 0x55))
                {
                    DataHeaderFlag = true;
                    break;
                }
            }else if((mRxBuffer[HeaderIndex] == 0xAA) && (mRxBuffer[HeaderIndex+1] == 0x55))
            {
                DataHeaderFlag = true;
                break;
            }
            // 如果以上都没有找到就HeaderIndex跳转下一字节寻找
            HeaderIndex++;
        }
        if(DataHeaderFlag != true)
        {
            break;
        }
        DataHeaderFlag = true;


        //-------------------------------------------------------------------------------
        // 数据提取和解析

        HighIndex = HeaderIndex+2;
        LowIndex = HeaderIndex+3;
        if(HighIndex >= RX_BUFFER_SIZE)
        {
            HighIndex = HighIndex - RX_BUFFER_SIZE;
        }
        if(LowIndex >= RX_BUFFER_SIZE)
        {
            LowIndex = LowIndex - RX_BUFFER_SIZE;
        }
        PackageLength = mRxBuffer[HighIndex]*256 + mRxBuffer[LowIndex];


        // HeaderIndex调整之后再计算
        if(RxCounterCurrent >= HeaderIndex)//LYY
        {
            RxLengthTemp = RxCounterCurrent - HeaderIndex;
        }else
        {
            RxLengthTemp = RX_BUFFER_SIZE - HeaderIndex + RxCounterCurrent;
        }
        if(RxLengthTemp < PackageLength + 4)	// PackageLength + 4	是一帧数据的长度
        {
            break;
        }


        // 此处还未返回，程序正常执行到这里


        //Now, HeaderIndex point to‘0xAA’
        //Head fram	  PackageLength	 DataNo	       Data	       ChkSum
        //0xAA,0x55	     HB,LB         2B   (ChNum*N*DNpCH) B	 1B
        //--------------------------------------------------------------------------------
        //Save data	in order


        unsigned long MoveIndex = HeaderIndex+6;	// Data的起始地址


        // TODO： 添加SUM校验代码

        /*
        if(MoveIndex >= RX_BUFFER_SIZE)
        {
            MoveIndex = MoveIndex - RX_BUFFER_SIZE;
        }
        unsigned short CheckSum = 0x00;

        for(i = 0x00; i <  PackageLength-3; i++)	// PackageLength-3 是去除编号和sum校验项的长度
        {
            CheckSum += mRxBuffer[MoveIndex];
            MoveIndex++;
            if(MoveIndex == RX_BUFFER_SIZE)
            {
                MoveIndex = 0x00;
            }
        }

        // 程序正常执行到这里，可以得知sum校验出错了

        // 判断校验项是否成功，该帧数据损坏，查找下一帧
        if(CheckSum != mRxBuffer[MoveIndex])
        {
            Data_Ptr_Out = MoveIndex;
            break;
        }
        */


        // 此处程序已经返回，上段程序出错


        //---------------------------------------------------------------------------------
        MoveIndex = HeaderIndex+6;				// 返回到数据开头
        if(MoveIndex >= RX_BUFFER_SIZE)
        {
            MoveIndex = MoveIndex - RX_BUFFER_SIZE;
        }
        if(m_dwDealDataIndex >= PROCESS_BUFFER_SIZE - PackageLength + 3)
        {
            m_dwDealDataIndex = 0x00;
            m_dwPackageCouter = 0x00;
            return -1;
        }


        for(i = 0x00; i < PackageLength-3; i++)
        {
            // 将	mRxBuffer中的data转存到	m_pDealDataBuffer中
            m_pDealDataBuffer[m_dwDealDataIndex++] = mRxBuffer[MoveIndex++];
            if(MoveIndex == RX_BUFFER_SIZE)
            {
                MoveIndex = 0x00;
            }
        }
        Data_Ptr_Out = MoveIndex;			// Data_Ptr_Out移动到下一帧
        m_dwPackageCouter++;				// 记录处理了几个数据帧

        if(m_dwDealDataIndex == 0x00)
        {
            return -1;
        }

        //-------------------------------------------------------------------------------
        //
        int Index = 0x00;
        PointCounter = m_dwDealDataIndex / 2 / M812X_CHN_NUMBER;   // 整数相除，记录处理第几组点
        if( (PointCounter >= PROCESS_BUFFER_SIZE) || (PointCounter <= 0) )
        {
            m_dwDealDataIndex = 0x00;
            m_dwPackageCouter = 0x00;
            return - 1;
        }

        unsigned long DispChSum[M812X_CHN_NUMBER] = {0x00};
        for(j = 0x00; j < PointCounter; j++)
        {

            for(i = 0; i < M812X_CHN_NUMBER; i++)
            {
                // TODO：添加记录历史数据的代码段

                m_pRecordDataBuffer[i][m_dwRecordCounter] = static_cast<unsigned short>
                        (m_pDealDataBuffer[Index]*256 + m_pDealDataBuffer[Index+1]);
                DispChSum[i] = m_pRecordDataBuffer[i][m_dwRecordCounter];
                Index = Index + 2;
            }
            m_dwRecordCounter++;
            if(m_dwRecordCounter >= RECORD_BUFFER_SIZE)
            {
                m_dwRecordCounter = 0x00;
            }
        }

        //---------------------------------------------------
        for(int k = 0x00; k < M812X_CHN_NUMBER;k++)
        {
            m_nADCounts[k]  = DispChSum[k]/PointCounter;
        }

        //show decoupled data
        dataProcess();

        //--------------------------------
        m_dwDealDataIndex = 0x00;
        m_dwPackageCouter = 0x00;
    }

    //------------------------------------
    return 1;

}

void SriSensor::stopRealTimeSample()
{
    state = SriSensorCommand::InitSystem;
    m_bIsRealTimeFlag = false;
    sendCommand("AT+GSD=STOP\r\n");
}

Eigen::Matrix<double, M812X_CHN_NUMBER, 1> SriSensor::readForceData()
{
    return m_dDecouplingValue;

}

Eigen::Matrix<double, M812X_CHN_NUMBER, 1> SriSensor::getAmpZero()
{
    return m_dAmpZero;
}

Eigen::Matrix<double, M812X_CHN_NUMBER, 1> SriSensor::getChnGain()
{
    return m_dChnGain;
}

Eigen::Matrix<double, M812X_CHN_NUMBER, 1> SriSensor::getChnEx()
{
    return m_dChnEx;
}


// --------------------------------------------------------------------------------
void SriSensor::onReceiverData()
{
    // 将数据转存到mRxBuffer中
    QByteArray dataInSocket = serverSocket->readAll();
    if(!dataInSocket.isEmpty())
    {
        //        qDebug()<<"size: "<<dataInSocket.size();
        int dataLen = dataInSocket.size();

        for (int i = 0x00; i < dataLen; ++i) {
            if(mRxCounter >= sizeof (mRxBuffer))
            {
                mRxCounter = 0x00;
            }
            mRxBuffer[mRxCounter++] = static_cast<unsigned char>(dataInSocket.data()[i]);
        }
    }

    // 此处可以优化，不想进入switch-case语句才这么写的
    if(m_bIsRealTimeFlag)
    {
        realTimeProcess();
        return;
    }


    // 根据操作指令执行不同的函数段
    switch (state) {
    case SriSensorCommand::ChannelConfig:
        qDebug()<<dataInSocket.data();
        break;
    case SriSensorCommand::AmpZero:
        getChannelPara(m_dAmpZero);
        break;
    case SriSensorCommand::ChnGain:
        getChannelPara(m_dChnGain);
        break;
    case SriSensorCommand::ChnEx:
        getChannelPara(m_dChnEx);
        break;
    case SriSensorCommand::SampleRate:
        qDebug()<<dataInSocket.data();
        break;
    case SriSensorCommand::GOD:
        getADCounts();
        dataProcess();
        break;
    default:
        break;
    }
}



void SriSensor::writePatameter()
{
    switch (state) {
    case SriSensorCommand::InitSystem:
        sendCommand("AT+SGDM=(A01,A02,A03,A04,A05,A06);C;1;(WMA:1)\r\n");
        state = SriSensorCommand::ChannelConfig;
        break;
    case SriSensorCommand::ChannelConfig:
        sendCommand("AT+AMPZ=?\r\n");
        state = SriSensorCommand::AmpZero;
        break;
    case SriSensorCommand::AmpZero:
        sendCommand("AT+CHNAPG=?\r\n");
        state = SriSensorCommand::ChnGain;
        break;
    case SriSensorCommand::ChnGain:
        sendCommand("AT+EXMV=?\r\n");
        state = SriSensorCommand::ChnEx;
        break;
    case SriSensorCommand::ChnEx:
        timer->stop();
        emit configSystemFinished();
        break;
    default:
        break;
    }
}


//---------------------------------------------------------------
SriDevice::SriDevice(QObject *parent): QObject(parent)
{
    _sensor = new SriSensor;
}

SriSensor *SriDevice::getObject()
{
    return _sensor;
}

void SriDevice::ConfigSystem()
{
    _sensor->setSamplingRate(500);
    _sensor->configSystem();
}

void SriDevice::startRealTimeSample()
{
    _sensor->startGsdMode();
}

void SriDevice::stopRealTimeSample()
{
    _sensor->stopRealTimeSample();
}

void SriDevice::singleSample()
{
    _sensor->startGodMode();
}

Eigen::Matrix<double, M812X_CHN_NUMBER, 1> SriDevice::readForceData()
{
    return _sensor->readForceData();
}
