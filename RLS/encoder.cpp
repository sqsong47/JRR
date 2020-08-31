#include "encoder.h"
#include <QByteArray>
#include <QDebug>
#include <QString>

MagneticEncoder::MagneticEncoder(QObject *parent) : QObject(parent)
{
    _serialport = new QSerialPort;
    _commandState = MagneticEncoderCommand::SRT;

    memset(mRxBuffer,0,sizeof (mRxBuffer));
    mRxCounter = 0x00;

    Data_Ptr_Out = 0xFFFFFFFF;

    m_pDealDataBuffer = new unsigned char[16384];
    memset(m_pDealDataBuffer,0,16384);

    _angleValue = 0;
    _avaliablePortNumber = 0;

    // 测试可用串口
    foreach(const QSerialPortInfo& info,QSerialPortInfo::availablePorts() )
    {
        QSerialPort serial;
        serial.setPort(info);
        if(serial.open(QIODevice::ReadWrite))
        {
            // qDebug()<<serial.portName();
            // qDebug()<<tr("Port %1 is avaliable").arg(serial.portName());
            serial.close();
            ++_avaliablePortNumber;
        }
    }

    // 收到数据之后的槽函数
    connect(_serialport,SIGNAL(readyRead()),this,SLOT(onReceiverData()));
}

MagneticEncoder::~MagneticEncoder()
{
    delete _serialport;
    delete m_pDealDataBuffer;
}


void MagneticEncoder::openSerialPort()
{
    QString portName;

    // 打开第一个可用的串口，之后进行设置
    foreach(const QSerialPortInfo& info, QSerialPortInfo::availablePorts())
    {
        QSerialPort serial;
        serial.setPort(info);
        if(serial.open(QIODevice::ReadWrite))
        {
            portName = serial.portName();
            serial.close();
            break;
        }
    }

    // 重新打开
    _serialport->setPortName(portName);
    if(!_serialport->open(QIODevice::ReadWrite))
    {
        qDebug()<<tr("Fail to open %1").arg(portName);
        return;
    }

    qDebug()<<tr("%1 has been opened").arg(portName);

    _serialport->setBaudRate(256000);
    _serialport->setDataBits(QSerialPort::Data8);
    _serialport->setParity(QSerialPort::NoParity);
    _serialport->setStopBits(QSerialPort::OneStop);
    _serialport->setFlowControl(QSerialPort::NoFlowControl);
}

void MagneticEncoder::singleSample()
{
    // 每次发送时都清空buffer以及重置counter
    memset(mRxBuffer,0,sizeof (mRxBuffer));
    mRxCounter = 0x00;

    // 发送0x31,之后进行信号和槽的连接
    _commandState = MagneticEncoderCommand::SS;
    _serialport->write("1");
}

void MagneticEncoder::showAlgorithmData()
{
    for(unsigned long i = 0x00; i < sizeof (mRxBuffer); ++i)
    {
        if(mRxBuffer[i] == 0xea)
        {
            unsigned long long pulseCounts = mRxBuffer[i+1] * 65536 + mRxBuffer[i+2] * 256 + mRxBuffer[i+3];
            _angleValue = 360.0 * static_cast<double>(pulseCounts) / static_cast<double>(16772115);
            return;
        }
    }
    qDebug()<<"There is no oxea in mRxBuffer";
}

void MagneticEncoder::realTimeSample()
{
    // 清空buffer，重置counter
    memset(mRxBuffer,0,sizeof (mRxBuffer));
    mRxCounter = 0x00;

    // 额外的数据处理,这一部分写的不好，有待改进
    Data_Ptr_Out = mRxCounter;
    m_dwDealDataIndex = 0x00;

    // 发送指令
    _commandState = MagneticEncoderCommand::RT;
    _serialport->write("2");
}

void MagneticEncoder::realTimeProcess()
{
    if(Data_Ptr_Out == 0xFFFFFFFF)
    {
        qDebug()<<"Can not find Data_Ptr_Out";
        return;
    }

    bool DataHeaderFlag = false;
    unsigned long i = 0x00;

    unsigned long HeaderIndex= 0x00;
    unsigned long PointCounter = 0x00;
    unsigned RxLengthTemp = 0x00;

    unsigned RxCounterCurrent= mRxCounter;

    // 数据处理循环
    while(1)
    {

        HeaderIndex = Data_Ptr_Out;

        if(RxCounterCurrent >= HeaderIndex)
        {
            RxLengthTemp = RxCounterCurrent - HeaderIndex;
        }else
        {
            RxLengthTemp = sizeof (mRxBuffer) - HeaderIndex + RxCounterCurrent;
        }

        //        qDebug()<<"RxLengthTemp: "<<RxLengthTemp;
        // 如果数据包长度小于一帧数据的长度，代表此次待读取的数据信息不足，此时需要break，等待下一次的onReceiveData();
        if(RxLengthTemp < 8)
        {
            break;
        }


        // 做一个循环找到数据帧头
        for (i = 0x00; i<RxLengthTemp; i++) {
            if(HeaderIndex == sizeof (mRxBuffer))
            {
                HeaderIndex = 0x00;
            }
            // 当HeaderIndex为合法值是看是否为0xea
            if(mRxBuffer[HeaderIndex] == 0xea)
            {
                DataHeaderFlag = true;
                break;
            }
            HeaderIndex++;
        }
        // 如果标示量还是false，代表没有找到合法的数据帧头
        if(DataHeaderFlag == false)
        {
            break;
        }
        DataHeaderFlag = false;     // 重置为false，为下次做准备

        // HeaderIndex更新为合法的数据帧头之后重新计算数据包的长度
        if(RxCounterCurrent >= HeaderIndex)//LYY
        {
            RxLengthTemp = RxCounterCurrent - HeaderIndex;
        }else
        {
            RxLengthTemp = sizeof (mRxBuffer) - HeaderIndex + RxCounterCurrent;
        }

        if(RxLengthTemp < 8)	// 此时的数据长度如果小于有效数据帧的长度还break
        {
            break;
        }

        unsigned long MoveIndex = HeaderIndex+1;    // 角度数据的起点

        // 保证数据的有效性
        if(MoveIndex >= sizeof (mRxBuffer))
        {
            MoveIndex = MoveIndex - sizeof (mRxBuffer);
        }

        // 不会出现，每次循环都重置为0了,m_dwDealDataIndex能到16381？
        if(m_dwDealDataIndex >= sizeof (mRxBuffer) - 3)
        {
            m_dwDealDataIndex = 0x00;
            return;
        }
        for (i = 0x00;i<3;i++) {
            // 将角度数据进行转存，m_pDealDataBuffer里面全是角度信息
            m_pDealDataBuffer[m_dwDealDataIndex++] = mRxBuffer[MoveIndex++];
            if(MoveIndex == sizeof (mRxBuffer))
            {
                MoveIndex = 0x00;
            }
        }

        Data_Ptr_Out = MoveIndex + 3;			// Data_Ptr_Out移动到停止位
        if(Data_Ptr_Out >= sizeof (mRxBuffer))
        {
            Data_Ptr_Out -= sizeof (mRxBuffer);
        }

        // 等于0的话代表没有进行数据存储，m_dwDealDataIndex只可能是3的倍数，小于16384-3，且不可能等于0
        // m_dwDealDataIndex由于每次会重置，其值最大为3
        if(m_dwDealDataIndex == 0x00)
        {
            return;
        }

        // PointCounter其值也只可能为1，确保有一组点
        PointCounter = m_dwDealDataIndex/3;   // 整数相除，记录处理第几组点

        // 这个判断条件不合理，所以注释掉了,PointCounter的上限应该是sizeof (mRxBuffer)/3)
        // if( (PointCounter >= sizeof (mRxBuffer)) || (PointCounter <= 0) )
        if( (PointCounter >= sizeof (mRxBuffer)) || (PointCounter <= 0) )
        {
            m_dwDealDataIndex = 0x00;
            return;
        }

        unsigned long DispChSum[3] = {0x00};        // 将来用来接收数据
        for(i = 0; i < 3; i++)
        {
            DispChSum[i] = m_pDealDataBuffer[i];
        }

        // 进行数据显示
        unsigned long long pulseCounts = DispChSum[0] * 65536 + DispChSum[1] * 256 + DispChSum[2];
        _angleValue = 360.0 * static_cast<double>(pulseCounts) / static_cast<double>(16777215) ;

        // 重置参数
        m_dwDealDataIndex = 0x00;
    }
}

void MagneticEncoder::stopRealTimeSample()
{
    _commandState = MagneticEncoderCommand::SRT;
    _serialport->write("0");
}

void MagneticEncoder::requestVersion()
{
    memset(mRxBuffer,0,sizeof (mRxBuffer));
    mRxCounter = 0x00;

    _commandState = MagneticEncoderCommand::VS;
    _serialport->write("v");
}

void MagneticEncoder::closeSerialPort()
{
    if(!_serialport->isOpen())
    {
        qDebug()<<"serial port is not open";
        return;
    }

    if(!_serialport->clear())
    {
        qDebug()<<"Fail to close serial port";
        return;
    }
    _serialport->close();
    qDebug()<<"Serial port has been closed";
}

double MagneticEncoder::requestAngleValue()
{
    return _angleValue;
}

int MagneticEncoder::requestAvaliablePortNumber()
{
    return _avaliablePortNumber;
}

void MagneticEncoder::onReceiverData()
{
    // 将信号进行转存
    QByteArray dataInSerialPort;
    dataInSerialPort = _serialport->readAll();

    if(dataInSerialPort.isEmpty())
    {
        qDebug()<<"There is no data avaliable";
        return;
    }

    int dataLength = dataInSerialPort.size();
    for (int i = 0x00; i < dataLength; ++i) {
        if(mRxCounter >= sizeof (mRxBuffer))
        {
            mRxCounter = 0x00;
        }
        mRxBuffer[mRxCounter++] = static_cast<unsigned char>(dataInSerialPort.data()[i]);
    }

    // 之后根据command state进行不同的操作
    if(_commandState == MagneticEncoderCommand::SS)
    {
        showAlgorithmData();
    }else if (_commandState == MagneticEncoderCommand::RT)
    {
        realTimeProcess();
    }else if (_commandState == MagneticEncoderCommand::VS)
    {
        QString str = static_cast<QString>(dataInSerialPort);
        qDebug() << str;
    }else if (_commandState == MagneticEncoderCommand::SRT)
    {
        memset(mRxBuffer,0,sizeof (mRxBuffer));
        mRxCounter = 0x00;
    }
    return;
}




//----------------------------------------------------------------
MagnecticRLS::MagnecticRLS(QObject *parent): QObject(parent)
{
    _encoder = new MagneticEncoder;
    _angleValue = 0;
}

MagnecticRLS::~MagnecticRLS()
{
    delete _encoder;
}

MagneticEncoder *MagnecticRLS::getObject()
{
    return _encoder;
}

void MagnecticRLS::requestVersion()
{
    // 执行该行代码会打印出来编码器的版本信息
    _encoder->requestVersion();
}

void MagnecticRLS::startRealtimeSample()
{
    _encoder->realTimeSample();
}

void MagnecticRLS::startSingleSample()
{
    _encoder->singleSample();
}

void MagnecticRLS::stopRealtimeSample()
{
    _encoder->stopRealTimeSample();
}

void MagnecticRLS::closeDevice()
{
    _encoder->closeSerialPort();
}

double MagnecticRLS::requestAngleValue()
{
    _angleValue = _encoder->requestAngleValue();
    return _angleValue;
}

int MagnecticRLS::requestAvaliablePortNumber()
{
    return _encoder->requestAvaliablePortNumber();
}

void MagnecticRLS::openDevice()
{
    _encoder->openSerialPort();
}
