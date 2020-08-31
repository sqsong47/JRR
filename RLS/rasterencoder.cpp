#include "rasterencoder.h"
#include <QByteArray>
#include <QDebug>
#include <QString>

RasterEncoder::RasterEncoder(QObject *parent) : QObject(parent)
{
    _serialport = new QSerialPort;
    _commandState = RasterEncoderCommand::SRT_R;

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
            qDebug()<<tr("Port %1 is avaliable").arg(serial.portName());
            serial.close();
            ++_avaliablePortNumber;
        }
    }

    // 收到数据之后的槽函数
    connect(_serialport,SIGNAL(readyRead()),this,SLOT(onReceiverData()));
}

RasterEncoder::~RasterEncoder()
{
    delete _serialport;
    delete m_pDealDataBuffer;
}


void RasterEncoder::openSerialPort()
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

    _serialport->setBaudRate(115200);
    _serialport->setDataBits(QSerialPort::Data8);
    _serialport->setParity(QSerialPort::NoParity);
    _serialport->setStopBits(QSerialPort::OneStop);
    _serialport->setFlowControl(QSerialPort::NoFlowControl);

    // 配置采样频率和机器时钟
    QByteArray command = "7F057F7D027D";
    _serialport->write(QByteArray::fromHex(command));
    qDebug()<<QByteArray::fromHex(command);

}

void RasterEncoder::singleSample()
{
    // 每次发送时都清空buffer以及重置counter
    memset(mRxBuffer,0,sizeof (mRxBuffer));
    mRxCounter = 0x00;

    // 单次数据采集的指令
    QByteArray command = "7E027E";
    _serialport->write(QByteArray::fromHex(command));
    qDebug()<<QByteArray::fromHex(command);

    _commandState = RasterEncoderCommand::SS_R;
}

void RasterEncoder::showAlgorithmData()
{
    for(unsigned long i = 0x00; i < sizeof (mRxBuffer); ++i)
    {
        // 第一个是0x7e,第二个字节不是0x7e
        if(mRxBuffer[i] == 0x7e && mRxBuffer[i+1] != 0x7e)
        {
            unsigned long long pulseCounts = mRxBuffer[i+1] * 16777216 + mRxBuffer[i+2] * 65536 + mRxBuffer[i+3] * 256 + mRxBuffer[i+4];
            _angleValue = 360.0 * static_cast<double>(pulseCounts) / static_cast<double>(67108864);
            // 67108864
            // 4294967295
            return;
        }
    }
    qDebug()<<"There is no ox7e in mRxBuffer";
}

void RasterEncoder::realTimeSample()
{
    // 清空buffer，重置counter
    memset(mRxBuffer,0,sizeof (mRxBuffer));
    mRxCounter = 0x00;

    // 额外的数据处理,这一部分写的不好，有待改进
    Data_Ptr_Out = mRxCounter;
    m_dwDealDataIndex = 0x00;

    // 连续数据采集的指令
    QByteArray command = "7E017E";
    _serialport->write(QByteArray::fromHex(command));
    qDebug()<<QByteArray::fromHex(command);

    _commandState = RasterEncoderCommand::RT_R;
}

void RasterEncoder::realTimeProcess()
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

        // 如果数据包长度小于一帧数据的长度，代表此次待读取的数据信息不足，此时需要break，等待下一次的onReceiveData();
        if(RxLengthTemp < 10)
        {
            break;
        }


        // 做一个循环找到数据帧头
        for (i = 0x00; i < RxLengthTemp; i++)
        {
            if(HeaderIndex == sizeof (mRxBuffer))
            {
                HeaderIndex = 0x00;
            }

            if(HeaderIndex == sizeof (mRxBuffer) - 1)
            {
                if(mRxBuffer[HeaderIndex] == 0x7e && mRxBuffer[0] != 0x7e)
                {
                    DataHeaderFlag = true;
                    break;
                }
            }else if (mRxBuffer[HeaderIndex] == 0x7e && mRxBuffer[HeaderIndex+1] != 0x7e)
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

        if(RxLengthTemp < 10)	// 此时的数据长度如果小于有效数据帧的长度还break
        {
            break;
        }

        unsigned long MoveIndex = HeaderIndex + 1;    // 角度数据的起点

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


        // 做四次转存，将角度信息存放待处理的buffer中
        for (i = 0x00; i < 4; i++) {
            // 将角度数据进行转存，m_pDealDataBuffer里面全是角度信息
            m_pDealDataBuffer[m_dwDealDataIndex++] = mRxBuffer[MoveIndex++];

            if(MoveIndex == sizeof (mRxBuffer))
            {
                MoveIndex = 0x00;
            }
        }


        // -------------------------------------------------------------------
        Data_Ptr_Out = MoveIndex + 5;			// Data_Ptr_Out移动到下一帧开头的0x7e
        if(Data_Ptr_Out >= sizeof (mRxBuffer))
        {
            Data_Ptr_Out -= sizeof (mRxBuffer);
        }

        // 等于0的话代表没有进行数据存储，m_dwDealDataIndex只可能是4的倍数，小于16384-4，且不可能等于0
        // m_dwDealDataIndex由于每次会重置，其值最大为4
        if(m_dwDealDataIndex == 0x00)
        {
            return;
        }

        // PointCounter其值也只可能为1，确保有一组点
        PointCounter = m_dwDealDataIndex / 4;   // 整数相除，记录处理第几组点

        // 这个判断条件不合理，所以注释掉了,PointCounter的上限应该是sizeof (mRxBuffer) / 3)
        if( (PointCounter >= sizeof (mRxBuffer)) || (PointCounter <= 0) )
        {
            m_dwDealDataIndex = 0x00;
            return;
        }

        unsigned long DispChSum[4] = {0x00};        // 将来用来接收数据
        for(i = 0; i < 4; i++)
        {
            // 再进行一次转存，其实没有必要，一次就可以搞定
            DispChSum[i] = m_pDealDataBuffer[i];
        }

        // 进行数据显示
        unsigned long long pulseCounts = DispChSum[0] * 16777216 + DispChSum[1] * 65536 + DispChSum[2] * 256 + DispChSum[3];
        _angleValue = 360.0 * static_cast<double>(pulseCounts) / static_cast<double>(67108864);

        // 重置参数
        m_dwDealDataIndex = 0x00;
    }
}

void RasterEncoder::stopRealTimeSample()
{
    // 停止连续数据采集的指令
    QByteArray command = "7E007E";
    _serialport->write(QByteArray::fromHex(command));
    qDebug()<<QByteArray::fromHex(command);

    _commandState = RasterEncoderCommand::SRT_R;
}



void RasterEncoder::closeSerialPort()
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

double RasterEncoder::requestAngleValue()
{
    return _angleValue;
}

int RasterEncoder::requestAvaliablePortNumber()
{
    return _avaliablePortNumber;
}

void RasterEncoder::onReceiverData()
{
    // 将信号进行转存
    QByteArray dataInSerialPort;
    dataInSerialPort = _serialport->readAll();

    if(dataInSerialPort.isEmpty())
    {
        qDebug()<<"There is no data avaliable";
        return;
    }

    // qDebug()<<tr("Message: %1").arg(QString(dataInSerialPort.toHex()) );

    int dataLength = dataInSerialPort.size();
    // qDebug()<<"dataLen: "<<dataLength;

    for (int i = 0x00; i < dataLength; ++i) {
        if(mRxCounter >= sizeof (mRxBuffer))
        {
            mRxCounter = 0x00;
        }
        mRxBuffer[mRxCounter++] = static_cast<unsigned char>(dataInSerialPort.data()[i]);
    }

    // 之后根据command state进行不同的操作
    if(_commandState == RasterEncoderCommand::SS_R)
    {
        showAlgorithmData();
    }else if (_commandState == RasterEncoderCommand::RT_R)
    {
        realTimeProcess();
    }else if (_commandState == RasterEncoderCommand::SRT_R)
    {
        memset(mRxBuffer,0,sizeof (mRxBuffer));
        mRxCounter = 0x00;
    }
    return;
}




//-------------------------------------------------------
RasterRLS::RasterRLS(QObject *parent): QObject(parent)
{
    _encoder = new RasterEncoder;
    _angleValue = 0;
}

RasterRLS::~RasterRLS()
{
    delete _encoder;
}

RasterEncoder *RasterRLS::getObject()
{
    return _encoder;
}


void RasterRLS::startRealtimeSample()
{
    _encoder->realTimeSample();
}

void RasterRLS::startSingleSample()
{
    _encoder->singleSample();
}

void RasterRLS::stopRealtimeSample()
{
    _encoder->stopRealTimeSample();
}

void RasterRLS::closeDevice()
{
    _encoder->closeSerialPort();
}

double RasterRLS::requestAngleValue()
{
    _angleValue = _encoder->requestAngleValue();
    return _angleValue;
}

int RasterRLS::requestAvaliablePortNumber()
{
    return _encoder->requestAvaliablePortNumber();
}

void RasterRLS::openDevice()
{
    _encoder->openSerialPort();
}
