#ifndef RASTERENCODER_H
#define RASTERENCODER_H


#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QString>


enum RasterEncoderCommand{
     SS_R ,  // single sample
     RT_R ,  // realtime
     SRT_R   // stop realtime
};


class RasterEncoder : public QObject
{
    Q_OBJECT
public:
    explicit RasterEncoder(QObject *parent = nullptr);
    ~RasterEncoder();

public:
    void openSerialPort();              // 打开串口，已改
    void singleSample();                // 发送单次采集指令，已改
    void showAlgorithmData();           // 单次采集的代码，已改
    void realTimeSample();              // 发送连续采集指令，已改
    void realTimeProcess();             // 连续数据采集的数据处理代码段，待验证
    void stopRealTimeSample();          // 发送停止连续数据采集的指令，已改
    void closeSerialPort();             // 关闭串口，不需要修改
    double requestAngleValue();         // 返回角度值，不需要修改
    int requestAvaliablePortNumber();   // 可用串口的数目，不需要修改

private:
    QSerialPort* _serialport;
    RasterEncoderCommand _commandState;
    double _angleValue;
    int _avaliablePortNumber;

private:
    unsigned char mRxBuffer[16384];
    unsigned long mRxCounter;

    unsigned long Data_Ptr_Out;
    unsigned long m_dwDealDataIndex;
    unsigned char* m_pDealDataBuffer;

signals:

public slots:
    void onReceiverData();              // 响应的槽函数，已改
};


// ----------------------------------------------------
class RasterRLS : public QObject
{
    Q_OBJECT

public:
    explicit RasterRLS(QObject *parent = nullptr);
    ~RasterRLS();

    RasterEncoder* getObject();
    void startRealtimeSample();
    void startSingleSample();
    void stopRealtimeSample();
    void closeDevice();
    void openDevice();

    double requestAngleValue();
    int requestAvaliablePortNumber();
private:
    RasterEncoder* _encoder;
    double _angleValue;

signals:

public slots:

};


#endif // RASTERENCODER_H
