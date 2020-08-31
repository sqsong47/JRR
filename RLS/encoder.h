#ifndef ENCODER_H
#define ENCODER_H

#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QString>

enum MagneticEncoderCommand{
     VS ,  // version
     SS ,  // single sample
     RT ,  // realtime
     SRT   // stop realtime
};


class MagneticEncoder : public QObject
{
    Q_OBJECT
public:
    explicit MagneticEncoder(QObject *parent = nullptr);
    ~MagneticEncoder();

public:
    void openSerialPort();
    void singleSample();
    void showAlgorithmData();
    void realTimeSample();
    void realTimeProcess();
    void stopRealTimeSample();
    void requestVersion();
    void closeSerialPort();
    double requestAngleValue();
    int requestAvaliablePortNumber();

private:
    QSerialPort* _serialport;
    MagneticEncoderCommand _commandState;
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
    void onReceiverData(); 
};


// ----------------------------------------------------
class MagnecticRLS : public QObject
{
    Q_OBJECT

public:
    explicit MagnecticRLS(QObject *parent = nullptr);
    ~MagnecticRLS();

    MagneticEncoder* getObject();
    void requestVersion();
    void startRealtimeSample();
    void startSingleSample();
    void stopRealtimeSample();
    void closeDevice();
    void openDevice();

    double requestAngleValue();
    int requestAvaliablePortNumber();
private:
    MagneticEncoder* _encoder;
    double _angleValue;

signals:

public slots:

};


#endif // ENCODER_H
