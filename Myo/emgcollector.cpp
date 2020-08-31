#include "emgcollector.h"
#include <QtMath>
#include <iostream>
#include <QDebug>

/**************************************************************************
 * 2020.06.11   宋世奇
 * 1. 在该版本的程序中将数据成员更新为私有成员，并设置了公有访问接口
 * 2. qtmath中有一些函数，可以将robot中的一些函数进行更新
 * 3. 目前程序中有一个小问题，当刚启动Myo采集时会先跳到High para后回到Low para
**************************************************************************/

MyoListener::MyoListener(QObject *parent) : QObject(parent),_emgSamples()
{
    for (int i = 0; i < MYO_CHN_NUMBER; ++i)
    {
        emgRecordData[i] = new int[RECORD_BUFFER_SIZE];
        memset(emgRecordData[i],0,RECORD_BUFFER_SIZE);
    }
    _recordCounter = 0;
    //    _emgThreshold = 0.02 * qPow(128,2);          // 可能过大了
    _emgThreshold = 0.01 * qPow(64,2);         // 握拳测试使用的阈值

    memset(_meanValue,0,RECORD_BUFFER_SIZE);
    memset(_emgPower,0,RECORD_BUFFER_SIZE);

    _currentActiveLevel = 0;
    _lastActiveLevel = 0;
}

MyoListener::~MyoListener()
{
    for (int i = 0; i < MYO_CHN_NUMBER; ++i)
    {
        delete emgRecordData[i];
    }
}

void MyoListener::onUnpair(myo::Myo *myo, uint64_t timestamp)
{
    Q_UNUSED(myo);
    Q_UNUSED(timestamp);

    _emgSamples.fill(0);
}

void MyoListener::onEmgData(myo::Myo *myo, uint64_t timestamp, const int8_t *emg)
{
    Q_UNUSED(myo);
    Q_UNUSED(timestamp);

    for (size_t i = 0; i < MYO_CHN_NUMBER; i++)
    {
        _emgSamples[i] = static_cast<int>(emg[i]);

        if(_recordCounter >= RECORD_BUFFER_SIZE)
        {
            _recordCounter = 0;
        }
        // 存储300点的历史数据,并转换成int类型的数据
        emgRecordData[i][_recordCounter] = _emgSamples[i];
    }
    ++_recordCounter;

    // TODO: 添加数据处理的代码
    emgDataProcess();
}

void MyoListener::onDisconnect(myo::Myo *myo, uint64_t timestamp)
{
    Q_UNUSED(myo);
    Q_UNUSED(timestamp);

    std::cout << "Myo has disconnected." << std::endl;
}

std::array<int, MYO_CHN_NUMBER> MyoListener::getEmgData()
{
    return _emgSamples;
}

void MyoListener::emgDataProcess()
{
    // 对每一时刻的8个通道的数据进行平均
    for (int t = 0; t < RECORD_BUFFER_SIZE; ++t) {
        int sum = 0;
        for (int chnIndex = 0; chnIndex < MYO_CHN_NUMBER; ++chnIndex) {
            sum += emgRecordData[chnIndex][t];
        }

        _meanValue[t] = 1.0 * sum / MYO_CHN_NUMBER;     // 8个通道的数据进行平均
        _emgPower[t] = qPow(_meanValue[t],2);           // 平方求能量
    }

    // 对300点的_emgPower进行求和平均做为当前点的激活值，平滑处理;该段代码可以优化，和上面的放一块，但是为了逻辑清晰先这么写
    double sum = 0;
    for(int i = 0; i < RECORD_BUFFER_SIZE; ++i)
    {
        sum += _emgPower[i];
    }
    double tmp = sum / RECORD_BUFFER_SIZE;                  // 求和平均
    _currentActiveLevel = (tmp > _emgThreshold) ? tmp : 0;  // 计算整个窗口的处理结果
    if(_lastActiveLevel <= _emgThreshold && _currentActiveLevel > _emgThreshold)
    {
        // TODO: 代表需要从低导纳值切换到高导纳值
        emit changeToHighPara();
        //        qDebug()<<tr("Change to high admittance parameter");
    }else if(_lastActiveLevel > _emgThreshold && _currentActiveLevel <= _emgThreshold)
    {
        // TODO: 代表需要从高导纳值切换到低导纳值
        emit changeToLowPara();
        //        qDebug()<<tr("Change to low admittance parameter");
    }
    _lastActiveLevel = _currentActiveLevel;
}



//----------------------------------------------------------------------

MyoDevice::MyoDevice(QObject *parent): QObject(parent),_stopped(false)
{
    _myoListener = new MyoListener;
    _stopped = false;
}

MyoDevice::~MyoDevice()
{
    delete _myoListener;
}

// 接收到数据的数据处理函数，主程序中应该不需要返回emg的原始值，只需要接收导纳值切换的信号即可
// 所以该段函数可以无返回值，只要发出一个信号即可
std::array<int, MYO_CHN_NUMBER> MyoDevice::readEmgData()
{
    return _myoListener->getEmgData();
}

void MyoDevice::stopEmgSample()
{
    qDebug()<<"MyoDevice-stop()";
    _stopped = true;
}

bool MyoDevice::isMyoStop()
{
    return _stopped;
}

MyoListener *MyoDevice::getMyoObject()
{
    return _myoListener;
}

void MyoDevice::startEmgSample()
{
    try {
        myo::Hub hub("com.example.emg-data-sample");
        std::cout << "Attemp to find Myo..." << std::endl;
        myo::Myo* myo = hub.waitForMyo(10000);

        if(!myo){
            throw std::runtime_error("Unable to find Myo");
        }

        std::cout<<"Connected to Myo armband!" << std::endl << std::endl;
        myo->setStreamEmg(myo::Myo::streamEmgEnabled);

        hub.addListener(_myoListener);

        while (!_stopped)
        {
            hub.run(MYO_SAMPLE_TIME);
        }
    } catch (const std::exception& e) {
        // 1.弹框提示
        std::cerr<<"ERROR:"<<e.what()<<std::endl;
        std::cout<<"Please refresh this application"<<std::endl;

        // 2.发送信号
        emit errorOccur(e.what());
        // 3.清空缓冲区
        std::cin.ignore();
        return;
    }

}
