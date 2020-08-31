#ifndef EMGCOLLECTOR_H
#define EMGCOLLECTOR_H

#include <QObject>
#include <Myo/myo.hpp>
#include <array>

/*******************************************************************
 * 2020.06.14   宋世奇
 * 1. 定义了采样时间的全局变量，修改了MyoDevice的采样时间代码段
 * 2. 修改了接口函数，将int8_t的array改成了int类型的；
 * 3. 程序编译会出现大量的C4100警告，这是使用msvc编译器造成的。一般出现才重写
 *    父类的虚函数中才会出现；项目文件中已经添加了解决代码
 * 4. C4819与编码有关，也是使用msvc编译器造成的，在文本编辑器的行为中可以修改
 * 5. 这里采用的窗宽是64，即信号强度超过阈值，并保持128ms进行切换
 * 6. 整个程序有些类没有写析构函数，一部分是因为QT自己有内存管理，另一部分需要
 *    再排查一遍，该加的加上
 * 7. 修改了MyoDevice的停止采集的函数名
*******************************************************************/


const unsigned char MYO_SAMPLE_TIME = 2;             // 采样周期2ms，500Hz；最大为1kHz
const int RECORD_BUFFER_SIZE = 300;                  // 历史记录缓冲区，500Hz情况下，窗宽时间为0.6s
const int MYO_CHN_NUMBER = 8;                        // 通道数---8

class MyoListener : public QObject,public myo::DeviceListener
{
    Q_OBJECT
public:
    explicit MyoListener(QObject *parent = nullptr);
    ~MyoListener();

    void onUnpair(myo::Myo* myo,uint64_t timestamp);
    void onEmgData(myo::Myo* myo,uint64_t timestamp,const int8_t* emg);
    void onDisconnect(myo::Myo* myo, uint64_t timestamp);

    std::array<int,MYO_CHN_NUMBER> getEmgData();

private:
    // 数据缓冲区
    std::array<int, MYO_CHN_NUMBER> _emgSamples;
    void emgDataProcess();

    int* emgRecordData[MYO_CHN_NUMBER];
    int _recordCounter;

    double _meanValue[RECORD_BUFFER_SIZE];  // 平均值
    double _emgPower[RECORD_BUFFER_SIZE];   // 能量值
    double _emgThreshold;                   // 阈值
    double _currentActiveLevel;
    double _lastActiveLevel;

signals:
    void changeToHighPara();
    void changeToLowPara();

public slots:
};



// 定义了myo设备的使用方法
class MyoDevice : public QObject{
    Q_OBJECT
public:
    explicit MyoDevice(QObject* parent = nullptr);
    ~MyoDevice();

    // 接口函数
    std::array<int,MYO_CHN_NUMBER> readEmgData();
    bool isMyoStop();
//    MyoListener* _myoListener;
    MyoListener* getMyoObject();

private:
    MyoListener* _myoListener;

    volatile bool _stopped;     // 加一个volatile，即使在死循环中也能检测并改变stopped的值

signals:
    void errorOccur(const std::string);

    // 导纳值切换的信号
    void crossThreshold();

public slots:
    void startEmgSample();
    void stopEmgSample();
};


#endif // EMGCOLLECTOR_H
