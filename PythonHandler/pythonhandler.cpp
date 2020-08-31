#include "pythonhandler.h"
#include <QDebug>
#include "pythreadstatelock.h"

/*
 * 目前程序存在以下几个问题
 * 1. 在向python脚本传输数据的时候，因为存在bool值，其格式控制符可能不对
 * 2. run函数现在未启动，需要最后确定其位置
 * 3. pythonHandler在一个新线程中，没有确定是不是python的脚本是不是在该线程中工作
 * 4. my_step函数中存在延时函数，不知道是不是C++线程会使其跳过去
 * 5. PytObject对象的内存申请可能有更好的方法
 * 6. 注意更换python脚本后需要向myfile.pth中输入路径
*/

PythonHandler::PythonHandler(QObject *parent) : QObject(parent)
{

    // PyObject类的对象销毁是特定的函数Py_DecRef
    // 内存申请的函数可以用new，但可能有更合适的写法
    _pModule = new PyObject;
    _pRet = new PyObject;
    _pClassDDPG = new PyObject;
    _pInstance = new PyObject;
}

// 初始化函数可以写到构造函数体里，那样的话构造的时候就会执行
bool PythonHandler::initPython()
{
    // 告知python根目录
    Py_SetPythonHome(const_cast<wchar_t *>(L"E:/Users/SuperMan/Anaconda3/envs/tf1"));
    qDebug() << Py_GetVersion();


    // 初始化python解释器，调用的第一步
    Py_Initialize();

    if(!Py_IsInitialized())
    {
        qDebug() << "Can not init python";
        return false;
    }else{
        // 初始化线程支持
        PyEval_InitThreads();

        // 检测线程支持是否开启成功
        int nInit = PyEval_ThreadsInitialized();
        if(nInit)
        {
            qDebug() << "Thread has been reset";
        }else {
            return false;
        }
        qDebug() << "Python has been inited";
    }

    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('./')");

    PyEval_ReleaseThread(PyThreadState_Get());
    return true;
}

bool PythonHandler::getPyScript()
{
    // 获得全局锁
    class PyThreadStateLock PyThreadLock;

    _pModule = PyImport_ImportModule("DDPG");
    if(!_pModule)
    {
        qDebug() << "Can not import DDPG";
        return false;
    }else {
        qDebug() << "Module has been imported";
    }


    //创建类
    _pClassDDPG = PyObject_GetAttrString(_pModule,"DDPG");
    if(!_pClassDDPG)
    {
        qDebug() << "Can not create a DDPG class";
        return false;
    }else {
        qDebug() << "DDPG has been imported";
    }


    //类实例化
    //    _pInstance = PyInstanceMethod_New(_pClassDDPG);
    _pInstance  = PyObject_CallObject(_pClassDDPG, nullptr);
    if(!_pInstance)
    {
        qDebug() << "Can not create a DDPG instance";
        return false;
    }else {
        qDebug() << "DDPG instance has been created";
    }
    return true;
}

void PythonHandler::startRun()
{
    // 获得全局锁
    class PyThreadStateLock PyThreadLock;

    // 启动run函数,必须在外部进行run函数的启动，否则程序会卡死
    PyObject_CallMethod(_pInstance, "run", nullptr);
}

double PythonHandler::trainOneStep(const double velocity,
                                   const double accelaration, const double jerk)
{
    // 获得全局锁
    class PyThreadStateLock PyThreadLock;

    // 需要查看bool类型的值是不是“i”,格式控制尤其要注意
    PyObject* pArg = PyTuple_New(3);

    PyTuple_SetItem(pArg, 0, Py_BuildValue("f", velocity));
    PyTuple_SetItem(pArg, 1, Py_BuildValue("f", accelaration));
    PyTuple_SetItem(pArg, 2, Py_BuildValue("f", jerk));

    // Python 脚本中的 step 有延时函数
    _pRet = PyObject_CallMethod(_pInstance, "step", "O", pArg);
    if(!_pRet)
    {
        PyErr_Print();
        Py_Finalize();
        return -404;
    }

    return PyFloat_AsDouble(_pRet);
}

double PythonHandler::sumReward()
{
    // 获得全局锁
    class PyThreadStateLock PyThreadLock;

    _pRet = PyObject_CallMethod(_pInstance, "show_episode_result", nullptr);
    if(!_pRet)
    {
        PyErr_Print();
        Py_Finalize();
        return -404;
    }
    return PyFloat_AsDouble(_pRet);
}

void PythonHandler::testCallMethod()
{
    class PyThreadStateLock PyThreadLock;
    _pRet = PyObject_CallMethod(_pInstance, "test_fun", nullptr);
    if(!_pRet)
    {
        qDebug() << "ERROR";
        return;
    }
    qDebug() << PyLong_AsLong(_pRet);
}

PythonHandler::~PythonHandler()
{
    Py_DecRef(_pInstance);
    Py_DecRef(_pClassDDPG);
    Py_DecRef(_pRet);
    Py_DecRef(_pModule);
    Py_Finalize();
}
