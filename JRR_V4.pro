#-------------------------------------------------
#
# Project created by QtCreator 2020-06-10T14:36:54
#
#-------------------------------------------------

QT       += core gui
QT       += network
QT       += serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets


INCLUDEPATH     += C:\Users\SuperMan\Desktop\JRR_V4\Eigen\Eigen
INCLUDEPATH     += C:\Users\SuperMan\Desktop\JRR_V4\Robot
INCLUDEPATH     += C:\Users\SuperMan\Desktop\JRR_V4\Sri
INCLUDEPATH     += C:\Users\SuperMan\Desktop\JRR_V4\RLS



TARGET = JRR_V4
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11
CONFIG += console

SOURCES += \
        main.cpp \
        mainwindow.cpp \
    Epos/qmotor.cpp \
    Myo/emgcollector.cpp \
    Robot/qrobot.cpp \
    Sri/srisensor.cpp \
    RLS/encoder.cpp \
    RLS/rasterencoder.cpp \
    PythonHandler/pythonhandler.cpp \
    PythonHandler/pythreadstatelock.cpp

HEADERS += \
        mainwindow.h \
    Epos/Definitions.h \
    Epos/qmotor.h \
    Myo/emgcollector.h \
    Myo/libmyo.h \
    Myo/myo.hpp \
    Robot/qrobot.h \
    Sri/srisensor.h \
    RLS/encoder.h \
    RLS/rasterencoder.h \
    PythonHandler/pythonhandler.h \
    PythonHandler/pythreadstatelock.h

FORMS += \
        mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32: LIBS += -L$$PWD/Epos/ -lEposCmd64

INCLUDEPATH += $$PWD/Epos
DEPENDPATH += $$PWD/Epos

win32: LIBS += -L$$PWD/Myo/ -lmyo64

INCLUDEPATH += $$PWD/Myo
DEPENDPATH += $$PWD/Myo

# 加入Python的模块
#INCLUDEPATH += E:\Users\SuperMan\anaconda3\include

#LIBS += -LE:/Users/SuperMan/anaconda3/libs/ -lpython35 \
#                                            -lpython3 \
#                                            -l_tkinter

#INCLUDEPATH += E:/Users/SuperMan/anaconda3/libs
#DEPENDPATH += E:/Users/SuperMan/anaconda3/libs

INCLUDEPATH += E:/Users/SuperMan/anaconda3/envs/tf1/include

LIBS += -LE:/Users/SuperMan/anaconda3/envs/tf1/libs/ -lpython35 \
                                            -lpython3 \
                                            -l_tkinter

INCLUDEPATH += E:/Users/SuperMan/anaconda3/envs/tf1/libs
DEPENDPATH += E:/Users/SuperMan/anaconda3/envs/tf1/libs



DISTFILES += \
    DDPG.py


# 该段代码消除了C4100的警告，但是最好在整个项目完成之后再打开，因为会影响调试
#win32-msvc* {
#   QMAKE_CXXFLAGS *=  /wd"4100"
#   contains (QMAKE_CXXFLAGS_WARN_ON, -w34100) : QMAKE_CXXFLAGS_WARN_ON -= -w34100
#}
