#ifndef QMOTOR_H
#define QMOTOR_H

#include <QObject>
#include "Definitions.h"
#include <QMap>

typedef enum MotorState{
    Disconnect  = -2,
    Connect     = -1,
    Disable     = 0,
    Quickstop   = 1,
    Enable      = 2,
    Fault       = 3
}MotorState;

typedef enum OperationMode{
    SDM=-6, MEM=-5, CM=-3,  VM=-2,  PM=-1,  PPM=1,
    PVM=3,  HM=6,   IPM=7,  CSP=8,  CSV=9,  CST=10, Default=403
}OperationMode;

class QMotor : public QObject
{
    Q_OBJECT
public:
    explicit QMotor(QObject *parent = nullptr);

public:
    // Start & Close
    BOOL ePOS_OpenDeviceDlg();
    BOOL ePOS_OpendeviceByUSB();
    BOOL ePOS_OpendeviceByCAN();
    BOOL ePOS_CloseAllDevice();

    // Enable & Diable
    BOOL ePOS_SetEnableState(const WORD NodeId);
    BOOL ePOS_GetEnableState(const WORD NodeId);
    BOOL ePOS_SetDisableState(const WORD NodeId);
    BOOL ePOS_GetDiableState(const WORD NodeId);

    // QuickStop & Is quick stop or not
    BOOL ePOS_SetQuickStopState(const WORD NodeId);
    BOOL ePOS_GetQuickStopState(const WORD NodeId);

    // Clear Fault & Is in Fault or not
    BOOL ePOS_ClearFault(const WORD NodeId);
    BOOL ePOS_GetFaultState(const WORD NodeId);

    // The number of error
    BOOL ePOS_GetNbOfDeviceError(const WORD NodeId);

    // The information of error, then pErrorInfo is the information of error
    BOOL ePOS_GetErrorInfo();

    // Set & get operation Mode
    BOOL ePOS_SetOperationMode(const WORD NodeId,const enum OperationMode operationMode);
    BOOL ePOS_GetOperationMode(const WORD NodeId);
    BOOL ePOS_GetOperationMode(const WORD NodeId,char& pOperationMode);             // overload

    // Get motor state
    BOOL ePOS_GetState(const WORD NodeId);

    // Configure profile mode
    BOOL ePOS_SetMaxProfileVelocity(const WORD NodeId,const DWORD MaxProfileVelocity);
    BOOL ePOS_GetMaxProfileVeloctiy(const WORD NodeId,DWORD& pMaxProfileVelocity);

    // Is target reached or not
    BOOL ePOS_GetMovementState(const WORD NodeId);

    // Wait for target reached or time out
    BOOL ePOS_WaitForTargetReached(const WORD NodeId, const DWORD Timeout);

    // Get current position
    // position 是单调递增的，内部是一个增量式编码器，每次上电计数从0开始，之后位置正转递增
    // 起到了一个位置记录的作用。可以计算从启动开始转过了多少圈
    BOOL ePOS_GetPositionIs(const WORD NodeId,long& pPositionIs);

    // Get current velocity
    BOOL ePOS_GetVelocityIsAveraged(const WORD NodeId,long& pVelocityIsAveraged);
    BOOL ePOS_GetVelocityIs(const WORD NodeId,long& pVelocityIs);

    // Get actual current
    BOOL ePOS_GetActualCurrent(const WORD NodeId,short& pCurrentIs);
    BOOL ePOS_GetCurrentIsAveraged(const WORD NodeId,short& pCurrentIsAveraged);


    // MEM
    BOOL ePos_GetMasterEncoderParameter(const WORD NodeId, WORD& pScalingNumerator,
                                        WORD& pScalingDenominator, BYTE& pPolarity, DWORD& pMaxVelocity, DWORD& pMaxAcceleration);


    // PPM
    BOOL ePOS_ActiveProfilePositionMode(const WORD NodeId);
    BOOL ePOS_SetPositionProfile(const WORD NodeId,const DWORD ProfileVelocity,
                                 const DWORD ProfileAcceleration, const DWORD ProfileDeceleration);
    BOOL ePOS_GetPositionProfile(const WORD NodeId, DWORD &ProfileVelocity,
                                 DWORD &ProfileAcceleration, DWORD &ProfileDeceleration);
    BOOL ePOS_MoveToPosition(const WORD NodeId,const long TargetPosition,const BOOL Absolute = FALSE,
                             const BOOL Immediately = TRUE);
    BOOL ePOS_GetTargetPosition(const WORD NodeId, long& pTargetPosition);
    BOOL ePOS_HaltPositionMovement(const WORD NodeId);


    // PM
    BOOL ePOS_ActivePositionMode(const WORD NodeId);
    BOOL ePOS_SetPositionMust(const WORD NodeId,const long PositionMust);
    BOOL ePOS_GetPositionMust(const WORD NodeId, long& PositionMust);


    // PVM
    BOOL ePOS_ActiveProfileVelocityMode(const WORD NodeId);
    BOOL ePOS_SetVelocityProfile(const WORD NodeId,const DWORD ProfileAcceleration, const DWORD ProfileDeceleration);
    BOOL ePOS_GetVelocityProfile(const WORD NodeId,DWORD &ProfileAcceleration, DWORD &ProfileDeceleration);
    BOOL ePOS_MoveWithVelocity(const WORD NodeId, const long TargetVelocity);
    BOOL ePOS_GetTargetVelocity(const WORD NodeId, long& TargetVelocity);
    BOOL ePOS_HaltVelocityMovement(const WORD NodeId);

    // VM
    BOOL ePOS_ActiveVelocityMode(const WORD NodeId);
    BOOL ePOS_SetVelocityMust(const WORD NodeId,const long VelocityMust);
    BOOL ePOS_GetVelocityMust(const WORD NodeId, long& pVelocityMust);

    // CM
    BOOL ePOS_ActiveCurrentMode(const WORD NodeId);
    BOOL ePOS_GetCurrentMust(const WORD NodeId, short &pCurrentMust);
    BOOL ePOS_SetCurrentMust(const WORD NodeId, const short CurrentMust);

    // Interface function
    OperationMode ePOS_ReadOperationInfo(const WORD NodeId);
    MotorState ePOS_ReadMotorState(const WORD NodeId);
    char* ePOS_ReadErrorMsg();
    BOOL ePOS_CloseSystem();            // similar to destructor
    BOOL ePOS_QuickStop();

private:
    HANDLE _keyHandle;                  // Handle of Epos2
    DWORD _errorCode;                   // Message of Error

    BYTE _errorNumber;                  // the number of Error
    const WORD _infoSzie = 1000;        // size of buffer
    char* _errorInfo;                   // Information of Error
    const DWORD _baudrate = 1000000;
    const DWORD _timeout = 500;

    // 为私有数据提供了公有访问接口
    QMap<WORD,enum OperationMode> _operationMap;
    QMap<WORD,enum MotorState> _stateMap;

signals:

public slots:
};



class MotorDevice : public QObject
{
    Q_OBJECT
public:
    explicit MotorDevice(QObject *parent = nullptr);
    QMotor* getQMotorObject();
private:
    QMotor* _qmotor;
};

#endif // QMOTOR_H
