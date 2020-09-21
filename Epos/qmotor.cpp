#include "qmotor.h"
#include <QDebug>

QMotor::QMotor(QObject *parent) : QObject(parent)
{
    // initialize state map and operation map
    _stateMap = {
        {1,Disconnect},
        {2,Disconnect},
        {3,Disconnect},
        {4,Disconnect},
        {5,Disconnect},
        {6,Disconnect}
    };
    _operationMap = {
        {1,Default},
        {2,Default},
        {3,Default},
        {4,Default},
        {5,Default},
        {6,Default}
    };

    _keyHandle = nullptr;
    _errorNumber = 0;
    _errorCode = 0;

    _errorInfo = new char[_infoSzie];
    memset(_errorInfo,0,_infoSzie);
}

BOOL QMotor::ePOS_OpenDeviceDlg()
{
    _keyHandle=VCS_OpenDeviceDlg(&_errorCode);

    if(!_keyHandle)
    {
        return FALSE;
    }

    for(auto iter = _stateMap.begin();iter != _stateMap.end(); ++iter)
    {
        iter.value() = Connect;
    }
    return TRUE;
}

BOOL QMotor::ePOS_OpendeviceByUSB()
{
    const WORD maxStrSize = 100;

    // opendevice() need four key parameters
    char DeviceNameSel[] = "EPOS2";                       // Device Name:             EPOS, EOPS2, EPOS4
    char ProtocolStackNameSel[]="MAXON SERIAL V2";        // Protocal stack name:     MAXON_RS232, MAXON SERIAL V2, CANopen
    char InterfaceNameSel[maxStrSize];                    // Interface name serial    USN, CAN, RS232
    char PortNameSel[maxStrSize];                         // Port name:               COM1, COM2, USB0, USB1, CAN0, CAN1

    BOOL endOfSel = FALSE;

    qDebug()<<"Device Name: "<<reinterpret_cast<char *>(DeviceNameSel);
    qDebug()<<"Protocol Stack Name: "<<reinterpret_cast<char *>(ProtocolStackNameSel);

    // get interface name serial
    if(VCS_GetInterfaceNameSelection(reinterpret_cast<char*>(DeviceNameSel),reinterpret_cast<char*>(ProtocolStackNameSel),
                                     TRUE,reinterpret_cast<char*>(InterfaceNameSel),maxStrSize,&endOfSel,&_errorCode))
    {
        while (!endOfSel) {
            VCS_GetInterfaceNameSelection(reinterpret_cast<char*>(DeviceNameSel),reinterpret_cast<char*>(ProtocolStackNameSel),
                                          FALSE,reinterpret_cast<char*>(InterfaceNameSel),maxStrSize,&endOfSel,&_errorCode);
        }
    }

    qDebug()<<"Interface Name: "<<reinterpret_cast<char*>(InterfaceNameSel);

    // get port name selection
    endOfSel = FALSE;
    if(VCS_GetPortNameSelection(reinterpret_cast<char*>(DeviceNameSel),reinterpret_cast<char*>(ProtocolStackNameSel),
                                reinterpret_cast<char*>(InterfaceNameSel),TRUE,reinterpret_cast<char*>(PortNameSel),maxStrSize,&endOfSel,&_errorCode))
    {
        while (!endOfSel) {
            VCS_GetPortNameSelection(reinterpret_cast<char*>(DeviceNameSel),reinterpret_cast<char*>(ProtocolStackNameSel),
                                     reinterpret_cast<char*>(InterfaceNameSel),FALSE,reinterpret_cast<char*>(PortNameSel),maxStrSize,&endOfSel,&_errorCode);
        }
    }

    qDebug()<<"PortName: "<<reinterpret_cast<char*>(PortNameSel);

    _keyHandle = VCS_OpenDevice(reinterpret_cast<char*>(DeviceNameSel),reinterpret_cast<char*>(ProtocolStackNameSel),
                                reinterpret_cast<char*>(InterfaceNameSel),reinterpret_cast<char*>(PortNameSel),&_errorCode);

    qDebug()<<"Handle: "<<_keyHandle;

    // check m_KeyHandle is valid or not
    if(!_keyHandle)
    {
        return FALSE;
    }
    // set protocol stack
    if(VCS_SetProtocolStackSettings(_keyHandle,_baudrate,_timeout,&_errorCode))
    {
        for(auto iter = _stateMap.begin();iter != _stateMap.end(); ++iter)
        {
            iter.value() = Connect;
        }
    }
    return TRUE;
}

BOOL QMotor::ePOS_OpendeviceByCAN()
{
    const WORD maxStrSize = 100;

    // opendevice() need four key parameters
    char DeviceNameSel[] = "EPOS2";                         // Device Name:             EPOS, EOPS2, EPOS4
    char ProtocolStackNameSel[]="CANopen";                  // Protocal stack name:     MAXON_RS232, MAXON SERIAL V2, CANopen
    char InterfaceNameSel[maxStrSize];                      // Interface name serial    USN, CAN, RS232
    char PortNameSel[maxStrSize];                           // Port name:               COM1, COM2, USB0, USB1, CAN0, CAN1

    BOOL endOfSel = FALSE;

    qDebug()<<"Device Name: "<<reinterpret_cast<char *>(DeviceNameSel);
    qDebug()<<"Protocol Stack Name: "<<reinterpret_cast<char *>(ProtocolStackNameSel);

    // get interface name serial
    if(VCS_GetInterfaceNameSelection(reinterpret_cast<char*>(DeviceNameSel),reinterpret_cast<char*>(ProtocolStackNameSel),
                                     TRUE,reinterpret_cast<char*>(InterfaceNameSel),maxStrSize,&endOfSel,&_errorCode))
    {
        while (!endOfSel) {
            VCS_GetInterfaceNameSelection(reinterpret_cast<char*>(DeviceNameSel),reinterpret_cast<char*>(ProtocolStackNameSel),
                                          FALSE,reinterpret_cast<char*>(InterfaceNameSel),maxStrSize,&endOfSel,&_errorCode);
        }
    }

    qDebug()<<"Interface Name: "<<reinterpret_cast<char*>(InterfaceNameSel);

    // get port name selection
    endOfSel = FALSE;
    if(VCS_GetPortNameSelection(reinterpret_cast<char*>(DeviceNameSel),reinterpret_cast<char*>(ProtocolStackNameSel),
                                reinterpret_cast<char*>(InterfaceNameSel),TRUE,reinterpret_cast<char*>(PortNameSel),maxStrSize,&endOfSel,&_errorCode))
    {
        while (!endOfSel) {
            VCS_GetPortNameSelection(reinterpret_cast<char*>(DeviceNameSel),reinterpret_cast<char*>(ProtocolStackNameSel),
                                     reinterpret_cast<char*>(InterfaceNameSel),FALSE,reinterpret_cast<char*>(PortNameSel),maxStrSize,&endOfSel,&_errorCode);
        }
    }

    qDebug()<<"PortName: "<<reinterpret_cast<char*>(PortNameSel);

    _keyHandle = VCS_OpenDevice(reinterpret_cast<char*>(DeviceNameSel),reinterpret_cast<char*>(ProtocolStackNameSel),
                                reinterpret_cast<char*>(InterfaceNameSel),reinterpret_cast<char*>(PortNameSel),&_errorCode);

    qDebug()<<"Handle: "<<_keyHandle;

    // check m_KeyHandle is valid or not
    if(!_keyHandle)
    {
        return FALSE;
    }
    // set protocol stack
    if(VCS_SetProtocolStackSettings(_keyHandle,_baudrate,_timeout,&_errorCode))
    {
        for(auto iter = _stateMap.begin();iter != _stateMap.end(); ++iter)
        {
            iter.value() = Connect;
        }

    }
    return TRUE;
}

BOOL QMotor::ePOS_CloseAllDevice()
{

    if(!VCS_CloseAllDevices(&_errorCode))
    {
        return FALSE;
    }

    for(auto iter = _stateMap.begin();iter!= _stateMap.end();++iter)
    {
        iter.value() = Disconnect;
    }
    return TRUE;
}

BOOL QMotor::ePOS_SetEnableState(const WORD NodeId)
{
    if(!VCS_SetEnableState(_keyHandle,NodeId,&_errorCode))
    {
        return FALSE;
    }
    _stateMap[NodeId] = Enable;
    return TRUE;

}

BOOL QMotor::ePOS_GetEnableState(const WORD NodeId)
{
    BOOL pIsEnabled = FALSE;
    if(!VCS_GetEnableState(_keyHandle,NodeId,&pIsEnabled,&_errorCode))
    {
        return FALSE;
    }
    if(pIsEnabled)
    {
        _stateMap[NodeId] = Enable;
    }
    return TRUE;
}

BOOL QMotor::ePOS_SetDisableState(const WORD NodeId)
{
    if(!VCS_SetDisableState(_keyHandle,NodeId,&_errorCode))
    {
        return FALSE;
    }
    _stateMap[NodeId] = Disable;
    return TRUE;

}

BOOL QMotor::ePOS_SetQuickStopState(const WORD NodeId)
{
    if(!VCS_SetQuickStopState(_keyHandle,NodeId,&_errorCode))
    {
        return FALSE;

    }
    _stateMap[NodeId] = Quickstop;
    return TRUE;

}

BOOL QMotor::ePOS_GetQuickStopState(const WORD NodeId)
{
    BOOL pIsQuickStopped = FALSE;
    if(!VCS_GetQuickStopState(_keyHandle,NodeId,&pIsQuickStopped,&_errorCode))
    {
        return FALSE;
    }
    if(pIsQuickStopped)
    {
        _stateMap[NodeId] = Quickstop;
    }
    return TRUE;

}

BOOL QMotor::ePOS_GetDiableState(const WORD NodeId)
{
    BOOL pIsDisableFlag = FALSE;
    if(!VCS_GetDisableState(_keyHandle,NodeId,&pIsDisableFlag,&_errorCode))
    {
        return FALSE;
    }
    if(pIsDisableFlag)
    {
        _stateMap[NodeId] = Disable;
    }
    return TRUE;
}

BOOL QMotor::ePOS_ClearFault(const WORD NodeId)
{
    if(!VCS_ClearFault(_keyHandle,NodeId,&_errorCode))
    {
        return FALSE;

    }
    if(_stateMap[NodeId] == Fault)
    {
        _stateMap[NodeId] = Connect;
    }
    return TRUE;

}

BOOL QMotor::ePOS_GetFaultState(const WORD NodeId)
{
    BOOL pIsInFault = FALSE;
    if(!VCS_GetFaultState(_keyHandle,NodeId,&pIsInFault,&_errorCode))
    {
        return FALSE;
    }
    if(pIsInFault)
    {
        _stateMap[NodeId] = Fault;
    }
    return TRUE;

}

BOOL QMotor::ePOS_GetNbOfDeviceError(const WORD NodeId)
{
    if(!VCS_GetNbOfDeviceError(_keyHandle,NodeId,&_errorNumber,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetErrorInfo()
{
    if(!VCS_GetErrorInfo(_errorCode,_errorInfo,_infoSzie))
    {
        return  FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_SetOperationMode(const WORD NodeId, const OperationMode operationMode)
{
    if(!VCS_SetOperationMode(_keyHandle,NodeId,static_cast<char>(operationMode),&_errorCode))
    {
        return FALSE;
    }
    _operationMap[NodeId] = operationMode;
    return TRUE;
}


BOOL QMotor::ePOS_GetOperationMode(const WORD NodeId)
{
    char pMode;
    if(!VCS_GetOperationMode(_keyHandle,NodeId,&pMode,&_errorCode))
    {
        return FALSE;
    }
    switch (pMode) {
    case 1:
        _operationMap[NodeId] = PPM;
        break;
    case 3:
        _operationMap[NodeId] = PVM;
        break;
    case 6:
        _operationMap[NodeId] = HM;
        break;
    case 7:
        _operationMap[NodeId] = IPM;
        break;
    case -1:
        _operationMap[NodeId] = PM;
        break;
    case -2:
        _operationMap[NodeId] = PM;
        break;
    case -3:
        _operationMap[NodeId] = CM;
        break;
    case -5:
        _operationMap[NodeId] = MEM;
        break;
    case -6:
        _operationMap[NodeId] = SDM;
        break;
    case 8:
        _operationMap[NodeId] = CSP;
        break;
    case 9:
        _operationMap[NodeId] = CSV;
        break;
    case 10:
        _operationMap[NodeId] = CST;
        break;
    default:
        break;
    }
    return TRUE;
}


BOOL QMotor::ePOS_SetMaxProfileVelocity(const WORD NodeId, const DWORD MaxProfileVelocity)
{
    if(!VCS_SetMaxProfileVelocity(_keyHandle, NodeId, MaxProfileVelocity, &_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetMaxProfileVeloctiy(const WORD NodeId, DWORD &pMaxProfileVelocity)
{
    if(!VCS_GetMaxProfileVelocity(_keyHandle, NodeId, &pMaxProfileVelocity, &_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePos_GetMasterEncoderParameter(const WORD NodeId, WORD &pScalingNumerator,
                                            WORD &pScalingDenominator, BYTE &pPolarity,
                                            DWORD &pMaxVelocity, DWORD &pMaxAcceleration)
{
    if(!VCS_GetMasterEncoderParameter(_keyHandle,NodeId,&pScalingNumerator,&pScalingDenominator,
                                      &pPolarity,&pMaxVelocity,&pMaxAcceleration,&_errorCode))
    {
        return FALSE;
    }
    _operationMap[NodeId] = MEM;
    return TRUE;

}

BOOL QMotor::ePOS_GetOperationMode(const WORD NodeId, char &pOperationMode)
{
    if(!VCS_GetOperationMode(_keyHandle,NodeId,&pOperationMode,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetState(const WORD NodeId)
{
    WORD pState;
    if(!VCS_GetState(_keyHandle,NodeId,&pState,&_errorCode))
    {
        return FALSE;
    }
    switch (pState) {
    case 0:
        _stateMap[NodeId] = Disable;
        break;
    case 1:
        _stateMap[NodeId] = Enable;
        break;
    case 2:
        _stateMap[NodeId] = Quickstop;
        break;
    case 3:
        _stateMap[NodeId] = Fault;
        break;
    default:
        _stateMap[NodeId] = Connect;
        break;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetMovementState(const WORD NodeId)
{
    BOOL pTargetReach;
    if(!VCS_GetMovementState(_keyHandle,NodeId,&pTargetReach,&_errorCode))
    {
        return FALSE;
    }
    if(!pTargetReach)
    {
        return  FALSE;
    }
    return TRUE;

}

BOOL QMotor::ePOS_WaitForTargetReached(const WORD NodeId, const DWORD Timeout)
{
    if(!VCS_WaitForTargetReached(_keyHandle,NodeId,Timeout,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}


BOOL QMotor::ePOS_GetPositionIs(const WORD NodeId, long &pPositionIs)
{
    if(!VCS_GetPositionIs(_keyHandle,NodeId,&pPositionIs,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetVelocityIsAveraged(const WORD NodeId, double &pVelocityIsAveraged, const int jingdu)
{
    long tmp_velocity = 0;
    if(!VCS_GetVelocityIsAveraged(_keyHandle,NodeId,&tmp_velocity,&_errorCode)){
        return FALSE;
    }

    switch (jingdu){
    case 0:{
        pVelocityIsAveraged = static_cast<double>( tmp_velocity);
        break;
    }
    case -1:{
        pVelocityIsAveraged = static_cast<double>( tmp_velocity) /10;
        break;
    }

    case -2:{
        pVelocityIsAveraged = static_cast<double>( tmp_velocity) /100;
        break;
    }
    case -3:{
        pVelocityIsAveraged = static_cast<double>( tmp_velocity) /1000;
        break;
    }
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetVelocityIs(const WORD NodeId, long &pVelocityIs)
{
    if(!VCS_GetVelocityIs(_keyHandle,NodeId,&pVelocityIs,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetActualCurrent(const WORD NodeId, short &pCurrentIs)
{
    if(!VCS_GetCurrentIs(_keyHandle,NodeId,&pCurrentIs,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetCurrentIsAveraged(const WORD NodeId, short &pCurrentIsAveraged)
{
    if(!VCS_GetCurrentIsAveraged(_keyHandle,NodeId,&pCurrentIsAveraged,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_ActiveProfilePositionMode(const WORD NodeId)
{
    if(!VCS_ActivateProfilePositionMode(_keyHandle,NodeId,&_errorCode))
    {
        return FALSE;
    }
    _operationMap[NodeId] = PPM;
    return TRUE;
}

BOOL QMotor::ePOS_SetPositionProfile(const WORD NodeId,const DWORD ProfileVelocity,
                                     const DWORD ProfileAcceleration,const DWORD ProfileDeceleration)
{
    if(!VCS_SetPositionProfile(_keyHandle,NodeId,ProfileVelocity,ProfileAcceleration,
                               ProfileDeceleration,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetPositionProfile(const WORD NodeId, DWORD &ProfileVelocity,
                                     DWORD &ProfileAcceleration, DWORD &ProfileDeceleration)
{
    if(!VCS_GetPositionProfile(_keyHandle,NodeId,&ProfileVelocity,&ProfileAcceleration,
                               &ProfileDeceleration,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}


BOOL QMotor::ePOS_MoveToPosition(const WORD NodeId, const long TargetPosition, const BOOL Absolute, const BOOL Immediately)
{
    if(!VCS_MoveToPosition(_keyHandle,NodeId,TargetPosition,Absolute,Immediately,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetTargetPosition(const WORD NodeId, long &pTargetPosition)
{
    if(!VCS_GetTargetPosition(_keyHandle,NodeId,&pTargetPosition,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_HaltPositionMovement(const WORD NodeId)
{
    if(!VCS_HaltPositionMovement(_keyHandle,NodeId,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_ActivePositionMode(const WORD NodeId)
{
    if(!VCS_ActivatePositionMode(_keyHandle, NodeId, &_errorCode))
    {
        return FALSE;
    }
    _operationMap[NodeId] = PM;
    return TRUE;
}

BOOL QMotor::ePOS_SetPositionMust(const WORD NodeId, const long PositionMust)
{
    if(!VCS_SetPositionMust(_keyHandle,NodeId,PositionMust,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetPositionMust(const WORD NodeId, long &PositionMust)
{
    if(!VCS_GetPositionMust(_keyHandle,NodeId,&PositionMust,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_ActiveProfileVelocityMode(const WORD NodeId)
{
    if(!VCS_ActivateProfileVelocityMode(_keyHandle,NodeId,&_errorCode))
    {
        return FALSE;
    }
    _operationMap[NodeId] = PVM;
    return TRUE;
}

BOOL QMotor::ePOS_SetVelocityProfile(const WORD NodeId, const DWORD ProfileAcceleration,
                                     const DWORD ProfileDeceleration)
{
    if(!VCS_SetVelocityProfile(_keyHandle,NodeId,ProfileAcceleration,
                               ProfileDeceleration,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetVelocityProfile(const WORD NodeId, DWORD &ProfileAcceleration,
                                     DWORD &ProfileDeceleration)
{
    if(!VCS_GetVelocityProfile(_keyHandle,NodeId,&ProfileAcceleration,
                               &ProfileDeceleration,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_MoveWithVelocity(const WORD NodeId, const long TargetVelocity, const int jindu )
{
    long velocity = 0;
    switch (jindu)
    {
    case 0:
    {
        velocity = TargetVelocity;
        break;
    }
    case -1:
    {
        velocity = TargetVelocity * 10;
        break;
    }
    case -2:
    {
        velocity = TargetVelocity * 100;
        break;
    }
    case -3:
    {
        velocity = TargetVelocity * 1000;
        break;
    }
    default:
        return FALSE;

    }
    if(!VCS_MoveWithVelocity(_keyHandle,NodeId,velocity,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetTargetVelocity(const WORD NodeId, long &TargetVelocity)
{
    if(!VCS_GetTargetVelocity(_keyHandle,NodeId,&TargetVelocity,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_HaltVelocityMovement(const WORD NodeId)
{
    if(!VCS_HaltVelocityMovement(_keyHandle,NodeId,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_ActiveVelocityMode(WORD NodeId)
{
    if(!VCS_ActivateVelocityMode(_keyHandle, NodeId, &_errorCode))
    {
        return FALSE;
    }
    _operationMap[NodeId] = VM;
    return TRUE;
}

BOOL QMotor::ePOS_SetVelocityMust(const WORD NodeId, const long VelocityMust, const int jingdu)
{
    long velocity = 0;
    switch (jingdu){
    case 0:{
        velocity = VelocityMust;
        break;
    }
    case -1:{
        velocity = VelocityMust * 10;
        break;
    }
    case -2:{
        velocity = VelocityMust * 100;
        break;
    }
    case -3:{
        velocity = VelocityMust * 1000;
        break;
    }
    default:
        return FALSE;
    }

    if(!VCS_SetVelocityMust(_keyHandle, NodeId, velocity, &_errorCode)){
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetVelocityMust(const WORD NodeId, long &pVelocityMust)
{
    if(!VCS_GetVelocityMust(_keyHandle,NodeId,&pVelocityMust,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_ActiveCurrentMode(const WORD NodeId)
{
    if(!VCS_ActivateCurrentMode(_keyHandle, NodeId, &_errorCode))
    {
        return FALSE;
    }
    _operationMap[NodeId] = CM;
    return TRUE;
}

BOOL QMotor::ePOS_SetCurrentMust(const WORD NodeId, const short CurrentMust)
{
    if(!VCS_SetCurrentMust(_keyHandle, NodeId, CurrentMust, &_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetCurrentMust(const WORD NodeId, short &pCurrentMust)
{
    if(!VCS_GetCurrentMust(_keyHandle,NodeId,&pCurrentMust,&_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

OperationMode QMotor::ePOS_ReadOperationInfo(const WORD NodeId)
{
    ePOS_GetOperationMode(NodeId);
    return _operationMap[NodeId];
}

MotorState QMotor::ePOS_ReadMotorState(const WORD NodeId)
{
    ePOS_GetState(NodeId);
    return  _stateMap[NodeId];
}


char *QMotor::ePOS_ReadErrorMsg()
{
    ePOS_GetErrorInfo();
    return _errorInfo;
}

BOOL QMotor::ePOS_CloseSystem()
{
    if(!ePOS_CloseAllDevice())
    {
        return FALSE;
    }
    for (auto iter = _operationMap.begin();iter!=_operationMap.end();++iter)
    {
        iter.value() = Default;
    }
    return TRUE;
}

BOOL QMotor::ePOS_QuickStop()
{
    for(WORD i = 1; i <= 6; i++)
    {
        if(!ePOS_SetQuickStopState(i))
        {
            return FALSE;
        }
    }
    return TRUE;
}

BOOL QMotor::ePOS_SetVelocityUnits(WORD NodeId, char VelNotation)
{
    BYTE VelDimension = 0xA4;
    if(!VCS_SetVelocityUnits(_keyHandle, NodeId, VelDimension, VelNotation, &_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_GetVelocityUnits(WORD NodeId, char& pVelNotation)
{
    // 测试
    BYTE VelDimesnsion = 0xA4;
    if(!VCS_GetVelocityUnits(_keyHandle, NodeId, &VelDimesnsion, &pVelNotation, &_errorCode))
    {
        return FALSE;
    }
    return TRUE;
}

BOOL QMotor::ePOS_SetMilliVelocityUnits()
{
    for(WORD i = 1; i <= 6; i++)
    {
        if(!ePOS_SetVelocityUnits(i, VN_MILLI))
        {
            return FALSE;
        }
    }
    return TRUE;
}





//------------------------------------------------------------------------------

MotorDevice::MotorDevice(QObject *parent):QObject (parent)
{
    _qmotor = new  QMotor;
}

QMotor *MotorDevice::getQMotorObject()
{
    return _qmotor;
}
