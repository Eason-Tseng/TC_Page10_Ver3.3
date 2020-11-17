//---------------------------------------------------------------------------
#include "SMEM.h"
#include "WRITEJOB.h"
#include "CDataToMessageOK.h"
#include "CTIMER.h"

#include "CSTC.h"

#include "DIGITALIO.h"

#include "CSTORINI.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>


SMEM smem;
//---------------------------------------------------------------------------
SMEM::SMEM(void)
{
try {

    pthread_mutex_init(&mutexSmem,NULL);

    InitialSMem();

    bConnectWithCenter = true;
    bConnectWithCenterStatus = true;

    for(int i = 0 ; i <MSGWAITREPONSECOUNT; i++)
      msgWaitResponse[i].sequence = INVAILDVALUE;

    localIp1_1 = 192;
    localIp1_2 = 168;
    localIp1_3 = 1;
    localIp1_4 = 101;
    localPort1 = 6003;
    //\uFFFD\uFFFD��192.168.1.101:6003
    distIp0_1 = 192;
    distIp0_2 = 168;
    distIp0_3 = 1;
    distIp0_4 = 102;
    dist0Port = 6003;
    //\uFFFD\uFFFD��192.168.1.102:6003
    distIp1 = 192;
    distIp2 = 168;
    distIp3 = 1;
    distIp4 = 1;
    distPort= 5003;
    //\uFFFD\uFFFD��192.168.1.1:5003

    //OT20111020
    netmask1 = 255;
    netmask2 = 255;
    netmask3 = 255;
    netmask4 = 0;
    gateway1 = 1;
    gateway2 = 2;
    gateway3 = 3;
    gateway4 = 254;

    siTransmitType[0] = cSTOPSENDTIME;                                                      //In VD TransmitType[0] = 0; //page 6-42
    siTransmitType[1] = 5;                                                          //5sec�A�s����3
    siTransmitType[2] = cSTOPSENDTIME;
    siTransmitType[3] = cSTOPSENDTIME;
    siTransmitType[4] = cSTOPSENDTIME;

    lLastRebootSec = 0;

//chain
    ucTC_TrainChainEnable = 0;
    ucTrainComingBanSubphase = 1;
    ucTrainComingForceJumpSubphase = 0;
//    ucTrainComingBanSubphaseLast = 0;


//92COMM
    ucCommandSet = 0x03;
    for (int i=0;i<6;i++) password[i]='0';  //�Ȧsfrom����code
    operMode = 0;
    HWCycleCode = 0x05;

    ucBootingDisplayRedcount = 1;  //Default to display.
    printf("ucBootingDisplayRedcountA:%d\n", ucBootingDisplayRedcount);
    vRead92COMMFromDisk();

//OTCombo

    ucTC_ActuateTypeByTOD = 0;  //default Actuate will set change to PlanID

    printf("ucTC_ActuateTypeByTOD1:%d\n", ucTC_ActuateTypeByTOD);

    vRead92TCSettingFromDisk();

    printf("ucTC_ActuateTypeByTOD2:%d\n", ucTC_ActuateTypeByTOD);

    startTime=time(NULL);

    vReadCCJDynCtlFromStorage();
    vReadCCJDynCtl2FromStorage();

    /*fuck define*/
    //Fuck Force Define.
//    ucReverseLaneManualEnable = 1;                                              //com1 will change to 81CtrlBoard.
    ucReverseLaneControlMode = 0;                                               //auto.

    printf("ucSmem92TC_ControlStrategy:%d\n", ucSmem92TC_ControlStrategy);

    printf("ucBootingDisplayRedcountB:%d\n", ucBootingDisplayRedcount);
    if(ucBootingDisplayRedcount != 0) {                                        // 0: not display,
      ucDisplayRedcount = 1;
    } else {
      ucDisplayRedcount = 0;
    }

    bEnableUpdateRTC = false;
    bGetResetTCSignal = false;
    bTCSegmentTypeUpdate = false;
    ucCSTCControlStrategy = 10; //TOD

    // iFirmwareYear = 2017;
    // iFirmwareMonth = 03;
    // iFirmwareDay = 23;
    iFirmwareFirstVersion = 3;  //韌體版本
    iFirmwareSecondVersion = 3;
    iFirmwareYear = 2020;  //年
    iFirmwareMonth = 11;  //月
    iFirmwareDay = 17;  //日

    //20140415, add dynamic seg for HsinChu dynamic control.
    //20140211, add special version for SIPA ped.
    //20131225, add CCJ heart beat check.
    //20131210, add kalkin protocol 5F83
    //20131129, add [CCJ] 5F9013, new byte for  TC_CSTC_ControlStrategy
    //20131121, add [CCJ] 5F90 msg for heartbeat
    //20121217, add [rev] printf msg for debug
    //20111201, add railway subphaseBan2
    //20111128, add change machine location function by ini, screenbug in act show every where
    //20111116, keypad & revLane add comm ok check (not check will crash in booting)
////OT20111104, smem.vSet5F18EffectTime(i5F10_EffectTime);  , 20111107
    //20111028, fix 5F18 protocol in 5f08report
    //20111026, disable 20110825 ask function
    //20111020, add screenNetmaskSet(class), change screenNetworkSet,screenNetworkSetConfirm, smem, change version from20110701 to 20111020
    //20111020, modify 5F1C, ctimer for dyn.
    //201110114, in CSTC, ReportCurrentOperationMode_5F08(), all type return 0x03
    //20110930, mask iAskW77E58CounterReturn too much for logging.
    //20110922, fix parseBlock overflow
    //20110921, change CCJ-5F91 be  5F9101/5F9102 for  Plan 5F18 change mode(one time/ one segment)
    //20110825, 0F04, CPU module set to lightBoard status, and fix bit6 change very fast.
    //20110729, CCJ 5F18 change condition to run
    //20110728, GPS                       if(smem.vGetCommEnable() == true) {
    //20110707, 0x80 will not change to TOD, redcount reset fix
    //20110706, refix 0x22(light), phase 0x80 as flash bug.
    //20110702, check file for reset enable, add file "/tmp/resetTCmyself.txt" will reset tc in last phase cycle
    //20110701, force check 0x80 is flash and fix it.
    //20110629 force check 0x80 is flash and fix it.
    //100,6,24 redcount repeat counte changeable by .ini
    //100,6,14 fix parseing 92 protocol
    //100,6,9 plan ID (5FB0),
    //100,5,17-> ?
    //100,4,29-> edit light, let flash light not in the same time
    //100,3,10-> add about 5F 10 02 setting
    //100,2,19-> keyword OT1000218, change 5F18, 5F10, 5F1C for normal server mode.
    //100,2,11-> commEnable after tc creating TC
    // ssPort disable

  //980623
  //990726 -> redcount?
    iFirmwareCompanyID = 0x00;

//here select ICOP & ARBOR
//     iFirmwareFirstVersion = 3;
// //    iFirmwareFirstVersion = 6;
//     iFirmwareSecondVersion = ucMBType;

/*
enum ControlStrategy{
  STRATEGY_TOD       =10,
  STRATEGY_AUTO_CADC =40,  //STRATEGY_AUTO_CADC is triggered from TOD
  STRATEGY_CADC      =50,  //STRATEGY_CADC is triggered from outter
  STRATEGY_MANUAL    =70,
  STRATEGY_FLASH     =80,
  STRATEGY_ALLRED    =90,
  STRATEGY_ALLDYNAMIC =95
};
*/

    printf("SMEM Init. OK!\n");

  } catch (...) {}
}
//---------------------------------------------------------------------------
SMEM::~SMEM(void)
{
}
//---------------------------------------------------------------------------
void SMEM::InitialSMem(void)
{
try {
    pthread_mutex_lock(&mutexSmem);

    bCommEnableSwitch = false;
    ui5F18EffectTime = 0;
    iSmem92TC_ChangePlanOneTime5F18_PlanID = 255; //default

    cFace=cLOGO;                                                                //�ťխ���
    lastFace=cMAIN;
    address=0;
    operMode=99;                                                                //�̪�S����,����L��J��~���Ȫ��N�q
    dbOperStat=0;

    iAdjudicateReturnAddressBCDorHEX = cHEX;

    ssComm=false;

    keypadReturn=false;
    lastKeypadTime=time(NULL);


    for (int i=0;i<6;i++) password[i]='0';  //�Ȧs

    iSmem92TC_PhaseUpdateFlag = 0;                                              //��s��flag
    bSmem92TC_TransmitCycle5F0F_ChangeFlag = false;
    bSmem92TC_TransmitCycle5F03_ChangeFlag = false;

    iSmem92TC_SignalLightStatus_5F0F_IntervalTime = 5;
    iSmem92TC_SignalStepStatus_5F03_IntervalTime = 5;

    ucSmem92TC_ControlStrategy = 0x01;
    iSmem92TC_EffectTime = 3;
    bSmem92TC_ChangePlanOneTime5F18_ChangeFlag = false;

    //OTADD
    for(int i = 0; i < 255; i++) {                                                //�����]������Ƴ��S��
      bSmemTC_PhaseAlive[i] = false;
      bSmemTC_PlanAlive[i] = false;
      bSmemTC_SegTypeAlive[i] = false;
    }

    bTCDoorStatus = false;
    bHaveTCDoorSwitch = false;
    bTCSignalConflictError = false;
    bSmem92TC_SubPhaseOfPhasePlanIncorrent = false;

    long lTmpTime = 0;                                                          //�̫᭫�}���ɶ�
    disk.vReadLastAliveTimeStampFile(&lTmpTime);
    lTmpTime;

    pthread_mutex_unlock(&mutexSmem);

    smem.vSetLastResetTime(lTmpTime);                                           //�⭫�}���ɶ����smem��

    vReadUDPMappingLCNDataFromStorage();                                        //ŪIP���

//OT Debug 951121
    vReadLCNFromDisk();
    address = uiLCNUsingStorage;

    iSmemTC_RedCountVer = TC_RedCountVerCCT97;                                  //default setting to newest version, 20110729

    for(int i = 0; i < 8; i ++)
      usiWayMappingRedCount[i] = i;

      //OT Debug Direct
    usiSignamMapMappingDir[dirN] = lightOut1;
    usiSignamMapMappingDir[dirE] = lightOut2;
    usiSignamMapMappingDir[dirS] = lightOut3;
    usiSignamMapMappingDir[dirW] = lightOut4;
    usiSignamMapMappingDir[dirNE] = lightOut5;
    usiSignamMapMappingDir[dirES] = lightOut6;
    usiSignamMapMappingDir[dirSW] = lightOut7;
    usiSignamMapMappingDir[dirWN] = lightOut8;
    usiSignamMapMappingLightBoard[0] = 0;
    usiSignamMapMappingLightBoard[1] = 1;
    usiSignamMapMappingLightBoard[2] = 2;
    usiSignamMapMappingLightBoard[3] = 3;
    usiSignamMapMappingLightBoard[4] = 4;
    usiSignamMapMappingLightBoard[5] = 5;

//    iSmem_Com2Type = Com2IsTesterPort;
    iSmem_Com2Type = Com2IsGPSPort;                                             //�g����
//    iSmem_Com2Type = Com2IsPassingPort;                                     //�g����
//    iSmem_Com2Type = Com2IsRedCount;
//    iSmem_Com2Type = Com2IsTainanPeopleLight;

    disk.vReadMotherBoardTypeFromFile(&ucMBType);
    disk.vReadCom2TypeFromFile(&iSmem_Com2Type);

    bSmemTC_PlanAlive[0x80] = true;
    bSmemTC_PlanAlive[0xFF] = true;

    for(int i = 0; i < 14; i++) {
      ucSignalLightStatus[i] = 0x0;
    }

//OT Debug 0523
    bTC_ActuateTypeFunctionEnable = true;
    bSmemTC_CCT_In_LongTanu_ActuateType_Switch = false;                                 //�s��S�OĲ��, 1 cycle change execplan
    usiSmemTC_CCT_In_LongTanu_ActuateType_PlanID = 1;                               //when actuating, change to this plan

    //OT Pass
    for(int i = 0; i < 256; i++) {
      ucACKW2W[i] = 0;
    }
    ucPassMode_0F8E8F = 0;
    usiPassServerLCN = 0;                                                    //for test

    usiScreenCopyPlanID = INVAILDVALUE;
    usiScreenCopySegID = INVAILDVALUE;

    vLoadLast92TC_5F15Time();

//OT20110825, default set to false
    bSignalDriverStatus = false;

    /* chain */
    vClearChildChainStruct();

    ucTC92_5F31Manual = 0;
    ucTC92_5F31TOD = 0;
    ucTC92_5F31StartSubPhaseId = 0;
    ucTC92_5F31EndSubPhaseId = 0;
    ucTC92_5F32StartSubPhaseId = 0;
    ucTC92_5F32EndSubPhaseId = 0;
    ucTC_MotherChainStartStepId = 0;
    ucTC_MotherChainEndStepId = 0;
    for(int i = 0; i < 64; i++) {
      siTC92_5F33StartOffset[i] = 0;
      siTC92_5F33EndOffset[i] = 0;
    }
    smem.vReadChainDataFromStroage();

    bGPSSyncStatus = false;                                                     //when booting, sync time by GPS

    b92VDUpdateDB_b0 = false;  //For TC
    b92VDUpdateDB_b1 = false;
    b92VDUpdateDB_b2 = false;
    b92VDUpdateDB_b3 = false;

    bEnableSendStepToCCJ = false;

    bNextStepForceEnable = false;

    ucSignalConflictErrorVar[0] = 0xFF;
    ucSignalConflictErrorVar[1] = 0xFF;
    ucSignalConflictErrorVar[2] = 0xFF;

    uiTrafficLightSend = 0;
    uiTrafficLightACK = 0;

//Fuck Force Define.
//    ucReverseLaneManualEnable = 1;                                              //com1 will change to 81CtrlBoard.
    ucReverseLaneControlMode = 0;                                               //auto.

    printf("ucReverseLaneManualEnable:%d\n", ucReverseLaneManualEnable);

    ucW77E58FWVer[0] = 0; //Year
    ucW77E58FWVer[1] = 0; //Month
    ucW77E58FWVer[2] = 0; //Day

    bForceClockRedCountOneTime = false;
    bBugStopSend0x22ForceCtl51Light = false;

//OT990419
    ucSmemTC_DynShowRedCountForAct = 0;

    ucSendCCTActArwenProtocal = 0;

    _NewestRevSyncStatus.bYMD_OK = false;
    _NewestRevSyncStatus.bTime_OK = false;
    _NewestRevSyncStatus.bRevStep_OK = false;
    _NewestRevSyncStatus.bRevSegSet_OK = false;
    _NewestRevSyncStatus.bRevTime_OK = false;

    iRevSyncStatus = 0; //unknow.
    ucRevSyncByteStatus = 0;

    //OT20110825
    lastGetProtocolTime = 0;
    _lastGetTrafficLightTime = 0;
    bTrafficeLightBoardConnectStatus = false;

    ucMachineLocation = 0;

    //OT20131225
    usiCCJHeartBeatCount = 0;

    //OT20140211
    for(int i = 0; i < 50; i++) {
      bPlanForceTOD[i] = false;
    }

    //OT20140415
    bDynSegSwitch = false;
    ucDynSegAdjustType = 0;
    usiDynSegAdjustSec = 0;
    bDynSegStatus = 0;
    usiDynSegRemainSec = 0;


/*
    usiVDPhaseMap[0][1] = oStorINI.vLoadIntFromINI("VD", "Phase0_1", 0);
    usiVDPhaseMap[0][2] = oStorINI.vLoadIntFromINI("VD", "Phase0_2", 0);
    usiVDPhaseMap[0][3] = oStorINI.vLoadIntFromINI("VD", "Phase0_3", 0);
    usiVDPhaseMap[0][4] = oStorINI.vLoadIntFromINI("VD", "Phase0_4", 0);
    usiVDPhaseMap[0][5] = oStorINI.vLoadIntFromINI("VD", "Phase0_5", 0);
    usiVDPhaseMap[0][6] = oStorINI.vLoadIntFromINI("VD", "Phase0_6", 0);
    usiVDPhaseMap[0][7] = oStorINI.vLoadIntFromINI("VD", "Phase0_7", 0);
    usiVDPhaseMap[1][0] = oStorINI.vLoadIntFromINI("VD", "Phase1_0", 0);
    usiVDPhaseMap[1][1] = oStorINI.vLoadIntFromINI("VD", "Phase1_1", 0);
    usiVDPhaseMap[1][2] = oStorINI.vLoadIntFromINI("VD", "Phase1_2", 0);
    usiVDPhaseMap[1][3] = oStorINI.vLoadIntFromINI("VD", "Phase1_3", 0);
    usiVDPhaseMap[1][4] = oStorINI.vLoadIntFromINI("VD", "Phase1_4", 0);
    usiVDPhaseMap[1][5] = oStorINI.vLoadIntFromINI("VD", "Phase1_5", 0);
    usiVDPhaseMap[1][6] = oStorINI.vLoadIntFromINI("VD", "Phase1_6", 0);
    usiVDPhaseMap[1][7] = oStorINI.vLoadIntFromINI("VD", "Phase1_7", 0);
    usiVDPhaseMap[2][0] = oStorINI.vLoadIntFromINI("VD", "Phase2_0", 0);
    usiVDPhaseMap[2][1] = oStorINI.vLoadIntFromINI("VD", "Phase2_1", 0);
    usiVDPhaseMap[2][2] = oStorINI.vLoadIntFromINI("VD", "Phase2_2", 0);
    usiVDPhaseMap[2][3] = oStorINI.vLoadIntFromINI("VD", "Phase2_3", 0);
    usiVDPhaseMap[2][4] = oStorINI.vLoadIntFromINI("VD", "Phase2_4", 0);
    usiVDPhaseMap[2][5] = oStorINI.vLoadIntFromINI("VD", "Phase2_5", 0);
    usiVDPhaseMap[2][6] = oStorINI.vLoadIntFromINI("VD", "Phase2_6", 0);
    usiVDPhaseMap[2][7] = oStorINI.vLoadIntFromINI("VD", "Phase2_7", 0);
    usiVDPhaseMap[3][0] = oStorINI.vLoadIntFromINI("VD", "Phase3_0", 0);
    usiVDPhaseMap[3][1] = oStorINI.vLoadIntFromINI("VD", "Phase3_1", 0);
    usiVDPhaseMap[3][2] = oStorINI.vLoadIntFromINI("VD", "Phase3_2", 0);
    usiVDPhaseMap[3][3] = oStorINI.vLoadIntFromINI("VD", "Phase3_3", 0);
    usiVDPhaseMap[3][4] = oStorINI.vLoadIntFromINI("VD", "Phase3_4", 0);
    usiVDPhaseMap[3][5] = oStorINI.vLoadIntFromINI("VD", "Phase3_5", 0);
    usiVDPhaseMap[3][6] = oStorINI.vLoadIntFromINI("VD", "Phase3_6", 0);
    usiVDPhaseMap[3][7] = oStorINI.vLoadIntFromINI("VD", "Phase3_7", 0);
    usiVDPhaseMap[4][0] = oStorINI.vLoadIntFromINI("VD", "Phase4_0", 0);
    usiVDPhaseMap[4][1] = oStorINI.vLoadIntFromINI("VD", "Phase4_1", 0);
    usiVDPhaseMap[4][2] = oStorINI.vLoadIntFromINI("VD", "Phase4_2", 0);
    usiVDPhaseMap[4][3] = oStorINI.vLoadIntFromINI("VD", "Phase4_3", 0);
    usiVDPhaseMap[4][4] = oStorINI.vLoadIntFromINI("VD", "Phase4_4", 0);
    usiVDPhaseMap[4][5] = oStorINI.vLoadIntFromINI("VD", "Phase4_5", 0);
    usiVDPhaseMap[4][6] = oStorINI.vLoadIntFromINI("VD", "Phase4_6", 0);
    usiVDPhaseMap[4][7] = oStorINI.vLoadIntFromINI("VD", "Phase4_7", 0);
    usiVDPhaseMap[5][0] = oStorINI.vLoadIntFromINI("VD", "Phase5_0", 0);
    usiVDPhaseMap[5][1] = oStorINI.vLoadIntFromINI("VD", "Phase5_1", 0);
    usiVDPhaseMap[5][2] = oStorINI.vLoadIntFromINI("VD", "Phase5_2", 0);
    usiVDPhaseMap[5][3] = oStorINI.vLoadIntFromINI("VD", "Phase5_3", 0);
    usiVDPhaseMap[5][4] = oStorINI.vLoadIntFromINI("VD", "Phase5_4", 0);
    usiVDPhaseMap[5][5] = oStorINI.vLoadIntFromINI("VD", "Phase5_5", 0);
    usiVDPhaseMap[5][6] = oStorINI.vLoadIntFromINI("VD", "Phase5_6", 0);
    usiVDPhaseMap[5][7] = oStorINI.vLoadIntFromINI("VD", "Phase5_7", 0);
    usiVDPhaseMap[6][0] = oStorINI.vLoadIntFromINI("VD", "Phase6_0", 0);
    usiVDPhaseMap[6][1] = oStorINI.vLoadIntFromINI("VD", "Phase6_1", 0);
    usiVDPhaseMap[6][2] = oStorINI.vLoadIntFromINI("VD", "Phase6_2", 0);
    usiVDPhaseMap[6][3] = oStorINI.vLoadIntFromINI("VD", "Phase6_3", 0);
    usiVDPhaseMap[6][4] = oStorINI.vLoadIntFromINI("VD", "Phase6_4", 0);
    usiVDPhaseMap[6][5] = oStorINI.vLoadIntFromINI("VD", "Phase6_5", 0);
    usiVDPhaseMap[6][6] = oStorINI.vLoadIntFromINI("VD", "Phase6_6", 0);
    usiVDPhaseMap[6][7] = oStorINI.vLoadIntFromINI("VD", "Phase6_7", 0);
    usiVDPhaseMap[7][0] = oStorINI.vLoadIntFromINI("VD", "Phase7_0", 0);
    usiVDPhaseMap[7][1] = oStorINI.vLoadIntFromINI("VD", "Phase7_1", 0);
    usiVDPhaseMap[7][2] = oStorINI.vLoadIntFromINI("VD", "Phase7_2", 0);
    usiVDPhaseMap[7][3] = oStorINI.vLoadIntFromINI("VD", "Phase7_3", 0);
    usiVDPhaseMap[7][4] = oStorINI.vLoadIntFromINI("VD", "Phase7_4", 0);
    usiVDPhaseMap[7][5] = oStorINI.vLoadIntFromINI("VD", "Phase7_5", 0);
    usiVDPhaseMap[7][6] = oStorINI.vLoadIntFromINI("VD", "Phase7_6", 0);
    usiVDPhaseMap[7][7] = oStorINI.vLoadIntFromINI("VD", "Phase7_7", 0);

    ucVDPhaseTriggerSwitch[0] = oStorINI.vLoadIntFromINI("VDTrigger", "PhaseTriggerSwitch0", 0);
    ucVDPhaseTriggerSwitch[1] = oStorINI.vLoadIntFromINI("VDTrigger", "PhaseTriggerSwitch1", 0);
    ucVDPhaseTriggerSwitch[2] = oStorINI.vLoadIntFromINI("VDTrigger", "PhaseTriggerSwitch2", 0);
    ucVDPhaseTriggerSwitch[3] = oStorINI.vLoadIntFromINI("VDTrigger", "PhaseTriggerSwitch3", 0);
    ucVDPhaseTriggerSwitch[4] = oStorINI.vLoadIntFromINI("VDTrigger", "PhaseTriggerSwitch4", 0);
    ucVDPhaseTriggerSwitch[5] = oStorINI.vLoadIntFromINI("VDTrigger", "PhaseTriggerSwitch5", 0);
    ucVDPhaseTriggerSwitch[6] = oStorINI.vLoadIntFromINI("VDTrigger", "PhaseTriggerSwitch6", 0);
    ucVDPhaseTriggerSwitch[7] = oStorINI.vLoadIntFromINI("VDTrigger", "PhaseTriggerSwitch7", 0);
*/

    //for test
//    disk.vWriteCom2TypeFromFile(0);
//    disk.vWriteMotherBoardTypeFromFile(1);

  } catch (...) {}
}
//---------------------------------------------------------------------------
int SMEM::vGetAdjudicateReturnAddressBCDorHEX(void)
{
try {
  int iTMP=0;
  pthread_mutex_lock(&mutexSmem);
  iTMP=iAdjudicateReturnAddressBCDorHEX;
  pthread_mutex_unlock(&mutexSmem);
  return iTMP;
} catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vSetAdjudicateReturnAddressBCDorHEX(int iTMP)
{
try {
  pthread_mutex_lock(&mutexSmem);
  iAdjudicateReturnAddressBCDorHEX=iTMP;
  vSave92COMMToDisk();                                                          //SaveToDOM
  pthread_mutex_unlock(&mutexSmem);
  return true;
} catch (...) {}
}
//---------------------------------------------------------------------------
int SMEM::GetcFace(void)
{
try {
    int tempFace=0;
    pthread_mutex_lock(&mutexSmem);
    tempFace=cFace;
    pthread_mutex_unlock(&mutexSmem);
    return tempFace;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::SetcFace(int face)
{
try {
    pthread_mutex_lock(&mutexSmem);
    cFace=face;
    printf("[printfMsg] screen Enter face:%d\n", face);
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}
//---------------------------------------------------------------------------
int SMEM::GetLastFace(void)
{
try {
    int tempFace=0;
    pthread_mutex_lock(&mutexSmem);
    tempFace=lastFace;
    pthread_mutex_unlock(&mutexSmem);
    return tempFace;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::SetLastFace(int face)
{
try {
    pthread_mutex_lock(&mutexSmem);
    lastFace=face;
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}
//---------------------------------------------------------------------------
int SMEM::GetAddress(void)
{
try {
    int tempAddress=0;
    pthread_mutex_lock(&mutexSmem);
    tempAddress=address;
    pthread_mutex_unlock(&mutexSmem);
    return tempAddress;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::SetAddress(int lcn)
{
try {
    pthread_mutex_lock(&mutexSmem);
    if(bEnableLCNUsingStorage) address = uiLCNUsingStorage;                     //From DOM
    else address=lcn;                                                           //From Control panel
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}
//---------------------------------------------------------------------------
int SMEM::GetOperMode(void)
{
try {
    bool tempOpenMode;
    pthread_mutex_lock(&mutexSmem);
    tempOpenMode=operMode;
    pthread_mutex_unlock(&mutexSmem);
    return tempOpenMode;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::SetOperMode(int mode)
{
try {
    pthread_mutex_lock(&mutexSmem);
    operMode=mode;
    vSave92COMMToDisk();                                                        //SaveToDOM
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}
//---------------------------------------------------------------------------
int SMEM::GetSequence(void)
{
try {
    int tempSequence;
    pthread_mutex_lock(&mutexSmem);
    tempSequence=sequence;
    sequence++;                                          //�Q�H������y�����n�[�@
    if (sequence>=256) sequence=0;                       //�y�����u���@��BYTE,�ҥH�̦h��255
    pthread_mutex_unlock(&mutexSmem);
    return tempSequence;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::GetKeypadReturn(void)
{
try {
    bool tempKeypadReturn;
    pthread_mutex_lock(&mutexSmem);
    tempKeypadReturn=keypadReturn;
    pthread_mutex_unlock(&mutexSmem);
    return tempKeypadReturn;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::SetKeypadReturn(bool kr)
{
try {
    pthread_mutex_lock(&mutexSmem);
    keypadReturn=kr;
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}
//---------------------------------------------------------------------------
char * SMEM::GetPassword(void)
{
try {
    char tempPassword[6];
    pthread_mutex_lock(&mutexSmem);
    for (int i=0;i<6;i++)  tempPassword[i]=password[i];
    pthread_mutex_unlock(&mutexSmem);
//    return tempPassword;  //cause error?
    return password;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::SetPassword(char p1,char p2,char p3,char p4,char p5,char p6)
{
try {
    if ((p1=='0'||p1=='1'||p1=='2'||p1=='3'||p1=='4'||p1=='5'||p1=='6'||p1=='7'||p1=='8'||p1=='9'||p1=='A'||p1=='B'||p1=='C'||p1=='D'||p1=='E'||p1=='F') &&
        (p2=='0'||p2=='1'||p2=='2'||p2=='3'||p2=='4'||p2=='5'||p2=='6'||p2=='7'||p2=='8'||p2=='9'||p2=='A'||p2=='B'||p2=='C'||p2=='D'||p2=='E'||p2=='F') &&
        (p3=='0'||p3=='1'||p3=='2'||p3=='3'||p3=='4'||p3=='5'||p3=='6'||p3=='7'||p3=='8'||p3=='9'||p3=='A'||p3=='B'||p3=='C'||p3=='D'||p3=='E'||p3=='F') &&
        (p4=='0'||p4=='1'||p4=='2'||p4=='3'||p4=='4'||p4=='5'||p4=='6'||p4=='7'||p4=='8'||p4=='9'||p4=='A'||p4=='B'||p4=='C'||p4=='D'||p4=='E'||p4=='F') &&
        (p5=='0'||p5=='1'||p5=='2'||p5=='3'||p5=='4'||p5=='5'||p5=='6'||p5=='7'||p5=='8'||p5=='9'||p5=='A'||p5=='B'||p5=='C'||p5=='D'||p5=='E'||p5=='F') &&
        (p6=='0'||p6=='1'||p6=='2'||p6=='3'||p6=='4'||p6=='5'||p6=='6'||p6=='7'||p6=='8'||p6=='9'||p6=='A'||p6=='B'||p6=='C'||p6=='D'||p6=='E'||p6=='F'))
    {
      pthread_mutex_lock(&mutexSmem);
      password[0]=p1;
      password[1]=p2;
      password[2]=p3;
      password[3]=p4;
      password[4]=p5;
      password[5]=p6;
      vSave92COMMToDisk();                                                      //SaveToDOM
      pthread_mutex_unlock(&mutexSmem);
      return true;
    } else return false;
  } catch (...) {}
}
//---------------------------------------------------------------------------
int SMEM::GetDbOperStat(void)
{
try {
    int tempOperStat;
    pthread_mutex_lock(&mutexSmem);
    tempOperStat=dbOperStat;
    pthread_mutex_unlock(&mutexSmem);
    return tempOperStat;
  } catch(...) {}
}
//---------------------------------------------------------------------------
bool SMEM::SetDbOperStat(int dbOper)
{
try {
    pthread_mutex_lock(&mutexSmem);
    dbOperStat=dbOper;
    vSave92COMMToDisk();                                                        //SaveToDOM
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch(...) {}
}
//---------------------------------------------------------------------------
bool SMEM::GetSSComm(void)
{
try {
    bool tempSSComm;
    pthread_mutex_lock(&mutexSmem);
    tempSSComm=ssComm;
    pthread_mutex_unlock(&mutexSmem);
    return tempSSComm;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::SetSSComm(bool scomm)
{
try {
    pthread_mutex_lock(&mutexSmem);
    ssComm = scomm;
    ssCommTime = time(NULL);
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}
//---------------------------------------------------------------------------
time_t SMEM::vGetSSCommTime(void)
{
try {
    time_t _TMP;
    pthread_mutex_lock(&mutexSmem);
    _TMP = ssCommTime;
    pthread_mutex_unlock(&mutexSmem);
    return _TMP;
  } catch (...) {}
}
//---------------------------------------------------------------------------
time_t SMEM::GetLastKeypadTime(void)
{
try {
    time_t tempTimer;
    pthread_mutex_lock(&mutexSmem);
    tempTimer=lastKeypadTime;
    pthread_mutex_unlock(&mutexSmem);
    return tempTimer;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::SetLastKeypadTime(void)
{
try {
    pthread_mutex_lock(&mutexSmem);
    lastKeypadTime=time(NULL);
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}


//---------------------------------------------------------------------------
time_t SMEM::GetStartTime(void)
{
try {
    time_t tempTime;
    pthread_mutex_lock(&mutexSmem);
    tempTime=startTime;
    pthread_mutex_unlock(&mutexSmem);
    return tempTime;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::SetWaitMsg(int seq,BYTE * packet,int length)
{
try {
    bool alreadyWait=false;
    pthread_mutex_lock(&mutexSmem);
    for (int i=0;i<MSGWAITREPONSECOUNT;i++) {
         if (msgWaitResponse[i].sequence==seq)
             alreadyWait=true;
    }

    if (alreadyWait==false) {
        for (int i=0;i<MSGWAITREPONSECOUNT;i++) {
             if (msgWaitResponse[i].sequence==INVAILDVALUE) {
                 msgWaitResponse[i].sequence=seq;
                 for (int j=0;j<length;j++) msgWaitResponse[i].packet[j]=packet[j];
                 msgWaitResponse[i].length=length;
                 msgWaitResponse[i].retryCount=0;
                 msgWaitResponse[i].sendTime=time(NULL);
                 break;
             }
        }
    }
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::ClearWaitMsg(int seq)
{
try {
    MESSAGEOK _MSG;
    bool bOldStatus;
    pthread_mutex_lock(&mutexSmem);
    for (int i=0;i<MSGWAITREPONSECOUNT;i++) {
         if (msgWaitResponse[i].sequence==seq)
         {
           msgWaitResponse[i].sequence=INVAILDVALUE;
         }
    }
    pthread_mutex_unlock(&mutexSmem);
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::GetConnectCenterStatus(void)
{
try {
    bool tempCCStatus;
    pthread_mutex_lock(&mutexSmem);
    tempCCStatus=bConnectWithCenterStatus;
    pthread_mutex_unlock(&mutexSmem);
    return tempCCStatus;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vSetLastResetTime(long lLastResetSec)
{
try {
    struct tm *tm = localtime(&lLastResetSec);
    pthread_mutex_lock(&mutexSmem);
    ResetTime.Year = tm->tm_year - 11;
    ResetTime.Month = tm->tm_mon + 1;
    ResetTime.Day = tm->tm_mday;
    ResetTime.Hour = tm->tm_hour;
    ResetTime.Min = tm->tm_min;
    ResetTime.Sec = tm->tm_sec;
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch(...) {}
}
//---------------------------------------------------------------------------
YMDHMS SMEM::vGetLastResetTime(void)
{
try {
    YMDHMS tmp;
    pthread_mutex_lock(&mutexSmem);
    tmp = ResetTime;
    pthread_mutex_unlock(&mutexSmem);
    return tmp;
  } catch(...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vSetSimIntervalTime(itimerspec _SimInterVal)
{
try {
   pthread_mutex_lock(&mutexSmem);
   bSimIntervalTimeChange = true;                                           //��CTIMER���ˬd
   _ShareSimCycle = _SimInterVal;  //save struct in Share mem
   pthread_mutex_unlock(&mutexSmem);
   return true;
  } catch(...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vGetSimIntervalTimeChangeStatus()
{
try {
  bool bTMP;
  pthread_mutex_lock(&mutexSmem);
  bTMP = bSimIntervalTimeChange;
  pthread_mutex_unlock(&mutexSmem);
  return bTMP;
} catch(...){}
}
//---------------------------------------------------------------------------
bool SMEM::vSetSimIntervalTimeChangeStatus(bool bTMP)
{
try {
  pthread_mutex_lock(&mutexSmem);
  bSimIntervalTimeChange = bTMP;
  pthread_mutex_unlock(&mutexSmem);
  return bTMP;
} catch(...){}
}
//---------------------------------------------------------------------------
bool SMEM::vSet0FHardwareCycle(itimerspec _HWCycleReportIntervalIN)
{
try {
   pthread_mutex_lock(&mutexSmem);
    bHWCycleReportIntervalChange = true;                                        //��CTIMER���ˬd
   _HWCycleReportInterval = _HWCycleReportIntervalIN;  //save struct in Share mem
   vSave92COMMToDisk();                                                   //SAVE to DOM
   pthread_mutex_unlock(&mutexSmem);
   return true;
  } catch(...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vGet0FHardwareCycleChangeStatus()
{
try {
  bool bTMP;
  pthread_mutex_lock(&mutexSmem);
  bTMP = bHWCycleReportIntervalChange;
  pthread_mutex_unlock(&mutexSmem);
  return bTMP;
} catch(...){}
}
//---------------------------------------------------------------------------
bool SMEM::vSet0FHardwareCycleChangeStatus(bool bTMP)
{
try {
  pthread_mutex_lock(&mutexSmem);
  bHWCycleReportIntervalChange = bTMP;
  pthread_mutex_unlock(&mutexSmem);
  return bTMP;
} catch(...){}
}

//----------------------------------------------------------------------
itimerspec SMEM::vGet0FHardwareCycle(void)
{
try {
    itimerspec _tmp;
    pthread_mutex_lock(&mutexSmem);
    _tmp = _HWCycleReportInterval;
    pthread_mutex_unlock(&mutexSmem);
    return _tmp;
  } catch(...) {}
}
//----------------------------------------------------------------------
//---------------------------------------------------------------------------
unsigned char SMEM::vGetHardwareStatus(int iWhatByte)
{
try {
    unsigned char ucTMPByte3, ucTMPByte4, ucTMP;
    ucTMPByte3 = 0x00;                                                          //init
    ucTMPByte4 = 0x00;
    pthread_mutex_lock(&mutexSmem);

    //OT20110825
    if(bTrafficeLightBoardConnectStatus == false) {  //traffic light board failure
      ucTMPByte4 += 0x01;
    }

    ucTMPByte3 += 0x40;                                                       //bit14

    if(bConnectWithCenterStatus == true) {
      ucTMPByte3 += 0x01;                                                       //bit8
    }
    else {
      ucTMPByte3 += 0x80;                                                      //bit15
    }

/* disable signal conflict error */
/*
    if(_DIOByte.switchBit.b1 == true ||                                         //signal conflict error
       _DIOByte.switchBit.b2 == true ||
       _DIOByte.switchBit.b3 == true ||
       _DIOByte.switchBit.b4 == true ||
       _DIOByte.switchBit.b5 == true ||
       _DIOByte.switchBit.b6 == true    )
      ucTMPByte3 += 0x08;
*/
    if(bTCSignalConflictError == true) {                                        //signal conflict error 2
       ucTMPByte3 += 0x08;
    }

//    if(_DIOByte.switchBit.b1 == true)
//      ucTMPByte3 += 0x02;                                                       //bit9

    if(bTCDoorStatus == true && bHaveTCDoorSwitch == true)
      ucTMPByte3 += 0x02;                                                       //bit9

    if(bSignalDriverStatus == false) ucTMPByte4 += 0x40;    //bit6


    if(iWhatByte == 3)
      ucTMP = ucTMPByte3;
    else if (iWhatByte == 4)
      ucTMP = ucTMPByte4;
    else
      ucTMP = 0;
    pthread_mutex_unlock(&mutexSmem);
    return ucTMP;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vSetHWCycleCodeFor_0F14_0FC4(unsigned char ucIN)
{
try {
    pthread_mutex_lock(&mutexSmem);
    HWCycleCode = ucIN;
    vSave92COMMToDisk();                                                        //SAVE to DOM
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
unsigned char SMEM::vGetHWCycleCodeFor_0F14_0FC4(void)
{
try {
    unsigned char ucTMP;
    pthread_mutex_lock(&mutexSmem);
    ucTMP = HWCycleCode;
    pthread_mutex_unlock(&mutexSmem);
    return ucTMP;
  } catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSet0FCommandSet(unsigned char ucIN)
{
try {
    pthread_mutex_lock(&mutexSmem);
    ucCommandSet = ucIN;
    vSave92COMMToDisk();
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
unsigned char SMEM::vGet0FCommandSet(void)
{
try {
    unsigned char ucTMP;
    pthread_mutex_lock(&mutexSmem);
    ucTMP = ucCommandSet;
    pthread_mutex_unlock(&mutexSmem);
    return ucTMP;
  } catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSyncSSInfo(SmartSensorINFO _SSInfoInSS)
{
try {
    if(1) {                                                                       //no log
         pthread_mutex_lock(&mutexSmem);
         _SSInfoInShareMem = _SSInfoInSS;
         pthread_mutex_unlock(&mutexSmem);
    }
    return true;
  } catch(...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vCheckConnectStatus(void)
{
try {
  int i = 0;
  int NotSendSEQCount = 0;
  bool bOldStatus;
  MESSAGEOK _MSG;
  time_t NowSec = time(NULL);
    pthread_mutex_lock(&mutexSmem);
    for(i = 0 ; i < MSGWAITREPONSECOUNT; i++)
    {
      if (msgWaitResponse[i].sequence != INVAILDVALUE)
      {
        NotSendSEQCount++;

      //OTMARKPRINTF  printf("%s[Message] There are data in %d seq:%d, and retry count %d\n%s", ColorRed, i, msgWaitResponse[i].sequence, msgWaitResponse[i].retryCount, ColorNormal);


//re send msg
//        if( (NowSec - msgWaitResponse[i].sendTime) >= 5) {                      // more then 5 sec
        if( (NowSec - msgWaitResponse[i].sendTime) >= 3) {                      // more then 5 sec
             msgWaitResponse[i].retryCount++;
             msgWaitResponse[i].sendTime = NowSec;
             pthread_mutex_unlock(&mutexSmem);
             writeJob.WritePhysicalOut(msgWaitResponse[i].packet,msgWaitResponse[i].length,DEVICECENTER92);
             pthread_mutex_lock(&mutexSmem);
        }


//OT         if(msgWaitResponse[i].retryCount >= 6) {
         if(msgWaitResponse[i].retryCount >= 4) {
           msgWaitResponse[i].sequence = INVAILDVALUE;
           msgWaitResponse[i].retryCount = 0;
}

       }
      }
  pthread_mutex_unlock(&mutexSmem);
  return true;
  }
catch(...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vSetTimeInterVal(int connected, int disconnected)
{
try{
    bool bConnecting, bConnectingStatus;
    pthread_mutex_lock(&mutexSmem);

    if(connected != INVAILDVALUE ) siTransmitType[1] = connected;               //�Y��999�h���]�w

    if(disconnected != INVAILDVALUE ) siTimeGap = disconnected;       //�Y��999�h���]�w


    fprintf(stderr, "%sSend TimeInterval change to T[1]:%d, TimeGap:%d.%s\n", ColorRed, siTransmitType[1], siTimeGap, ColorNormal);


    bConnecting = bConnectWithCenter;
    bConnectingStatus = bConnectWithCenterStatus;
    pthread_mutex_unlock(&mutexSmem);

  //OTMARKPRINTF  printf("[MESSAGE] SetTimeIntervalOK!\n");

//  vSyncSSTimeInterVal();
  } catch(...) {}
}
//should add get status for 92vd
/*
//---------------------------------------------------------------------------
bool SMEM::vSyncSSTimeInterVal()
{
try {
    bool bConnecting, bConnectingStatus;
    int connected, disconnected;
    pthread_mutex_lock(&mutexSmem);

    connected = siTransmitType[1];

    disconnected = siTimeGap;

    bConnecting = bConnectWithCenter;
    bConnectingStatus = bConnectWithCenterStatus;
    pthread_mutex_unlock(&mutexSmem);

    MESSAGEOK _MSG;
    if(bConnectingStatus) {                                                     //�p�G�{�b�O�s�u�~���Interval
       _MSG = oDataToMessageOK.SendSSProtocalSetTimeInterval(connected);                //�]�wSmartSensor

     //OTMARKPRINTF  printf("[MESSAGE] Now in connect mode, set time interval is %d\n", connected);

    } else {
       _MSG = oDataToMessageOK.SendSSProtocalSetTimeInterval(disconnected);

     //OTMARKPRINTF  printf("[MESSAGE] Now in disconnect mode, set time interval is %d\n", disconnected);

    }
    _MSG.InnerOrOutWard = cOutWard;
    writeJob.WritePhysicalOut(_MSG.packet,_MSG.packetLength,DEVICESS);
  } catch(...) {}
}
*/
//---------------------------------------------------------------------------
bool SMEM::vLoadCenterConnectStatus(void)
{
try {
    bool status;
    pthread_mutex_lock(&mutexSmem);
    status = bConnectWithCenterStatus;
    pthread_mutex_unlock(&mutexSmem);
    return status;
  } catch(...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vSaveCenterConnectStatus(bool status)
{
try {

    pthread_mutex_lock(&mutexSmem);

    bConnectWithCenter=bConnectWithCenterStatus;
    bConnectWithCenterStatus=status;

    pthread_mutex_unlock(&mutexSmem);
    return true;

  /*
    pthread_mutex_lock(&mutexSmem);
    //OT Debug 0706
    if(bConnectWithCenterStatus != status && bConnectWithCenter == bConnectWithCenterStatus) {
      bConnectWithCenter=bConnectWithCenterStatus;
      bConnectWithCenterStatus=status;
    }
    pthread_mutex_unlock(&mutexSmem);
    return true;
  */
  } catch(...) {}
}
//----------------------------------------------------------------------
DATA_Bit SMEM::Get_DIOByte(void)
{
try {
    DATA_Bit temp;
    pthread_mutex_lock(&mutexSmem);
    temp=_DIOByte;
    pthread_mutex_unlock(&mutexSmem);
//    printf("DIO:%X\n", temp.DBit);
    return temp;
  } catch(...) {}
}
//----------------------------------------------------------------------
SmartSensorINFO SMEM::vGetSSInfo(void)
{
try {
    SmartSensorINFO _SSInfo;
    if(1) {                                                                       //check log!?
         pthread_mutex_lock(&mutexSmem);
         _SSInfo = _SSInfoInShareMem;
         pthread_mutex_unlock(&mutexSmem);
    }
    return _SSInfo;
  } catch(...) {}
}
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
bool SMEM::vGetCenterConnectOldStatus(void)
{
try{
    bool bTMP;
    pthread_mutex_lock(&mutexSmem);
    bTMP = bConnectWithCenter;
    pthread_mutex_unlock(&mutexSmem);
    return bTMP;
  } catch(...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vSet_DIOByte(DATA_Bit _tmpINDIO)
{
try {
    pthread_mutex_lock(&mutexSmem);
    _OldDIOByte = _DIOByte;
    _DIOByte = _tmpINDIO;
    pthread_mutex_unlock(&mutexSmem);
  } catch(...) {}
}
//----------------------------------------------------------------------
DATA_Bit SMEM::Get_OldDIOByte(void)
{
try {
    DATA_Bit temp;
    pthread_mutex_lock(&mutexSmem);
    temp=_OldDIOByte;
    pthread_mutex_unlock(&mutexSmem);
    return temp;
  } catch(...) {}
}
//----------------------------------------------------------------------
bool SMEM::vSave92COMMToDisk(void)
{
try {
    int iToro[8] = {0};
    disk.vWrite92COMMSettingFile(ucCommandSet,
                                 password,
                                 dbOperStat,
                                 HWCycleCode,
                                 iToro,
                                 iAdjudicateReturnAddressBCDorHEX,
                                 usiWayMappingRedCount,
                                 //OT Pass
                                 ucPassMode_0F8E8F,
                                 usiPassServerLCN,
                                 //OT Debug Direct
                                 usiSignamMapMappingDir,
                                 usiSignamMapMappingLightBoard,
//OT961211 new add IP editor
                                 localIp1_1,localIp1_2,localIp1_3,localIp1_4,localPort1,
                                 distIp0_1,distIp0_2,distIp0_3,distIp0_4,dist0Port,
                                 distIp1,distIp2,distIp3,distIp4,distPort,

                                 ucTC_TrainChainEnable, ucTrainComingForceJumpSubphase, ucTrainComingBanSubphase,

                                 ucReverseLaneManualEnable,

                                 //OT20111020
                                 netmask1, netmask2, netmask3, netmask4,
                                 gateway1, gateway2, gateway3, gateway4


                                 );
    return true;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vRead92COMMFromDisk(void)
{
try {
    int iToro[8];
    disk.vRead92COMMSettingFile( &ucCommandSet,
                                 password,
                                 &dbOperStat,
                                 &HWCycleCode,
                                 iToro,
                                 &iAdjudicateReturnAddressBCDorHEX,
                                 usiWayMappingRedCount,
                                 //OT Pass
                                 &ucPassMode_0F8E8F,
                                 &usiPassServerLCN,
                                 //OT Debug Direct
                                 usiSignamMapMappingDir,
                                 usiSignamMapMappingLightBoard,
//OT961211 new add IP editor
                                 &localIp1_1,&localIp1_2,&localIp1_3,&localIp1_4,&localPort1,
                                 &distIp0_1,&distIp0_2,&distIp0_3,&distIp0_4,&dist0Port,
                                 &distIp1,&distIp2,&distIp3,&distIp4,&distPort,

                                 &ucTC_TrainChainEnable, &ucTrainComingForceJumpSubphase, &ucTrainComingBanSubphase,

                                 &ucReverseLaneManualEnable,

                                 //OT20111020
                                 &netmask1, &netmask2, &netmask3, &netmask4,
                                 &gateway1, &gateway2, &gateway3, &gateway4


                                 );
    return true;
  } catch (...) {}
}


//OTCombo
//---------------------------------------------------------------------------
bool SMEM::vSave92TCSettingToDisk(void)
{
try {
    disk.vWrite92TCSettingFile( ucSmem92TC_ControlStrategy,
                                iSmem92TC_SignalLightStatus_5F0F_IntervalTime,
                                iSmem92TC_SignalStepStatus_5F03_IntervalTime,
                                iSmemTC_RedCountVer,
//OT Debug 0523
                                bTC_ActuateTypeFunctionEnable,
                                usiSmemTC_CCT_In_LongTanu_ActuateType_PlanID,
                                ucActuatePhaseExtend,
                                usiActuateVDID,
                                ucBootingDisplayRedcount,
                                ucTC_ActuateTypeByTOD
                               );
    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vRead92TCSettingFromDisk(void)
{
try {
    disk.vRead92TCSettingFile(  &ucSmem92TC_ControlStrategy,
                                &iSmem92TC_SignalLightStatus_5F0F_IntervalTime,
                                &iSmem92TC_SignalStepStatus_5F03_IntervalTime,
                                &iSmemTC_RedCountVer,
                                //OT Debug 0523
                                &bTC_ActuateTypeFunctionEnable,
                                &usiSmemTC_CCT_In_LongTanu_ActuateType_PlanID,
                                &ucActuatePhaseExtend,
                                &usiActuateVDID,
                                &ucBootingDisplayRedcount,
                                &ucTC_ActuateTypeByTOD
                               );

    return true;
  } catch (...) {}
}


//---------------------------------------------------------------------------
bool SMEM::vGet0F11CommuncationResetChangeStatus()
{
try {
  bool bTMP;
  pthread_mutex_lock(&mutexSmem);
  bTMP = b0F11CommuncationResetChangeStatus;
  pthread_mutex_unlock(&mutexSmem);
  return bTMP;
} catch(...){}
}
//----------------------------------------------------------------------
bool SMEM::vSet0F11CommuncationResetChangeStatus(bool bTMP)
{
try {
  pthread_mutex_lock(&mutexSmem);
  b0F11CommuncationResetChangeStatus = bTMP;
  pthread_mutex_unlock(&mutexSmem);
  return bTMP;
} catch(...){}
}
//----------------------------------------------------------------------
bool SMEM::vGet0F16LockDBRequestStatus()
{
try {
  bool bTMP;
  pthread_mutex_lock(&mutexSmem);
  bTMP = b0F11CommuncationResetChangeStatus;
  bTMP = b0F16LockDBRequestStatus;
  pthread_mutex_unlock(&mutexSmem);
  return bTMP;
} catch(...){}
}
//----------------------------------------------------------------------
bool SMEM::vSet0F16LockDBRequestStatus(bool bTMP)
{
try {
  pthread_mutex_lock(&mutexSmem);
  b0F16LockDBRequestStatus = bTMP;
  pthread_mutex_unlock(&mutexSmem);
  return bTMP;
} catch(...){}
}
//---------------------------------------------------------------------------
bool SMEM::vSaveShareMemoryDataToDOM()
{
try {

  unsigned short int iCurrentPhaseID  = stc.vGetUSIData(CSTC_exec_plan_phase_order);
  unsigned short int iCurrentPlanID   = stc.vGetUSIData(CSTC_exec_plan_plan_ID);
  stc.Lock_to_Load_Plan_for_Panel(iCurrentPlanID);
  stc.Lock_to_Load_Phase_for_Panel(iCurrentPhaseID);
  unsigned int uiPhaseRunningTime;
  unsigned short int usiLocalRevStep;
  CReverseTimeInfo _LocalRev;
  unsigned int uiRevSec;

  char cTMP[128];
  bzero(cTMP, 128);

  char cTransmitChar[65535];
  bzero(cTransmitChar, 65535);

  stc.vGetRevInfo(&usiLocalRevStep, &_LocalRev);
  uiRevSec = stc.vGetRevTimerSec();


  fprintf(stderr, "%s[Message] Recode SMEM Data to DOM,%s\n", ColorRed, ColorNormal);

  pthread_mutex_lock(&mutexSmem);

  sprintf(cTMP, "[TC]\n");
  strcat(cTransmitChar, cTMP);

  sprintf(cTMP, "d   iCurrentPlanID: %02d,   ", iCurrentPlanID);
  strcat(cTransmitChar, cTMP);

  sprintf(cTMP, "iCurrentPhaseID: 0x%02X\n", iCurrentPhaseID);
  strcat(cTransmitChar, cTMP);

  sprintf(cTMP, "d   CurrentSubphase: %d/%d,", stc.vGetUSIData(CSTC_exec_phase_current_subphase) + 1, stc._panel_phase._subphase_count);
  strcat(cTransmitChar, cTMP);

  sprintf(cTMP, "   CurrentStep: %02d,", stc.vGetUSIData(CSTC_exec_phase_current_subphase_step));
  strcat(cTransmitChar, cTMP);
//  sprintf(cTMP, "d   CurrentStepTotal: %d\n", stc._panel_phase._total_step_count);
//  strcat(cTransmitChar, cTMP);
  sprintf(cTMP, "   StepSec: %03d\n", stc.vGetStepTime());
  strcat(cTransmitChar, cTMP);

  sprintf(cTMP, "d   address: %d\n", address);
  strcat(cTransmitChar, cTMP);

//OT20111128
  sprintf(cTMP, "d   machineLocation: %d\n", ucMachineLocation);
  strcat(cTransmitChar, cTMP);

  sprintf(cTMP, "d   iFirmware: %d %d %d %x\n", iFirmwareYear, iFirmwareMonth, iFirmwareDay, iFirmwareCompanyID);
  strcat(cTransmitChar, cTMP);

  sprintf(cTMP, "d   iRevStep:%d, revSyncEnable:%d, Sec:%d, revSyncStatus:%d\nd   revCenterByte:%x, [D:%d, T:%d, S:%d, Seg:%d, TabT:%d]\n",
                                                                                usiLocalRevStep,
                                                                                ucRevSyncEnable,
                                                                                uiRevSec,
                                                                                iRevSyncStatus,
                                                                                ucRevSyncByteStatus,
                                                                                _NewestRevSyncStatus.bYMD_OK,
                                                                                _NewestRevSyncStatus.bTime_OK,
                                                                                _NewestRevSyncStatus.bRevStep_OK,
                                                                                _NewestRevSyncStatus.bRevSegSet_OK,
                                                                                _NewestRevSyncStatus.bRevTime_OK
                                                                                );
  strcat(cTransmitChar, cTMP);

  uiPhaseRunningTime = vGetActRunningGreenTime();
  sprintf(cTMP, "d   PhaseRunningTime:%d\n", uiPhaseRunningTime);
  strcat(cTransmitChar, cTMP);

  sprintf(cTMP, "d   iAdjudicateReturnAddressBCDorHEX: %d\n", iAdjudicateReturnAddressBCDorHEX);
  strcat(cTransmitChar, cTMP);


/*
  sprintf(cTMP, "l   _ShareSimCycle.it_value.tv_sec: %d\n", _ShareSimCycle.it_value.tv_sec);
  strcat(cTransmitChar, cTMP);
  sprintf(cTMP, "d   _ShareSimCycle.it_interval.tv_sec: %d\n", _ShareSimCycle.it_interval.tv_sec);
  strcat(cTransmitChar, cTMP);
  sprintf(cTMP, "l   _HWCycleReportInterval.it_value.tv_sec: %d\n", _HWCycleReportInterval.it_value.tv_sec);
  strcat(cTransmitChar, cTMP);
  sprintf(cTMP, "d   _HWCycleReportInterval.it_interval.tv_sec: %d\n", _HWCycleReportInterval.it_interval.tv_sec);
  strcat(cTransmitChar, cTMP);
*/
  sprintf(cTMP, "x   HWCycleCode: 0x%x\n", HWCycleCode);
  strcat(cTransmitChar, cTMP);
  sprintf(cTMP, "x   ucCommandSet: 0x%x\n", ucCommandSet);
  strcat(cTransmitChar, cTMP);
  sprintf(cTMP, "x   _DIOByte.DBit: 0x%x\n", _DIOByte.DBit);
/*
  strcat(cTransmitChar, cTMP);
  sprintf(cTMP, "s   _SSInfoInShareMem.cMultiDropID: %s\n", _SSInfoInShareMem.cMultiDropID);
*/
  strcat(cTransmitChar, cTMP);
  sprintf(cTMP, "d   bConnectWithCenter: %d\n", bConnectWithCenter);
  strcat(cTransmitChar, cTMP);
  sprintf(cTMP, "d   bConnectWithCenterStatus: %d\n", bConnectWithCenterStatus);
  strcat(cTransmitChar, cTMP);

  //OT20110528
  struct tm *now = localtime(&ucLastShowRedCountTime);
  sprintf(cTMP, "last redcount time at:%d:%d:%d\n", (now->tm_hour), (now->tm_min), (now->tm_sec));
  strcat(cTransmitChar, cTMP);
  if(iSmemTC_RedCountVer == TC_RedCountNA) {
    sprintf(cTMP, "redcount ver:TC_RedCountNA\n");
  } else if(iSmemTC_RedCountVer == TC_RedCountVerHK) {
    sprintf(cTMP, "redcount ver:TC_RedCountVerHK\n");
  } else if(iSmemTC_RedCountVer == TC_RedCountVer94) {
    sprintf(cTMP, "redcount ver:TC_RedCountVer94\n");
  } else if(iSmemTC_RedCountVer == TC_RedCountVer94v2) {
    sprintf(cTMP, "redcount ver:TC_RedCountVer94v2\n");
  } else if(iSmemTC_RedCountVer == TC_RedCountVerCCT97) {
    sprintf(cTMP, "redcount ver:TC_RedCountVerCCT97\n");
  }
  strcat(cTransmitChar, cTMP);

  sprintf(cTMP, "redcount status: ucBootingDisplayRedcount:%d, ucDisplayRedcount:%d\n", ucBootingDisplayRedcount, ucDisplayRedcount);
  strcat(cTransmitChar, cTMP);

  sprintf(cTMP, "last redcount [1:%d] [2:%d] [3:%d] [4:%d] [5:%d] [6:%d] [7:%d] [8:%d] \n",
  ucLastShowRedCountVar[0], ucLastShowRedCountVar[1], ucLastShowRedCountVar[2], ucLastShowRedCountVar[3],
  ucLastShowRedCountVar[4], ucLastShowRedCountVar[5], ucLastShowRedCountVar[6], ucLastShowRedCountVar[7] );
  strcat(cTransmitChar, cTMP);

  pthread_mutex_unlock(&mutexSmem);

  disk.vWriteShareMEMALLFile(cTransmitChar);

}catch(...){}
}

//----------------------------------------------------------------------
bool SMEM::vWriteMsgToDOM(char *cString)
{
try {
      char temp[300] = { 0 };
      strcpy(temp,cString);
      disk.vWriteMsgToFile(temp);
      return true;
    } catch(...) {}
}
//------------jacky20140507----------------------------------------------------------
bool SMEM::vWriteReverseLog(char *cString)
{
try {
      char temp[300] = { 0 };
      strcpy(temp,cString);
      disk.vWriteReverseLog(temp);
      return true;
    } catch(...) {}
}
//----------------------------------------------------------------------
//OTCombo
unsigned char SMEM::vGetUCData(const int iSelect)
{
try {
  unsigned char ucTMP;
  switch(iSelect) {

    case(TC92_ucControlStrategy):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSmem92TC_ControlStrategy;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(200):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSignalLightStatus[0];
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(201):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSignalLightStatus[1];
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(202):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSignalLightStatus[2];
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(203):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSignalLightStatus[3];
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(204):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSignalLightStatus[4];
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(205):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSignalLightStatus[5];
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(206):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSignalLightStatus[6];
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(207):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSignalLightStatus[7];
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(208):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSignalLightStatus[8];
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(209):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSignalLightStatus[9];
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(210):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSignalLightStatus[10];
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(211):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSignalLightStatus[11];
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(212):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSignalLightStatus[12];
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(213):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSignalLightStatus[13];
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92_5F31Manual):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTC92_5F31Manual;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC92_5F31TOD):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTC92_5F31TOD;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC92_5F31StartSubPhaseId):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTC92_5F31StartSubPhaseId;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC92_5F31EndSubPhaseId):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTC92_5F31EndSubPhaseId;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC92_5F32StartSubPhaseId):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTC92_5F32StartSubPhaseId;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC92_5F32EndSubPhaseId):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTC92_5F32EndSubPhaseId;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_MotherChainStartStepId):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTC_MotherChainStartStepId;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_MotherChainEndStepId):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTC_MotherChainEndStepId;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_KeyPadP6Value):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTC_KeyPadP6Value;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_TrainChainEnable):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTC_TrainChainEnable;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_TrainChainNOW):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTC_TrainChainStatusNOW;
      printf("get, ucTC_TrainChainStatusLast:%d, ucTC_TrainChainStatusNOW:%d\n", ucTC_TrainChainStatusLast, ucTC_TrainChainStatusNOW);
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_TrainChainLast):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTC_TrainChainStatusLast;
//      printf("get, ucTC_TrainChainStatusLast:%d, ucTC_TrainChainStatusNOW:%d\n", ucTC_TrainChainStatusLast, ucTC_TrainChainStatusNOW);
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_TrainComingBanSubphase):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTrainComingBanSubphase;
      pthread_mutex_unlock(&mutexSmem);
      vSave92COMMToDisk();
    break;

    case(TC_TrainComingForceJumpSubphase):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTrainComingForceJumpSubphase;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_ReverseLane_Control_Mode):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucReverseLaneControlMode;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_ReverseLane_Manual_Enable_Status):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucReverseLaneManualEnable;
//      printf("in smem, ucReverseLaneManualEnable:%d\n", ucReverseLaneManualEnable);
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_Redcount_Booting_Display_Enable):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucBootingDisplayRedcount;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_Redcount_Display_Enable):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucDisplayRedcount;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_CSTC_ControlStrategy):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucCSTCControlStrategy;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_CSTC_FieldOperate):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucCSTCFieldOperate;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_CCT_ActuateType_By_TOD):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTC_ActuateTypeByTOD;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(DynShowRedCountForAct):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSmemTC_DynShowRedCountForAct;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC_CCT_Send_ActuateArwen_Protocal):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSendCCTActArwenProtocal;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(revSyncEnable):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucRevSyncEnable;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(revSyncInterval):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucRevSyncInterval;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(revSyncTimeDiffTolarence):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucRevSyncTimeDiffTolarence;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(revSyncCountineStatusAdj):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucRevSyncCountineStatusAdj;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(revDefaultVehWay):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucRevDefaultVehWay;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(revLogToFile):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucRevLogToFile;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(revSendProtocol):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucRevSendProtocol;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(revSyncByteStatus):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucRevSyncByteStatus;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(amegidsDynEnable):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucAmegidsDynEnable;
      pthread_mutex_unlock(&mutexSmem);
      break;

//OT20110607
    case(TC_CCT_SendCCTPhaseCycleProtocalForCenter):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSendCCTPhaseCycleProtocalForCenter;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(CSTC_SegmentNoRunning):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucSegmentNoRunning;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(CSTC_RedcountRepeatCount):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucRedcountRepeatCount;
      pthread_mutex_unlock(&mutexSmem);
      break;

    //OT20111128
    case(TC_CCT_MachineLocation):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucMachineLocation;
      pthread_mutex_unlock(&mutexSmem);
      break;

//OT20111201
    case(TC_TrainComingBanSubphase2):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucTrainComingBanSubphase2;
      pthread_mutex_unlock(&mutexSmem);
//using oStorINI      vSave92COMMToDisk();
    break;

//OT20140415
    case(TCDynSegAdjustType):
      pthread_mutex_lock(&mutexSmem);
      ucTMP = ucDynSegAdjustType;
      pthread_mutex_unlock(&mutexSmem);
    break;


    default:
      ucTMP = 0;
      break;
  }
  return ucTMP;
} catch (...) {}
}

//----------------------------------------------------------------------
bool SMEM::vSetUCData(const int iSelect, const unsigned char ucTMP)
{
try {
  switch(iSelect) {

    case(TC92_ucControlStrategy):
      pthread_mutex_lock(&mutexSmem);
      ucSmem92TC_ControlStrategy = ucTMP;
      vSave92TCSettingToDisk();
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(200):
      pthread_mutex_lock(&mutexSmem);
      ucSignalLightStatus[0] = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(201):
      pthread_mutex_lock(&mutexSmem);
      ucSignalLightStatus[1] = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(202):
      pthread_mutex_lock(&mutexSmem);
      ucSignalLightStatus[2] = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(203):
      pthread_mutex_lock(&mutexSmem);
      ucSignalLightStatus[3] = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(204):
      pthread_mutex_lock(&mutexSmem);
      ucSignalLightStatus[4] = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(205):
      pthread_mutex_lock(&mutexSmem);
      ucSignalLightStatus[5] = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(206):
      pthread_mutex_lock(&mutexSmem);
      ucSignalLightStatus[6] = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(207):
      pthread_mutex_lock(&mutexSmem);
      ucSignalLightStatus[7] = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(208):
      pthread_mutex_lock(&mutexSmem);
      ucSignalLightStatus[8] = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(209):
      pthread_mutex_lock(&mutexSmem);
      ucSignalLightStatus[9] = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(210):
      pthread_mutex_lock(&mutexSmem);
      ucSignalLightStatus[10] = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(211):
      pthread_mutex_lock(&mutexSmem);
      ucSignalLightStatus[11] = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(212):
      pthread_mutex_lock(&mutexSmem);
      ucSignalLightStatus[12] = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(213):
      pthread_mutex_lock(&mutexSmem);
      ucSignalLightStatus[13] = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC_GreenConflictDetFromCSTC):
      pthread_mutex_lock(&mutexSmem);
      ucGreenConflictDetFromCSTC = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(TC_GreenConflictDetFromDIO):
      pthread_mutex_lock(&mutexSmem);
      ucGreenConflictDetFromDIO = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92_5F31Manual):
      pthread_mutex_lock(&mutexSmem);
      ucTC92_5F31Manual = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC92_5F31TOD):
      pthread_mutex_lock(&mutexSmem);
      ucTC92_5F31TOD = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      smem.vWriteChainDataFromStroage();
    break;

    case(TC92_5F31StartSubPhaseId):
      pthread_mutex_lock(&mutexSmem);
      ucTC92_5F31StartSubPhaseId = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC92_5F31EndSubPhaseId):
      pthread_mutex_lock(&mutexSmem);
      ucTC92_5F31EndSubPhaseId = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      smem.vWriteChainDataFromStroage();
    break;

    case(TC92_5F32StartSubPhaseId):
      pthread_mutex_lock(&mutexSmem);
      ucTC92_5F32StartSubPhaseId = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC92_5F32EndSubPhaseId):
      pthread_mutex_lock(&mutexSmem);
      ucTC92_5F32EndSubPhaseId = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      smem.vWriteChainDataFromStroage();
    break;

    case(TC_MotherChainStartStepId):
      pthread_mutex_lock(&mutexSmem);
      ucTC_MotherChainStartStepId = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_MotherChainEndStepId):
      pthread_mutex_lock(&mutexSmem);
      ucTC_MotherChainEndStepId = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      smem.vWriteChainDataFromStroage();
    break;

    case(TC_KeyPadP6Value):
      pthread_mutex_lock(&mutexSmem);
      ucTC_KeyPadP6Value = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_TrainChainEnable):
      pthread_mutex_lock(&mutexSmem);
      ucTC_TrainChainEnable = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_TrainChainNOW):
      pthread_mutex_lock(&mutexSmem);
      ucTC_TrainChainStatusLast = ucTC_TrainChainStatusNOW;
      ucTC_TrainChainStatusNOW = ucTMP;
//      printf("set, ucTC_TrainChainStatusLast:%d, ucTC_TrainChainStatusNOW:%d\n", ucTC_TrainChainStatusLast, ucTC_TrainChainStatusNOW);
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_TrainComingBanSubphase):
      pthread_mutex_lock(&mutexSmem);
      ucTrainComingBanSubphase = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_TrainComingForceJumpSubphase):
      pthread_mutex_lock(&mutexSmem);
      ucTrainComingForceJumpSubphase = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_ReverseLane_Control_Mode):
      pthread_mutex_lock(&mutexSmem);
      ucReverseLaneControlMode = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_ReverseLane_Manual_Enable_Status):
      pthread_mutex_lock(&mutexSmem);
      ucReverseLaneManualEnable = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      vSave92COMMToDisk();
    break;

    case(TC_Redcount_Booting_Display_Enable):
      pthread_mutex_lock(&mutexSmem);
      ucBootingDisplayRedcount = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      vSave92TCSettingToDisk();
    break;

    case(TC_Redcount_Display_Enable):
      pthread_mutex_lock(&mutexSmem);
      ucDisplayRedcount = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_CSTC_ControlStrategy):
      pthread_mutex_lock(&mutexSmem);
      ucCSTCControlStrategy = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_CSTC_FieldOperate):
      pthread_mutex_lock(&mutexSmem);
      ucCSTCFieldOperate = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TC_CCT_ActuateType_By_TOD):
      pthread_mutex_lock(&mutexSmem);
      ucTC_ActuateTypeByTOD = ucTMP;
      vSave92TCSettingToDisk();
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(DynShowRedCountForAct):
      pthread_mutex_lock(&mutexSmem);
      ucSmemTC_DynShowRedCountForAct = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC_CCT_Send_ActuateArwen_Protocal):
      pthread_mutex_lock(&mutexSmem);
      ucSendCCTActArwenProtocal = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(revSyncEnable):
      pthread_mutex_lock(&mutexSmem);
      ucRevSyncEnable = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(revSyncInterval):
      pthread_mutex_lock(&mutexSmem);
      ucRevSyncInterval = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(revSyncTimeDiffTolarence):
      pthread_mutex_lock(&mutexSmem);
      ucRevSyncTimeDiffTolarence = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(revSyncCountineStatusAdj):
      pthread_mutex_lock(&mutexSmem);
      ucRevSyncCountineStatusAdj = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(revDefaultVehWay):
      pthread_mutex_lock(&mutexSmem);
      ucRevDefaultVehWay = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(revLogToFile):
      pthread_mutex_lock(&mutexSmem);
      ucRevLogToFile = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(revSendProtocol):
      pthread_mutex_lock(&mutexSmem);
      ucRevSendProtocol = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(revSyncByteStatus):
      pthread_mutex_lock(&mutexSmem);
      ucRevSyncByteStatus = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(amegidsDynEnable):
      pthread_mutex_lock(&mutexSmem);
      ucAmegidsDynEnable = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

//OT20110607
    case(TC_CCT_SendCCTPhaseCycleProtocalForCenter):
      pthread_mutex_lock(&mutexSmem);
      ucSendCCTPhaseCycleProtocalForCenter = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(CSTC_SegmentNoRunning):
      pthread_mutex_lock(&mutexSmem);
      ucSegmentNoRunning = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(CSTC_RedcountRepeatCount):
      pthread_mutex_lock(&mutexSmem);
      ucRedcountRepeatCount = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    //OT20111128
    case(TC_CCT_MachineLocation):
      pthread_mutex_lock(&mutexSmem);
      ucMachineLocation = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    //OT20111201
    case(TC_TrainComingBanSubphase2):
      pthread_mutex_lock(&mutexSmem);
      ucTrainComingBanSubphase2 = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    //OT20140415
    case(TCDynSegAdjustType):
      pthread_mutex_lock(&mutexSmem);
      ucDynSegAdjustType = ucTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;


    default:
      return false;
      break;
  }
  return true;
} catch (...) {}
}

//----------------------------------------------------------------------
int SMEM::vGetINTData(const int iSelect)
{
try {
  int iRet;
  switch(iSelect) {

    case(TC92_iEffectTime):
      pthread_mutex_lock(&mutexSmem);
      iRet = iSmem92TC_EffectTime;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92_iUpdatePhaseData):
      pthread_mutex_lock(&mutexSmem);
      iRet = iSmem92TC_PhaseUpdateFlag;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92SignalLightStatus_5F0F_IntervalTime):
      pthread_mutex_lock(&mutexSmem);
      iRet = iSmem92TC_SignalLightStatus_5F0F_IntervalTime;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92SignalStepStatus_5F03_IntervalTime):
      pthread_mutex_lock(&mutexSmem);
      iRet = iSmem92TC_SignalStepStatus_5F03_IntervalTime;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92_PlanOneTime5F18_PlanID):
      pthread_mutex_lock(&mutexSmem);
      iRet = iSmem92TC_ChangePlanOneTime5F18_PlanID;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92_RedCountVer):
      pthread_mutex_lock(&mutexSmem);
      iRet = iSmemTC_RedCountVer;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(Com2_TYPE):
      pthread_mutex_lock(&mutexSmem);
      iRet = iSmem_Com2Type;
      pthread_mutex_unlock(&mutexSmem);
      break;

      //OT Debug 0410
          case(TC92_5F1C_SPID):
            pthread_mutex_lock(&mutexSmem);
            iRet = iSmemTC_SPID;
            pthread_mutex_unlock(&mutexSmem);
            break;
          case(TC92_5F1C_SID):
            pthread_mutex_lock(&mutexSmem);
            iRet = iSmemTC_SID;
            pthread_mutex_unlock(&mutexSmem);
            break;
          case(TC92_5F1C_ET):
            pthread_mutex_lock(&mutexSmem);
            iRet = iSmemTC_ET;
            pthread_mutex_unlock(&mutexSmem);
            break;

    default:
      iRet = 0;
      break;
  }

  return iRet;
} catch (...) {}
}
//----------------------------------------------------------------------
bool SMEM::vSetINTData(const int iSelect, const int iTMP)
{
try {
  switch(iSelect) {

    case(TC92_iEffectTime):
      pthread_mutex_lock(&mutexSmem);
      iSmem92TC_EffectTime = iTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92_iUpdatePhaseData):
      pthread_mutex_lock(&mutexSmem);
      iSmem92TC_PhaseUpdateFlag = iTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92SignalLightStatus_5F0F_IntervalTime):
      pthread_mutex_lock(&mutexSmem);
      bSmem92TC_TransmitCycle5F0F_ChangeFlag = true;
      iSmem92TC_SignalLightStatus_5F0F_IntervalTime = iTMP;
      vSave92TCSettingToDisk();
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92SignalStepStatus_5F03_IntervalTime):
      pthread_mutex_lock(&mutexSmem);
      bSmem92TC_TransmitCycle5F03_ChangeFlag = true;
      iSmem92TC_SignalStepStatus_5F03_IntervalTime = iTMP;
      vSave92TCSettingToDisk();
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92_PlanOneTime5F18_PlanID):
      pthread_mutex_lock(&mutexSmem);
      bSmem92TC_ChangePlanOneTime5F18_ChangeFlag = true;
      iSmem92TC_ChangePlanOneTime5F18_PlanID = iTMP;
      printf("Set iSmem92TC_ChangePlanOneTime5F18_PlanID:%d\n", iSmem92TC_ChangePlanOneTime5F18_PlanID);
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92_RedCountVer):
      pthread_mutex_lock(&mutexSmem);
      iSmemTC_RedCountVer = iTMP;
      vSave92TCSettingToDisk();
      pthread_mutex_unlock(&mutexSmem);
      break;

//OT Debug 0410
    case(TC92_5F1C_SPID):
      pthread_mutex_lock(&mutexSmem);
      iSmemTC_SPID = iTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(TC92_5F1C_SID):
      pthread_mutex_lock(&mutexSmem);
      iSmemTC_SID = iTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(TC92_5F1C_ET):
      pthread_mutex_lock(&mutexSmem);
      iSmemTC_ET = iTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(Com2_TYPE):
      pthread_mutex_lock(&mutexSmem);
//      iSmem_Com2Type = iTMP;
      disk.vWriteCom2TypeFromFile(iTMP);
      pthread_mutex_unlock(&mutexSmem);
      break;

    default:
      return false;
      break;
  }

  return true;
} catch (...) {}
}


//OOOO
//OT Debug 0523
//----------------------------------------------------------------------
unsigned short int SMEM::vGetUSIData(const int iSelect)
{
try {
  unsigned short int usiRet;
  switch(iSelect) {

    case(TC_CCT_In_LongTanu_ActuateType_Switch):
      pthread_mutex_lock(&mutexSmem);
      usiRet = usiSmemTC_CCT_In_LongTanu_ActuateType_PlanID;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC_ActuateVDID):
      pthread_mutex_lock(&mutexSmem);
      usiRet = usiActuateVDID;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(amegidsDynDevLCN):
      pthread_mutex_lock(&mutexSmem);
      usiRet = usiAmegidsDynDevLCN;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TOD_PLAN_ID):
      pthread_mutex_lock(&mutexSmem);
      usiRet = usiTODPlanID;
      pthread_mutex_unlock(&mutexSmem);
    break;

    //OT20131225
    case(CCJ_HeartBeatCount):
      pthread_mutex_lock(&mutexSmem);
      usiRet = usiCCJHeartBeatCount;
      pthread_mutex_unlock(&mutexSmem);
    break;

    //OT20140415
    case(TCDynSegAdjustSec):
      pthread_mutex_lock(&mutexSmem);
      usiRet = usiDynSegAdjustSec;
      pthread_mutex_unlock(&mutexSmem);
    break;
    case(TCDynSegRemainSec):
      pthread_mutex_lock(&mutexSmem);
      usiRet = usiDynSegRemainSec;
      pthread_mutex_unlock(&mutexSmem);
    break;

    default:
      usiRet = 0;
      break;
  }

//  printf("smem.vGet: %d\n" ,usiSmemTC_CCT_In_LongTanu_ActuateType_PlanID);

  return usiRet;
} catch (...) {}
}
//----------------------------------------------------------------------
bool SMEM::vSetUSIData(const int iSelect, const unsigned short int usiTMP)
{
try {
  switch(iSelect) {

    case(TC_CCT_In_LongTanu_ActuateType_Switch):
      pthread_mutex_lock(&mutexSmem);
      usiSmemTC_CCT_In_LongTanu_ActuateType_PlanID = usiTMP;
      vSave92TCSettingToDisk();
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC_ActuateVDID):
      pthread_mutex_lock(&mutexSmem);
      usiActuateVDID = usiTMP;
      vSave92TCSettingToDisk();
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(amegidsDynDevLCN):
      pthread_mutex_lock(&mutexSmem);
      usiAmegidsDynDevLCN = usiTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TOD_PLAN_ID):
      pthread_mutex_lock(&mutexSmem);
      usiTODPlanID = usiTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(TCDynSegAdjustSec):
      pthread_mutex_lock(&mutexSmem);
      usiDynSegAdjustSec = usiTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;
    case(TCDynSegRemainSec):
      pthread_mutex_lock(&mutexSmem);
      usiDynSegRemainSec = usiTMP;
      pthread_mutex_unlock(&mutexSmem);
    break;

    default:
      return false;
      break;
  }

  return true;
} catch (...) {}
}


//----------------------------------------------------------------------
bool SMEM::vGetBOOLData(const int iSelect)
{
try {
  bool bRet;
  switch(iSelect) {

    case(TC92_TransmitCycle5F03_Change):
      pthread_mutex_lock(&mutexSmem);
      bRet = bSmem92TC_TransmitCycle5F03_ChangeFlag;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(TC92_TransmitCycle5F0F_Change):
      pthread_mutex_lock(&mutexSmem);
      bRet = bSmem92TC_TransmitCycle5F0F_ChangeFlag;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92_PlanOneTime5F18_Change):
      pthread_mutex_lock(&mutexSmem);
      bRet = bSmem92TC_ChangePlanOneTime5F18_ChangeFlag;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC_DoorStatus_Change):
      pthread_mutex_lock(&mutexSmem);
      bRet = bTCDoorStatus;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC_SignalConflictError):
      pthread_mutex_lock(&mutexSmem);
      bRet = bTCSignalConflictError;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92_SubPhaseOfPhasePlanIncorrent):
      pthread_mutex_lock(&mutexSmem);
      bRet = bSmem92TC_SubPhaseOfPhasePlanIncorrent;
      pthread_mutex_unlock(&mutexSmem);
      break;

//OT Debug 0523
    case(TC_CCT_In_LongTanu_ActuateType_Switch):
      pthread_mutex_lock(&mutexSmem);
      bRet = bSmemTC_CCT_In_LongTanu_ActuateType_Switch;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(TC_CCT_In_LongTanu_ActuateType_FunctionEnable):
      pthread_mutex_lock(&mutexSmem);
      bRet = bTC_ActuateTypeFunctionEnable;
      pthread_mutex_unlock(&mutexSmem);
      break;

//OT Debug Signal 951116
    case(TC_SIGNAL_NEXT_STEP_OK):
      pthread_mutex_lock(&mutexSmem);
      bRet = bTC_SIGNAL_NEXT_STEP_OK;
      pthread_mutex_unlock(&mutexSmem);
      break;

//OT Debug Signal 951116
    case(TC_SIGNAL_DRIVER_UNIT):
      pthread_mutex_lock(&mutexSmem);
      bRet = bSignalDriverStatus;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(GPS_SYNC):
      pthread_mutex_lock(&mutexSmem);
      bRet = bGPSSyncStatus;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC_CCTActuate_TOD_Running):
      pthread_mutex_lock(&mutexSmem);
      bRet = bActuateRunningFlag;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC_Actuate_By_TFD):
      pthread_mutex_lock(&mutexSmem);
      bRet = bActuateByTFD;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(CCJ_SendStepToCCJ):
      pthread_mutex_lock(&mutexSmem);
      bRet = bEnableSendStepToCCJ;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(NextStepForceEnable):
      pthread_mutex_lock(&mutexSmem);
      bRet = bNextStepForceEnable;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(EnableUpdateRTC):
      pthread_mutex_lock(&mutexSmem);
      bRet = bEnableUpdateRTC;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(IFGetResetTCSignal):
      pthread_mutex_lock(&mutexSmem);
      bRet = bGetResetTCSignal;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TCSegmentTypeUpdate):
      pthread_mutex_lock(&mutexSmem);
      bRet = bTCSegmentTypeUpdate;
      pthread_mutex_unlock(&mutexSmem);
      break;

    //OT20140415
    case(TCDynSegSwitch):
      pthread_mutex_lock(&mutexSmem);
      bRet = bDynSegSwitch;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(TCDynSegStatus):
      pthread_mutex_lock(&mutexSmem);
      bRet = bDynSegStatus;
      pthread_mutex_unlock(&mutexSmem);
      break;

    default:
      bRet = false;
      break;
  }
  return bRet;
} catch (...) {}
}

//----------------------------------------------------------------------
bool SMEM::vSetBOOLData(const int iSelect, const bool bTMP)
{
try {
  switch(iSelect) {

    case(TC92_TransmitCycle5F0F_Change):
      pthread_mutex_lock(&mutexSmem);
      bSmem92TC_TransmitCycle5F0F_ChangeFlag = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(TC92_TransmitCycle5F03_Change):
      pthread_mutex_lock(&mutexSmem);
      bSmem92TC_TransmitCycle5F03_ChangeFlag = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92_PlanOneTime5F18_Change):
      pthread_mutex_lock(&mutexSmem);
      bSmem92TC_ChangePlanOneTime5F18_ChangeFlag = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC_DoorStatus_Change):
      pthread_mutex_lock(&mutexSmem);
      if(bTMP != bTCDoorStatus) bTCDoorStatus = bTMP;
      if(bTMP == false && bHaveTCDoorSwitch == false) bHaveTCDoorSwitch = true;     // only when have switch, can get low voltage signal
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC_SignalConflictError):
      pthread_mutex_lock(&mutexSmem);
      bTCSignalConflictError = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC92_SubPhaseOfPhasePlanIncorrent):
      pthread_mutex_lock(&mutexSmem);
      bSmem92TC_SubPhaseOfPhasePlanIncorrent = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

//OT Debug 0523
    case(TC_CCT_In_LongTanu_ActuateType_Switch):
      pthread_mutex_lock(&mutexSmem);
      bSmemTC_CCT_In_LongTanu_ActuateType_Switch = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(TC_CCT_In_LongTanu_ActuateType_FunctionEnable):
      pthread_mutex_lock(&mutexSmem);
      bTC_ActuateTypeFunctionEnable = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

//OT Debug Signal 951116
    case(TC_SIGNAL_NEXT_STEP_OK):
      pthread_mutex_lock(&mutexSmem);
      bTC_SIGNAL_NEXT_STEP_OK = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

//OT Debug Signal 951116
    case(TC_SIGNAL_DRIVER_UNIT):
      pthread_mutex_lock(&mutexSmem);
      bSignalDriverStatus = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(GPS_SYNC):
      pthread_mutex_lock(&mutexSmem);
      bGPSSyncStatus = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC_CCTActuate_TOD_Running):
      pthread_mutex_lock(&mutexSmem);
      bActuateRunningFlag = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TC_Actuate_By_TFD):
      pthread_mutex_lock(&mutexSmem);
      bActuateByTFD = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(CCJ_SendStepToCCJ):
      pthread_mutex_lock(&mutexSmem);
      bEnableSendStepToCCJ = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(NextStepForceEnable):
      pthread_mutex_lock(&mutexSmem);
      bNextStepForceEnable = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(EnableUpdateRTC):
      pthread_mutex_lock(&mutexSmem);
      bEnableUpdateRTC = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(IFGetResetTCSignal):
      pthread_mutex_lock(&mutexSmem);
      bGetResetTCSignal = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    case(TCSegmentTypeUpdate):
      pthread_mutex_lock(&mutexSmem);
      bTCSegmentTypeUpdate = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

      //OT20140415
    case(TCDynSegSwitch):
      pthread_mutex_lock(&mutexSmem);
      bDynSegSwitch = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;
    case(TCDynSegStatus):
      pthread_mutex_lock(&mutexSmem);
      bDynSegStatus = bTMP;
      pthread_mutex_unlock(&mutexSmem);
      break;

    default:
      return false;
      break;
  }

  return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSaveLastAliveStatus(void)                                           //��SS�_�u�ɡA��interval�Ӭ����ɶ�toDOM
{
try {
    time_t NowSec = time(NULL);
    disk.vWriteLastAliveTimeStampFile(NowSec);
    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
int SMEM::vGetFirmwareInfo_0FC3(const int iSwitch)                                       // 0: Year, 1: Month, 2: Day, 3: CompanyID. 4: FirstVersion, 5: SecondVersion
{
try {
  int iTmp;
  pthread_mutex_lock(&mutexSmem);
  switch(iSwitch) {
    case(0):
      iTmp = iFirmwareYear;
      break;
    case(1):
      iTmp = iFirmwareMonth;
      break;
    case(2):
      iTmp = iFirmwareDay;
      break;
    case(3):
      if(ucW77E58FWVer[0] == 0 && ucW77E58FWVer[1] == 0 && ucW77E58FWVer[2] == 0) {  //unknow
        iFirmwareCompanyID = 0;
      } else if(ucW77E58FWVer[0] == 9 && ucW77E58FWVer[1] == 6 && ucW77E58FWVer[2] == 23) {  //2009/6/23
        iFirmwareCompanyID = 1;
      } else {  //other
        iFirmwareCompanyID = 255;
      }
      iTmp = iFirmwareCompanyID;
      break;
    case(4):
      iTmp = iFirmwareFirstVersion;
      break;
    case(5):
      iTmp = iFirmwareSecondVersion;
      break;
  }
  pthread_mutex_unlock(&mutexSmem);

  return iTmp;
  } catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vGetTCPhasePlanSegTypeData(const int iSwitch, const int iNumber)
{
try {
  bool bTmp;
  pthread_mutex_lock(&mutexSmem);
  switch(iSwitch) {
    case(TC_Phase):
      bTmp = bSmemTC_PhaseAlive[iNumber];
      break;

    case(TC_Plan):
      bTmp = bSmemTC_PlanAlive[iNumber];
      break;

    case(TC_SegType):
      bTmp = bSmemTC_SegTypeAlive[iNumber];
      break;

    default:
      bTmp = false;
      break;
    }
    pthread_mutex_unlock(&mutexSmem);

    return bTmp;
  } catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSetTCPhasePlanSegTypeData(const int iSwitch, const int iNumber, const bool bStat)
{
try {
  pthread_mutex_lock(&mutexSmem);
  switch(iSwitch) {
    case(TC_Phase):
      bSmemTC_PhaseAlive[iNumber] = bStat;
      break;

    case(TC_Plan):
      bSmemTC_PlanAlive[iNumber] = bStat;
      break;

    case(TC_SegType):
      bSmemTC_SegTypeAlive[iNumber] = bStat;
      break;

    default:
      break;
    }
    pthread_mutex_unlock(&mutexSmem);

    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vReadUDPMappingLCNDataFromStorage()
{
try{

  for(int i = 0; i < MaxOpenUDPPort; i++)
  {
    bzero(_sUDPMappingLCN[i].cLocalIP, sizeof(_sUDPMappingLCN[i].cLocalIP));
    bzero(_sUDPMappingLCN[i].cRemoteIP, sizeof(_sUDPMappingLCN[i].cRemoteIP));
    _sUDPMappingLCN[i].iListenPort = 0;
    _sUDPMappingLCN[i].iSendPort = 0;
    _sUDPMappingLCN[i].iMachineLCN = 0;
  }

  FILE *IP_FD=NULL;

  char cReadString[255];
  char cTMP[255];
  int iTMP;
  int iTMPPtr;
  int iSection;

  int iStrLen;
  int iLine = 0;

  bzero(cReadString, sizeof(cReadString));
  bzero(cTMP, sizeof(cTMP));

  IP_FD = fopen("//cct//Data//SETTING//IP.txt" , "r"); //fopen return NULL if file not exist
  if(IP_FD) {
  //OTMARKPRINTF  printf("open ok\n");

    while(fscanf(IP_FD, "%s", cReadString)!=EOF) {
       iTMPPtr = 0;
       iSection = 0;

       iStrLen = strlen(cReadString);

       for(int i = 0 ;i < strlen(cReadString); i++)
       {
         if(cReadString[i] != ':' && cReadString[i] != '\n' ) {
           cTMP[iTMPPtr] = cReadString[i];
           iTMPPtr++;
         }
         else
         {
           if(iSection == 0) {
             strncpy( _sUDPMappingLCN[iLine].cLocalIP, cTMP, 15);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].cLocalIP:%s\n", iLine, _sUDPMappingLCN[iLine].cLocalIP);
           }

           if(iSection == 1) {
             _sUDPMappingLCN[iLine].iListenPort = atoi(cTMP);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].iListenPort:%d\n", iLine, _sUDPMappingLCN[iLine].iListenPort);
           }

           if(iSection == 2) {
             strncpy( _sUDPMappingLCN[iLine].cRemoteIP, cTMP, 15);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].cRemoteIP:%s\n", iLine, _sUDPMappingLCN[iLine].cRemoteIP);
           }

           if(iSection == 3) {
             _sUDPMappingLCN[iLine].iSendPort = atoi(cTMP);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].iSendPort:%d\n", iLine, _sUDPMappingLCN[iLine].iSendPort);
           }

           if(iSection == 4) {
             _sUDPMappingLCN[iLine].iMachineLCN = atoi(cTMP);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].iMachineLCN:%d\n", iLine, _sUDPMappingLCN[iLine].iMachineLCN);
           }
           iSection++;
           iTMPPtr = 0;
           bzero(cTMP, sizeof(cTMP));
         }
       }
       iLine++;
    }
  }

//OT Debug 0523
  else {
  //OTMARKPRINTF  printf("open failure\n");
  }

  //printfMsg
  for(int i = 0 ; i < MaxOpenUDPPort; i++) {
    printf("%s:%d %s:%d %d\n",
    _sUDPMappingLCN[i].cLocalIP,
    _sUDPMappingLCN[i].iListenPort,
    _sUDPMappingLCN[i].cRemoteIP,
    _sUDPMappingLCN[i].iSendPort,
    _sUDPMappingLCN[i].iMachineLCN);
  }

  if(IP_FD)
    fclose( IP_FD );

//OTMARKPRINTF  printf("");

  return true;
}catch(...){}
}

//---------------------------------------------------------------------------
tsUDPMappingLCN SMEM::vGetUDPMappingLCNData(int iSelect)
{
try{
  tsUDPMappingLCN _sTMP;

  pthread_mutex_lock(&mutexSmem);
  _sTMP = _sUDPMappingLCN[iSelect];
  pthread_mutex_unlock(&mutexSmem);

  return _sTMP;
}catch(...){}
}


//---------------------------------------------------------------------------
unsigned short int SMEM::vGetWayMappingRedCount(unsigned short int usiQuery)
{
try{
  unsigned short int usiTMP;
  pthread_mutex_lock(&mutexSmem);
  usiTMP = usiWayMappingRedCount[usiQuery];
  pthread_mutex_unlock(&mutexSmem);
  return usiTMP;
}catch(...){}
}

//---------------------------------------------------------------------------
unsigned short int SMEM::vSetWayMappingRedCount(unsigned short int usiSet, unsigned short int usiVar)
{
try{
  pthread_mutex_lock(&mutexSmem);
  usiWayMappingRedCount[usiSet] = usiVar;
  if(usiSet>=7) vSave92COMMToDisk();                                                      //SaveToDOM
  pthread_mutex_unlock(&mutexSmem);
  return true;
}catch(...){}
}

//OT PASS
//---------------------------------------------------------------------------
bool SMEM::vSetACK_W2W(unsigned char ucSEQ, unsigned char ucDevFlow)
{
try{

  pthread_mutex_lock(&mutexSmem);
  ucACKW2W[ucSEQ] = ucDevFlow;
  pthread_mutex_unlock(&mutexSmem);
  return true;

}catch(...){}
}
//---------------------------------------------------------------------------
unsigned char SMEM::vGetACK_W2W(unsigned char ucSEQ)
{
try{
  unsigned char ucRet;
  pthread_mutex_lock(&mutexSmem);
  ucRet = ucACKW2W[ucSEQ];
  pthread_mutex_unlock(&mutexSmem);
  return ucRet;
}catch(...){}
}
//---------------------------------------------------------------------------
bool SMEM::vSetPassMode(unsigned char ucMode)
{
try{

  pthread_mutex_lock(&mutexSmem);
  ucPassMode_0F8E8F = ucMode;
  // should write to file
  vSave92COMMToDisk();
  pthread_mutex_unlock(&mutexSmem);
  return true;
}catch(...){}
}
//---------------------------------------------------------------------------
unsigned char SMEM::vGetPassMode(void)
{
try{
  unsigned char ucRet;
  pthread_mutex_lock(&mutexSmem);
  ucRet = ucPassMode_0F8E8F;
  pthread_mutex_unlock(&mutexSmem);
  return ucRet;
}catch(...){}
}
//---------------------------------------------------------------------------
bool SMEM::vSetPassServerLCN(unsigned short int usiPSL)
{
try{

  pthread_mutex_lock(&mutexSmem);
  usiPassServerLCN = usiPSL;
  // should write to file
  vSave92COMMToDisk();
  pthread_mutex_unlock(&mutexSmem);
  return true;
}catch(...){}
}
//---------------------------------------------------------------------------
unsigned short int SMEM::vGetPassServerLCN(void)
{
try{
  unsigned short int usiRet;
  pthread_mutex_lock(&mutexSmem);
  usiRet = usiPassServerLCN;
  pthread_mutex_unlock(&mutexSmem);
  return usiRet;
}catch(...){}
}


//OT Debug Direct
//---------------------------------------------------------------------------
unsigned short int SMEM::vGetSignamMapMappingDir(unsigned short int usiQuery)
{
try{
  unsigned short int usiTMP;
  pthread_mutex_lock(&mutexSmem);
  usiTMP = usiSignamMapMappingDir[usiQuery];
  pthread_mutex_unlock(&mutexSmem);
  return usiTMP;
}catch(...){}
}

//---------------------------------------------------------------------------
unsigned short int SMEM::vSetSignamMapMappingDir(unsigned short int usiSet, unsigned short int usiVar)
{
try{
  pthread_mutex_lock(&mutexSmem);
  usiSignamMapMappingDir[usiSet] = usiVar;
  if(usiSet>=7) vSave92COMMToDisk();                                                          //SaveToDOM
  pthread_mutex_unlock(&mutexSmem);
  return true;
}catch(...){}
}
//---------------------------------------------------------------------------
unsigned short int SMEM::vGetSignamMapMappingLightBoard(unsigned short int usiQuery)
{
try{
  unsigned short int usiTMP;
  pthread_mutex_lock(&mutexSmem);
  usiTMP = usiSignamMapMappingLightBoard[usiQuery];
  pthread_mutex_unlock(&mutexSmem);
  return usiTMP;
}catch(...){}
}

//---------------------------------------------------------------------------
unsigned short int SMEM::vSetSignamMapMappingLightBoard(unsigned short int usiSet, unsigned short int usiVar)
{
try{
  pthread_mutex_lock(&mutexSmem);
  usiSignamMapMappingLightBoard[usiSet] = usiVar;
  if(usiSet>=5) vSave92COMMToDisk();                                            //SaveToDOM
  pthread_mutex_unlock(&mutexSmem);
  return true;
}catch(...){}
}

//----------------------------------------------------------------------
unsigned short int SMEM::vGetScreenCopyPlanID(void)
{
try {
    unsigned short int usiTMP;
    pthread_mutex_lock(&mutexSmem);
    usiTMP = usiScreenCopyPlanID;
    pthread_mutex_unlock(&mutexSmem);
    return usiTMP;
  } catch(...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vSetScreenCopyPlanID(unsigned short int usiIN)
{
try{
  pthread_mutex_lock(&mutexSmem);
  usiScreenCopyPlanID = usiIN;
  pthread_mutex_unlock(&mutexSmem);
  return true;
}catch(...){}
}
//----------------------------------------------------------------------
unsigned short int SMEM::vGetScreenCopySegID(void)
{
try {
    unsigned short int usiTMP;
    pthread_mutex_lock(&mutexSmem);
    usiTMP = usiScreenCopySegID;
    pthread_mutex_unlock(&mutexSmem);
    return usiTMP;
  } catch(...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vSetScreenCopySegID(unsigned short int usiIN)
{
try{
  pthread_mutex_lock(&mutexSmem);
  usiScreenCopySegID = usiIN;
  pthread_mutex_unlock(&mutexSmem);
  return true;
}catch(...){}
}

//---------------------------------------------------------------------------
bool SMEM::vReadCCJDynCtlFromStorage()
{
try{

  bzero(_sCCJDynCtl.cLocalIP, sizeof(_sCCJDynCtl.cLocalIP));
  bzero(_sCCJDynCtl.cRemoteIP, sizeof(_sCCJDynCtl.cRemoteIP));
  _sCCJDynCtl.iListenPort = 0;
  _sCCJDynCtl.iSendPort = 0;
  _sCCJDynCtl.iMachineLCN = 0;

  FILE *IP_FD=NULL;

  char cReadString[255];
  char cTMP[255];
  int iTMP;
  int iTMPPtr;
  int iSection;

  int iStrLen;

  bzero(cReadString, sizeof(cReadString));
  bzero(cTMP, sizeof(cTMP));

  IP_FD = fopen("//cct//Data//SETTING//CCJDynCtlIP.txt" , "r"); //fopen return NULL if file not exist
  if(IP_FD) {
  //OTMARKPRINTF  printf("open ok\n");

    while(fscanf(IP_FD, "%s", cReadString)!=EOF) {
       iTMPPtr = 0;
       iSection = 0;

       iStrLen = strlen(cReadString);

       for(int i = 0 ;i < strlen(cReadString); i++)
       {
         if(cReadString[i] != ':' && cReadString[i] != '\n' ) {
           cTMP[iTMPPtr] = cReadString[i];
           iTMPPtr++;
         }
         else
         {
           if(iSection == 0) {
             strncpy( _sCCJDynCtl.cLocalIP, cTMP, 15);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].cLocalIP:%s\n", iLine, _sUDPMappingLCN[iLine].cLocalIP);
           }

           if(iSection == 1) {
             _sCCJDynCtl.iListenPort = atoi(cTMP);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].iListenPort:%d\n", iLine, _sUDPMappingLCN[iLine].iListenPort);
           }

           if(iSection == 2) {
             strncpy( _sCCJDynCtl.cRemoteIP, cTMP, 15);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].cRemoteIP:%s\n", iLine, _sUDPMappingLCN[iLine].cRemoteIP);
           }

           if(iSection == 3) {
             _sCCJDynCtl.iSendPort = atoi(cTMP);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].iSendPort:%d\n", iLine, _sUDPMappingLCN[iLine].iSendPort);
           }

           if(iSection == 4) {
             _sCCJDynCtl.iMachineLCN = atoi(cTMP);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].iMachineLCN:%d\n", iLine, _sUDPMappingLCN[iLine].iMachineLCN);
           }
           iSection++;
           iTMPPtr = 0;
           bzero(cTMP, sizeof(cTMP));
         }
       }
       printf("CCJ IP:%s Read OK\n", _sCCJDynCtl.cRemoteIP);
    }
  }

//OT Debug 0523
  else {
  //OTMARKPRINTF  printf("open failure\n");
  }

  //printfMsg
    printf("%s:%d %s:%d %d\n",
    _sCCJDynCtl.cLocalIP,
    _sCCJDynCtl.iListenPort,
    _sCCJDynCtl.cRemoteIP,
    _sCCJDynCtl.iSendPort,
    _sCCJDynCtl.iMachineLCN);

  if(IP_FD)
    fclose( IP_FD );


  return true;
}catch(...){}
}

//---------------------------------------------------------------------------
bool SMEM::vReadCCJDynCtl2FromStorage()
{
try{

  bzero(_sCCJDynCtl2.cLocalIP, sizeof(_sCCJDynCtl2.cLocalIP));
  bzero(_sCCJDynCtl2.cRemoteIP, sizeof(_sCCJDynCtl2.cRemoteIP));
  _sCCJDynCtl2.iListenPort = 0;
  _sCCJDynCtl2.iSendPort = 0;
  _sCCJDynCtl2.iMachineLCN = 0;

  FILE *IP_FD=NULL;

  char cReadString[255];
  char cTMP[255];
  int iTMP;
  int iTMPPtr;
  int iSection;

  int iStrLen;

  bzero(cReadString, sizeof(cReadString));
  bzero(cTMP, sizeof(cTMP));

  IP_FD = fopen("//cct//Data//SETTING//CCJDynCtl2IP.txt" , "r"); //fopen return NULL if file not exist
  if(IP_FD) {
  //OTMARKPRINTF  printf("open ok\n");

    while(fscanf(IP_FD, "%s", cReadString)!=EOF) {
       iTMPPtr = 0;
       iSection = 0;

       iStrLen = strlen(cReadString);

       for(int i = 0 ;i < strlen(cReadString); i++)
       {
         if(cReadString[i] != ':' && cReadString[i] != '\n' ) {
           cTMP[iTMPPtr] = cReadString[i];
           iTMPPtr++;
         }
         else
         {
           if(iSection == 0) {
             strncpy( _sCCJDynCtl2.cLocalIP, cTMP, 15);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].cLocalIP:%s\n", iLine, _sUDPMappingLCN[iLine].cLocalIP);
           }

           if(iSection == 1) {
             _sCCJDynCtl2.iListenPort = atoi(cTMP);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].iListenPort:%d\n", iLine, _sUDPMappingLCN[iLine].iListenPort);
           }

           if(iSection == 2) {
             strncpy( _sCCJDynCtl2.cRemoteIP, cTMP, 15);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].cRemoteIP:%s\n", iLine, _sUDPMappingLCN[iLine].cRemoteIP);
           }

           if(iSection == 3) {
             _sCCJDynCtl2.iSendPort = atoi(cTMP);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].iSendPort:%d\n", iLine, _sUDPMappingLCN[iLine].iSendPort);
           }

           if(iSection == 4) {
             _sCCJDynCtl2.iMachineLCN = atoi(cTMP);
           //OTMARKPRINTF  printf("_sUDPMappingLCN[%d].iMachineLCN:%d\n", iLine, _sUDPMappingLCN[iLine].iMachineLCN);
           }
           iSection++;
           iTMPPtr = 0;
           bzero(cTMP, sizeof(cTMP));
         }
       }
       printf("CCJ IP2:%s Read OK\n", _sCCJDynCtl2.cRemoteIP);
    }
  }

//OT Debug 0523
  else {
  //OTMARKPRINTF  printf("open failure\n");
  }

  //printfMsg
    printf("%s:%d %s:%d %d\n",
    _sCCJDynCtl2.cLocalIP,
    _sCCJDynCtl2.iListenPort,
    _sCCJDynCtl2.cRemoteIP,
    _sCCJDynCtl2.iSendPort,
    _sCCJDynCtl2.iMachineLCN);

  if(IP_FD)
    fclose( IP_FD );


  return true;
}catch(...){}
}


//---------------------------------------------------------------------------
tsUDPMappingLCN SMEM::vGetCCJDynCtlIPData(void)
{
try{
  tsUDPMappingLCN _sTMP;

  pthread_mutex_lock(&mutexSmem);
  _sTMP = _sCCJDynCtl;
  pthread_mutex_unlock(&mutexSmem);

  return _sTMP;
}catch(...){}
}



//---------------------------------------------------------------------------
tsUDPMappingLCN SMEM::vGetCCJDynCtl2IPData(void)
{
try{
  tsUDPMappingLCN _sTMP;

  pthread_mutex_lock(&mutexSmem);
  _sTMP = _sCCJDynCtl2;
  pthread_mutex_unlock(&mutexSmem);

  return _sTMP;
}catch(...){}
}


//---------------------------------------------------------------------------
bool SMEM::vReadLCNFromDisk(void)
{
try {
    disk.vReadLCNFromFile(  &bEnableLCNUsingStorage,
                                &uiLCNUsingStorage
                               );

    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSaveLast92TC_5F15Time(void)
{
try {
    time_t TC5F15Sec = time(NULL);
    lLast92TC_5F15Time = TC5F15Sec;
    disk.vWriteLast92TC_5F15Time(TC5F15Sec);
    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vLoadLast92TC_5F15Time(void)
{
try {
    time_t TC5F15Sec;
    disk.vReadLast92TC_5F15Time(&TC5F15Sec);
    lLast92TC_5F15Time = TC5F15Sec;

    return true;
  } catch(...) {}
}

//---------------------------------------------------------------------------
YMDHMS SMEM::vGetLast92TC_5F15Time(void)
{
try {
    YMDHMS tmp;
    struct tm *tm = localtime(&lLast92TC_5F15Time);
    pthread_mutex_lock(&mutexSmem);
    tmp.Year = tm->tm_year - 11;
    tmp.Month = tm->tm_mon + 1;
    tmp.Day = tm->tm_mday;
    tmp.Hour = tm->tm_hour;
    tmp.Min = tm->tm_min;
    tmp.Sec = tm->tm_sec;
    pthread_mutex_unlock(&mutexSmem);

    return tmp;
  } catch(...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vCmpGreenConflict(void)
{
try {
    bool bRet = false;
    pthread_mutex_lock(&mutexSmem);
    if(ucGreenConflictDetFromDIO == 0xFF) { bRet = true; }
    else if(ucGreenConflictDetFromDIO == 192) { bRet = true; }
    else if(ucGreenConflictDetFromDIO == ucGreenConflictDetFromCSTC) { bRet = true; }
    pthread_mutex_unlock(&mutexSmem);

//    printf("[OTMsg] GreenConflictDec %X %X\n", ucGreenConflictDetFromCSTC, ucGreenConflictDetFromDIO);
    return bRet;
  } catch(...) {}
}

//---------------------------------------------------------------------------
sChildChain SMEM::vGetChildChainStruct(void)
{
try {
  sChildChain sCCTMP;
  pthread_mutex_lock(&mutexSmem);
  sCCTMP = sSMEMChildChain;
  pthread_mutex_unlock(&mutexSmem);
  return sCCTMP;
} catch(...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSetChildChainStruct(sChildChain sCCSet)
{
try {
  pthread_mutex_lock(&mutexSmem);
  sSMEMChildChain = sCCSet;
  pthread_mutex_unlock(&mutexSmem);
  return true;
} catch(...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vClearChildChainStruct(void)
{
try {
  pthread_mutex_lock(&mutexSmem);
  sSMEMChildChain.bHaveReasonableChildChainSignal = false;
  sSMEMChildChain.oldStartTime = 0;
  sSMEMChildChain.newStartTime = 0;
  sSMEMChildChain.oldEndTime = 0;
  sSMEMChildChain.newEndTime = 0;
  sSMEMChildChain.iChainCycle = 0;
  sSMEMChildChain.iStartKeepTime = 0;
  sSMEMChildChain.iEndKeepTime = 0;
  pthread_mutex_unlock(&mutexSmem);
  return true;
} catch(...) {}
}

//----------------------------------------------------------------------
int SMEM::vGetChainOffset(const int type, const int iSelect)
{
try {
  int iRet;
  switch(type) {

    case(1):                                                                    //Start
      pthread_mutex_lock(&mutexSmem);
      iRet = siTC92_5F33StartOffset[iSelect];
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(2):                                                                    //End
      pthread_mutex_lock(&mutexSmem);
      iRet = siTC92_5F33EndOffset[iSelect];
      pthread_mutex_unlock(&mutexSmem);
    break;

    default:
    break;
  }
  return iRet;
} catch(...) {}
}

//----------------------------------------------------------------------
bool SMEM::vSetChainOffset(const int type, const int iSelect, short int iValue)
{
try {
  int iRet;
  switch(type) {

    case(1):                                                                    //Start
      pthread_mutex_lock(&mutexSmem);
      siTC92_5F33StartOffset[iSelect] = iValue;
      pthread_mutex_unlock(&mutexSmem);
    break;

    case(2):                                                                    //End
      pthread_mutex_lock(&mutexSmem);
      siTC92_5F33EndOffset[iSelect] = iValue;
      pthread_mutex_unlock(&mutexSmem);
      smem.vWriteChainDataFromStroage();
    break;

    default:
      return false;
    break;
  }
  return true;
} catch(...) {}
}

//----------------------------------------------------------------------
bool SMEM::vWriteDIO(unsigned char cDigitalOUT)
{
try {
  pthread_mutex_lock(&mutexSmem);
  ucSendDigitalIO = cDigitalOUT;
  pthread_mutex_unlock(&mutexSmem);

  digitalIO.WriteDigitalIO(cDigitalOUT);
  return true;
} catch(...) {}
}

//----------------------------------------------------------------------
unsigned char SMEM::vGetWriteDIO(void)
{
try {
  unsigned char ucRet;
  pthread_mutex_lock(&mutexSmem);
  ucRet = ucSendDigitalIO;
  pthread_mutex_unlock(&mutexSmem);
  return ucRet;

} catch(...) {}
}

//----------------------------------------------------------------------
bool SMEM::vWriteChainDataFromStroage(void)
{
try {
  disk.vWriteChainSettFile(ucTC92_5F31Manual,
                           ucTC92_5F31TOD,
                           ucTC92_5F31StartSubPhaseId,
                           ucTC92_5F31EndSubPhaseId,
                           ucTC92_5F32StartSubPhaseId,
                           ucTC92_5F32EndSubPhaseId,
                           ucTC_MotherChainStartStepId,
                           ucTC_MotherChainEndStepId,
                           siTC92_5F33StartOffset,
                           siTC92_5F33EndOffset
                         ) ;

  return true;

} catch(...) {}
}

//----------------------------------------------------------------------
bool SMEM::vReadChainDataFromStroage(void)
{
try {
  disk.vReadChainSettFile( &ucTC92_5F31Manual,
                           &ucTC92_5F31TOD,
                           &ucTC92_5F31StartSubPhaseId,
                           &ucTC92_5F31EndSubPhaseId,
                           &ucTC92_5F32StartSubPhaseId,
                           &ucTC92_5F32EndSubPhaseId,
                           &ucTC_MotherChainStartStepId,
                           &ucTC_MotherChainEndStepId,
                           siTC92_5F33StartOffset,
                           siTC92_5F33EndOffset
                         ) ;

  return true;

} catch(...) {}
}


//---------------------------------------------------------------------------
bool SMEM::SetLocalIP1(int idx,int value)
{
try {
    pthread_mutex_lock(&mutexSmem);
    switch (idx) {
        case 1:    localIp1_1=value;    break;
        case 2:    localIp1_2=value;    break;
        case 3:    localIp1_3=value;    break;
        case 4:    localIp1_4=value;    break;
        case 5:    localPort1=value;    break;
        default:
        break;
    }
    vSave92COMMToDisk();
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::SetdistIp0(int idx,int value)
{
try {
    pthread_mutex_lock(&mutexSmem);
    switch (idx) {
        case 1:    distIp0_1=value;    break;
        case 2:    distIp0_2=value;    break;
        case 3:    distIp0_3=value;    break;
        case 4:    distIp0_4=value;    break;
        case 5:    dist0Port=value;    break;
        default:
        break;
    }
    vSave92COMMToDisk();
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::SetDistIP(int idx,int value)
{
try {
    pthread_mutex_lock(&mutexSmem);
    switch (idx) {
        case 1:    distIp1=value;    break;
        case 2:    distIp2=value;    break;
        case 3:    distIp3=value;    break;
        case 4:    distIp4=value;    break;
        case 5:    distPort=value;    break;
        default:
        break;
    }
    vSave92COMMToDisk();
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::SetNetmask(int idx,int value)
{
try {
    pthread_mutex_lock(&mutexSmem);
    switch (idx) {
        case 1:    netmask1=value;    break;
        case 2:    netmask2=value;    break;
        case 3:    netmask3=value;    break;
        case 4:    netmask4=value;    break;
        default:
        break;
    }
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}


//---------------------------------------------------------------------------
bool SMEM::SetGateway(int idx,int value)
{
try {
    pthread_mutex_lock(&mutexSmem);
    switch (idx) {
        case 1:    gateway1=value;    break;
        case 2:    gateway2=value;    break;
        case 3:    gateway3=value;    break;
        case 4:    gateway4=value;    break;
        default:
        break;
    }
    vSave92COMMToDisk();
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
int SMEM::GetLocalIP1(int idx)
{
try {

    int temp=0;
    pthread_mutex_lock(&mutexSmem);
    switch (idx) {
        case 1:    temp=localIp1_1;    break;
        case 2:    temp=localIp1_2;    break;
        case 3:    temp=localIp1_3;    break;
        case 4:    temp=localIp1_4;    break;
        case 5:    temp=localPort1;    break;
        default:   temp=0;    break;
    }
    pthread_mutex_unlock(&mutexSmem);
    return temp;
  } catch (...) {}
}
//---------------------------------------------------------------------------
int SMEM::GetdistIp0(int idx)
{
try {
    int temp=0;
    pthread_mutex_lock(&mutexSmem);
    switch (idx) {
        case 1:    temp=distIp0_1;    break;
        case 2:    temp=distIp0_2;    break;
        case 3:    temp=distIp0_3;    break;
        case 4:    temp=distIp0_4;    break;
        case 5:    temp=dist0Port;    break;
        default:   temp=0;    break;
    }
    pthread_mutex_unlock(&mutexSmem);
    return temp;
  } catch (...) {}
}
//---------------------------------------------------------------------------
int SMEM::GetDistIP(int idx)
{
try {
    int temp=0;
    pthread_mutex_lock(&mutexSmem);
    switch (idx) {
        case 1:    temp=distIp1;    break;
        case 2:    temp=distIp2;    break;
        case 3:    temp=distIp3;    break;
        case 4:    temp=distIp4;    break;
        case 5:    temp=distPort;    break;
        default:   temp=0;    break;
    }
    pthread_mutex_unlock(&mutexSmem);
    return temp;
  } catch (...) {}
}

//---------------------------------------------------------------------------
int SMEM::GetNetmask(int idx)
{
try {
    int temp=0;
    pthread_mutex_lock(&mutexSmem);
    switch (idx) {
        case 1:    temp=netmask1;    break;
        case 2:    temp=netmask2;    break;
        case 3:    temp=netmask3;    break;
        case 4:    temp=netmask4;    break;
        default:   temp=0;    break;
    }
    pthread_mutex_unlock(&mutexSmem);
    return temp;
  } catch (...) {}
}

//---------------------------------------------------------------------------
int SMEM::GetGateway(int idx)
{
try {
    int temp=0;
    pthread_mutex_lock(&mutexSmem);
    switch (idx) {
        case 1:    temp=gateway1;    break;
        case 2:    temp=gateway2;    break;
        case 3:    temp=gateway3;    break;
        case 4:    temp=gateway4;    break;
        default:   temp=0;    break;
    }
    pthread_mutex_unlock(&mutexSmem);
    return temp;
  } catch (...) {}
}



//---------------------------------------------------------------------------
void SMEM::vSendRequestToKeypad(void)
{
try {
  MESSAGEOK messageOut;
  messageOut.protocol=PROTOCOLKEYPAD;
  messageOut.packetLength=6;
  messageOut.cksStatus=true;
  messageOut.success=true;
  messageOut.ReadorWrite=cWRITE;

  messageOut.packet[0]=0xAA;
  messageOut.packet[1]=0xBB;
  messageOut.packet[2]=0x03;
  messageOut.packet[3]=0xAA;
  messageOut.packet[4]=0xCC;
  messageOut.packet[5]=0;
  for (int i=0;i<5;i++) messageOut.packet[5]^=messageOut.packet[i];

//  writeJob.WriteWorkByMESSAGEOUT(messageOut);
  writeJob.WritePhysicalOut(messageOut.packet, 6, DEVICEKEYPAD);

  } catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSetMotherBoardType(unsigned char ucMBTypeIn)
{
try{

  pthread_mutex_lock(&mutexSmem);
//  ucMBType = ucMBTypeIn;
//Only Read from boot
  disk.vWriteMotherBoardTypeFromFile(ucMBTypeIn);
  pthread_mutex_unlock(&mutexSmem);
  return true;

}catch(...){}
}
//---------------------------------------------------------------------------
unsigned char SMEM::vGetMotherBoardType(void)
{
try{
  unsigned char ucRet;
  pthread_mutex_lock(&mutexSmem);
//  disk.vReadMotherBoardTypeFromFile(&ucMBType);
  ucRet = ucMBType;
  pthread_mutex_unlock(&mutexSmem);
  return ucRet;
}catch(...){}
}

//---------------------------------------------------------------------------
bool SMEM::vSetActuatePhaseExtend(unsigned char ucIn)
{
try{

  pthread_mutex_lock(&mutexSmem);
  ucActuatePhaseExtend = ucIn;
  vSave92TCSettingToDisk();
  pthread_mutex_unlock(&mutexSmem);
  return true;

}catch(...){}
}
//---------------------------------------------------------------------------
unsigned char SMEM::vGetActuatePhaseExtend(void)
{
try{
  unsigned char ucRet;
  pthread_mutex_lock(&mutexSmem);
  ucRet = ucActuatePhaseExtend;
  pthread_mutex_unlock(&mutexSmem);
  return ucRet;
}catch(...){}
}

//----------------------------------------------------------
bool SMEM::vSendTimerUpdateToCCJ_5F9E(void)
{
try {

  unsigned char data[5];
  MESSAGEOK _MsgOK;

  data[0] = 0x5F;
  data[1] = 0x9E;

  _MsgOK = oDataToMessageOK.vPackageINFOTo92Protocol(data, 2, true);
  _MsgOK.InnerOrOutWard = cOutWard;
  writeJob.WritePhysicalOut(_MsgOK.packet, _MsgOK.packetLength, DEVICECCJDYNCTL);

  return true;

  return 0;
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSetVDUpdateDB(int iTMP, bool bStatus)
{
try {
  pthread_mutex_lock(&mutexSmem);

  if(iTMP == 0) { b92VDUpdateDB_b0 = bStatus; b92VDUpdateDB_Status = true; }
  if(iTMP == 1) { b92VDUpdateDB_b1 = bStatus; b92VDUpdateDB_Status = true; }
  if(iTMP == 2) { b92VDUpdateDB_b2 = bStatus; b92VDUpdateDB_Status = true; }
  if(iTMP == 3) { b92VDUpdateDB_b3 = bStatus; b92VDUpdateDB_Status = true; }
  if(iTMP == 888) { b92VDUpdateDB_Status = bStatus; }

  pthread_mutex_unlock(&mutexSmem);

  printf("[OT Msg] VDUpdateDB refresh, now: %X\n", b92VDUpdateDB_Status);

  return true;
} catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vGetVDUpdateDB(int iTMP)
{
try {
  bool iRet;
  pthread_mutex_lock(&mutexSmem);
  if(iTMP == 888)
    iRet = b92VDUpdateDB_Status;
  else if(iTMP == 0)
    iRet = b92VDUpdateDB_b0;
  else if(iTMP == 1)
    iRet = b92VDUpdateDB_b1;
  else if(iTMP == 2)
    iRet = b92VDUpdateDB_b2;
  else if(iTMP == 3)
    iRet = b92VDUpdateDB_b3;
  pthread_mutex_unlock(&mutexSmem);

  return iRet;
} catch (...) {}
}


//---------------------------------------------------------------------------
bool SMEM::vSetSignalConflictErrorVar(unsigned char ucTmp1, unsigned char ucTmp2, unsigned char ucTmp3)
{
try {
  ucSignalConflictErrorVar[0] = ucTmp1;
  ucSignalConflictErrorVar[1] = ucTmp2;
  ucSignalConflictErrorVar[2] = ucTmp3;
  return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vGetSignalConflictErrorVar(unsigned char *ucPTmp)
{
try {
  ucPTmp[0] = ucSignalConflictErrorVar[0];
  ucPTmp[1] = ucSignalConflictErrorVar[1];
  ucPTmp[2] = ucSignalConflictErrorVar[2];
  return true;
} catch (...) {}
}


//---------------------------------------------------------------------------
bool SMEM::vTrafficLightSendAndCheck(void)
{
try {
  char cTMP[256];

  if(ucW77E58FWVer[0] == 0 && ucW77E58FWVer[1] == 0 && ucW77E58FWVer[2] == 0) {
  } else {

    if(uiTrafficLightSend != uiTrafficLightACK && uiTrafficLightACK != 0) {
    //error happened.

      sprintf(cTMP, "LightCtlBoardError:%d,%d", uiTrafficLightSend, uiTrafficLightACK);
      vWriteMsgToDOM(cTMP);
      printf("%s\n", cTMP);
      pthread_mutex_lock(&mutexSmem);
      uiTrafficLightSend = 0;  //clear
      uiTrafficLightACK = 0;  //clear

    //980623
//    smem.vSetUCData(TC_Redcount_Display_Enable, 1);
      ucDisplayRedcount = 0;
      pthread_mutex_unlock(&mutexSmem);
      stc.CalculateAndSendRedCount(10);
    }
    else if(uiTrafficLightSend == uiTrafficLightACK && uiTrafficLightACK > 1) {
      pthread_mutex_lock(&mutexSmem);
      ucDisplayRedcount = 1;
      pthread_mutex_unlock(&mutexSmem);
    }

    pthread_mutex_lock(&mutexSmem);
    ++uiTrafficLightSend;
    pthread_mutex_unlock(&mutexSmem);

  }

  return true;
} catch (...) {}
}


//---------------------------------------------------------------------------
bool SMEM::vTrafficLightAckAdd(void)
{
try {
  pthread_mutex_lock(&mutexSmem);
  ++uiTrafficLightACK;
  pthread_mutex_unlock(&mutexSmem);
return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
time_t SMEM::GetRevManualPadTime(void)
{
try {
    time_t tempTimer;
    pthread_mutex_lock(&mutexSmem);
    tempTimer = lastRevManualPadTime;
    pthread_mutex_unlock(&mutexSmem);
    return tempTimer;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::SetRevManualPadTime(void)
{
try {
    pthread_mutex_lock(&mutexSmem);
    lastRevManualPadTime = time(NULL);
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
unsigned char SMEM::vGetTimerMutexCSTC(void)
{
try {
    unsigned char ucRet;
    pthread_mutex_lock(&mutexSmem);
    ucRet = ucTimerCSTC;
    pthread_mutex_unlock(&mutexSmem);
    return ucRet;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vSetTimerMutexCSTC(unsigned char ucStatus)
{
try {
    pthread_mutex_lock(&mutexSmem);
    ucTimerCSTC = ucStatus;
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
unsigned char SMEM::vGetTimerMutexCTIMER(void)
{
try {
    unsigned char ucRet;
    pthread_mutex_lock(&mutexSmem);
    ucRet = ucTimerCTIMER;
    pthread_mutex_unlock(&mutexSmem);
    return ucRet;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vSetTimerMutexCTIMER(unsigned char ucStatus)
{
try {
    pthread_mutex_lock(&mutexSmem);
    ucTimerCTIMER = ucStatus;
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}


//---------------------------------------------------------------------------
unsigned char SMEM::vGetTimerMutexRESET(void)
{
try {
    unsigned char ucRet;
    pthread_mutex_lock(&mutexSmem);
    ucRet = ucTimerRESET;
    pthread_mutex_unlock(&mutexSmem);
    return ucRet;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vSetTimerMutexRESET(unsigned char ucStatus)
{
try {
    pthread_mutex_lock(&mutexSmem);
    ucTimerRESET = ucStatus;
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
unsigned char SMEM::vGetW77E58FWVer(unsigned char ucSelect)
{
try {
    unsigned char ucRet;
    pthread_mutex_lock(&mutexSmem);
    ucRet = ucW77E58FWVer[ucSelect];
    pthread_mutex_unlock(&mutexSmem);
    return ucRet;
  } catch (...) {}
}
//---------------------------------------------------------------------------
bool SMEM::vSetW77E58FWVer(unsigned char ucSelect, unsigned char ucVar)
{
try {
    pthread_mutex_lock(&mutexSmem);
    ucW77E58FWVer[ucSelect] = ucVar;
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSetSystemClockTime(unsigned short int usiYear, unsigned char ucMonth,
                               unsigned char ucDay,  unsigned char ucHour,
                               unsigned char ucMin,  unsigned char ucSec         )
{
try {
    char date[64] = {0}, ctime[24] = {0};

    if(ucMBType == MB_ICOP6115) {
      pthread_mutex_lock(&mutexSmem);
      sprintf(date,"date -s %#02d%#02d%#02d%#02d%#04d.%#02d",ucMonth,ucDay,ucHour,ucMin,usiYear, ucSec);
      system(date);
      pthread_mutex_unlock(&mutexSmem);

    } else {
      pthread_mutex_lock(&mutexSmem);
      sprintf(date,"date -s %#02d%#02d%#02d%#02d%#04d",ucMonth,ucDay,ucHour,ucMin,usiYear);
      system(date);
      printf("\n%s\n",date);
      sprintf(ctime,"date -s %#02d:%#02d:%#02d",ucHour,ucMin,ucSec);
      system(ctime);
//moveOut      system("hwclock -w");

      printf("\n%s\n",ctime);
      pthread_mutex_unlock(&mutexSmem);
    }
    return true;
  } catch (...) {}
}


//---------------------------------------------------------------------------
bool SMEM::vSetVDPhaseMapTable(unsigned char ucPhase, unsigned char ucVD, unsigned short int usiLCN)
{
try {
  int iNo;
  pthread_mutex_lock(&mutexSmem);
  usiVDPhaseMap[ucPhase][ucVD] = usiLCN;
  pthread_mutex_unlock(&mutexSmem);
  iNo = ucPhase*10 + usiLCN;

/*
  switch(iNo) {
    case(0): oStorINI.vSaveIntToINI("VD", "Phase0_0", usiLCN); break;
    case(1): oStorINI.vSaveIntToINI("VD", "Phase0_1", usiLCN); break;
    case(2): oStorINI.vSaveIntToINI("VD", "Phase0_2", usiLCN); break;
    case(3): oStorINI.vSaveIntToINI("VD", "Phase0_3", usiLCN); break;
    case(4): oStorINI.vSaveIntToINI("VD", "Phase0_4", usiLCN); break;
    case(5): oStorINI.vSaveIntToINI("VD", "Phase0_5", usiLCN); break;
    case(6): oStorINI.vSaveIntToINI("VD", "Phase0_6", usiLCN); break;
    case(7): oStorINI.vSaveIntToINI("VD", "Phase0_7", usiLCN); break;
    case(10): oStorINI.vSaveIntToINI("VD", "Phase1_0", usiLCN); break;
    case(11): oStorINI.vSaveIntToINI("VD", "Phase1_1", usiLCN); break;
    case(12): oStorINI.vSaveIntToINI("VD", "Phase1_2", usiLCN); break;
    case(13): oStorINI.vSaveIntToINI("VD", "Phase1_3", usiLCN); break;
    case(14): oStorINI.vSaveIntToINI("VD", "Phase1_4", usiLCN); break;
    case(15): oStorINI.vSaveIntToINI("VD", "Phase1_5", usiLCN); break;
    case(16): oStorINI.vSaveIntToINI("VD", "Phase1_6", usiLCN); break;
    case(17): oStorINI.vSaveIntToINI("VD", "Phase1_7", usiLCN); break;
    case(20): oStorINI.vSaveIntToINI("VD", "Phase2_0", usiLCN); break;
    case(21): oStorINI.vSaveIntToINI("VD", "Phase2_1", usiLCN); break;
    case(22): oStorINI.vSaveIntToINI("VD", "Phase2_2", usiLCN); break;
    case(23): oStorINI.vSaveIntToINI("VD", "Phase2_3", usiLCN); break;
    case(24): oStorINI.vSaveIntToINI("VD", "Phase2_4", usiLCN); break;
    case(25): oStorINI.vSaveIntToINI("VD", "Phase2_5", usiLCN); break;
    case(26): oStorINI.vSaveIntToINI("VD", "Phase2_6", usiLCN); break;
    case(27): oStorINI.vSaveIntToINI("VD", "Phase2_7", usiLCN); break;
    case(30): oStorINI.vSaveIntToINI("VD", "Phase3_0", usiLCN); break;
    case(31): oStorINI.vSaveIntToINI("VD", "Phase3_1", usiLCN); break;
    case(32): oStorINI.vSaveIntToINI("VD", "Phase3_2", usiLCN); break;
    case(33): oStorINI.vSaveIntToINI("VD", "Phase3_3", usiLCN); break;
    case(34): oStorINI.vSaveIntToINI("VD", "Phase3_4", usiLCN); break;
    case(35): oStorINI.vSaveIntToINI("VD", "Phase3_5", usiLCN); break;
    case(36): oStorINI.vSaveIntToINI("VD", "Phase3_6", usiLCN); break;
    case(37): oStorINI.vSaveIntToINI("VD", "Phase3_7", usiLCN); break;
    case(40): oStorINI.vSaveIntToINI("VD", "Phase4_0", usiLCN); break;
    case(41): oStorINI.vSaveIntToINI("VD", "Phase4_1", usiLCN); break;
    case(42): oStorINI.vSaveIntToINI("VD", "Phase4_2", usiLCN); break;
    case(43): oStorINI.vSaveIntToINI("VD", "Phase4_3", usiLCN); break;
    case(44): oStorINI.vSaveIntToINI("VD", "Phase4_4", usiLCN); break;
    case(45): oStorINI.vSaveIntToINI("VD", "Phase4_5", usiLCN); break;
    case(46): oStorINI.vSaveIntToINI("VD", "Phase4_6", usiLCN); break;
    case(47): oStorINI.vSaveIntToINI("VD", "Phase4_7", usiLCN); break;
    case(50): oStorINI.vSaveIntToINI("VD", "Phase5_0", usiLCN); break;
    case(51): oStorINI.vSaveIntToINI("VD", "Phase5_1", usiLCN); break;
    case(52): oStorINI.vSaveIntToINI("VD", "Phase5_2", usiLCN); break;
    case(53): oStorINI.vSaveIntToINI("VD", "Phase5_3", usiLCN); break;
    case(54): oStorINI.vSaveIntToINI("VD", "Phase5_4", usiLCN); break;
    case(55): oStorINI.vSaveIntToINI("VD", "Phase5_5", usiLCN); break;
    case(56): oStorINI.vSaveIntToINI("VD", "Phase5_6", usiLCN); break;
    case(57): oStorINI.vSaveIntToINI("VD", "Phase5_7", usiLCN); break;
    case(60): oStorINI.vSaveIntToINI("VD", "Phase6_0", usiLCN); break;
    case(61): oStorINI.vSaveIntToINI("VD", "Phase6_1", usiLCN); break;
    case(62): oStorINI.vSaveIntToINI("VD", "Phase6_2", usiLCN); break;
    case(63): oStorINI.vSaveIntToINI("VD", "Phase6_3", usiLCN); break;
    case(64): oStorINI.vSaveIntToINI("VD", "Phase6_4", usiLCN); break;
    case(65): oStorINI.vSaveIntToINI("VD", "Phase6_5", usiLCN); break;
    case(66): oStorINI.vSaveIntToINI("VD", "Phase6_6", usiLCN); break;
    case(67): oStorINI.vSaveIntToINI("VD", "Phase6_7", usiLCN); break;
    case(70): oStorINI.vSaveIntToINI("VD", "Phase7_0", usiLCN); break;
    case(71): oStorINI.vSaveIntToINI("VD", "Phase7_1", usiLCN); break;
    case(72): oStorINI.vSaveIntToINI("VD", "Phase7_2", usiLCN); break;
    case(73): oStorINI.vSaveIntToINI("VD", "Phase7_3", usiLCN); break;
    case(74): oStorINI.vSaveIntToINI("VD", "Phase7_4", usiLCN); break;
    case(75): oStorINI.vSaveIntToINI("VD", "Phase7_5", usiLCN); break;
    case(76): oStorINI.vSaveIntToINI("VD", "Phase7_6", usiLCN); break;
    case(77): oStorINI.vSaveIntToINI("VD", "Phase7_7", usiLCN); break;

    default:
      break;

  }
*/
} catch (...) {}
}

//---------------------------------------------------------------------------
unsigned short int SMEM::vGetVDPhaseMapTable(unsigned char ucPhase, unsigned char ucVD)
{
try {

  unsigned short int usiLCN = 0;
//  pthread_mutex_lock(&mutexSmem);
  usiLCN = usiVDPhaseMap[ucPhase][ucVD];
//  pthread_mutex_unlock(&mutexSmem);
  return usiLCN;

} catch (...) {}
}


//---------------------------------------------------------------------------
bool SMEM::vSetVDPhaseTriggerSwitch(unsigned char ucPhase, unsigned char ucSwitch)
{
try {

    pthread_mutex_lock(&mutexSmem);
    ucVDPhaseTriggerSwitch[ucPhase] = ucSwitch;
    pthread_mutex_unlock(&mutexSmem);

/*
  switch(ucPhase) {
    case(0): oStorINI.vSaveIntToINI("VDTrigger", "PhaseTriggerSwitch0", ucSwitch); break;
    case(1): oStorINI.vSaveIntToINI("VDTrigger", "PhaseTriggerSwitch1", ucSwitch); break;
    case(2): oStorINI.vSaveIntToINI("VDTrigger", "PhaseTriggerSwitch2", ucSwitch); break;
    case(3): oStorINI.vSaveIntToINI("VDTrigger", "PhaseTriggerSwitch3", ucSwitch); break;
    case(4): oStorINI.vSaveIntToINI("VDTrigger", "PhaseTriggerSwitch4", ucSwitch); break;
    case(5): oStorINI.vSaveIntToINI("VDTrigger", "PhaseTriggerSwitch5", ucSwitch); break;
    case(6): oStorINI.vSaveIntToINI("VDTrigger", "PhaseTriggerSwitch6", ucSwitch); break;
    case(7): oStorINI.vSaveIntToINI("VDTrigger", "PhaseTriggerSwitch7", ucSwitch); break;

    default:
      break;

  }
*/
} catch (...) {}
}

//---------------------------------------------------------------------------
unsigned char SMEM::vGetVDPhaseTriggerSwitch(unsigned char ucPhase)
{
try {

  unsigned char ucTmp;

//  pthread_mutex_lock(&mutexSmem);
  ucTmp = ucVDPhaseTriggerSwitch[ucPhase];
//  pthread_mutex_unlock(&mutexSmem);
  return ucTmp;

} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSetForceClockRedCountOneTime(bool bVar)
{
try {
    pthread_mutex_lock(&mutexSmem);
    bForceClockRedCountOneTime = bVar;
    pthread_mutex_unlock(&mutexSmem);
    return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vGetForceClockRedCountOneTime(void)
{
try {
  bool bTmp;
  pthread_mutex_lock(&mutexSmem);
  bTmp = bForceClockRedCountOneTime;
  pthread_mutex_unlock(&mutexSmem);
  return bTmp;
} catch (...) {}
}


//---------------------------------------------------------------------------
bool SMEM::vSetStopSend0x22(bool bVar)
{
try {
    pthread_mutex_lock(&mutexSmem);
    bBugStopSend0x22ForceCtl51Light = bVar;
    pthread_mutex_unlock(&mutexSmem);
    return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vGetStopSend0x22(void)
{
try {
  bool bTmp;
  pthread_mutex_lock(&mutexSmem);
  bTmp = bBugStopSend0x22ForceCtl51Light;
  pthread_mutex_unlock(&mutexSmem);
  return bTmp;
} catch (...) {}
}


//---------------------------------------------------------------------------
bool SMEM::vSetTC5F08Status(unsigned char ucVar)
{
try {
    pthread_mutex_lock(&mutexSmem);
    if(ucVar != ucTC5F08Status) {
      bTC5F08StatusChange = true;
    }
    ucTC5F08Status = ucVar;
    pthread_mutex_unlock(&mutexSmem);

    printf("[Debug] smem, set 5F08 to %X, %d\n", ucTC5F08Status, bTC5F08StatusChange);

    return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
unsigned char SMEM::vGetTC5F08Status(void)
{
try {
  unsigned char ucTmp;
  pthread_mutex_lock(&mutexSmem);
  ucTmp = ucTC5F08Status;
  pthread_mutex_unlock(&mutexSmem);
  printf("[MESSAGE DEBUG] smem.vGetTC5F08Status.. 0x%X\n", ucTC5F08Status);
  return ucTmp;
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSetTC5F08StatusChange(bool bVar)
{
try {
    pthread_mutex_lock(&mutexSmem);
    bTC5F08StatusChange = bVar;
    pthread_mutex_unlock(&mutexSmem);
    return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vGetTC5F08StatusChange(void)
{
try {
  bool bTmp;
  pthread_mutex_lock(&mutexSmem);
  bTmp = bTC5F08StatusChange;
  pthread_mutex_unlock(&mutexSmem);
  return bTmp;
} catch (...) {}
}


//---------------------------------------------------------------------------
bool SMEM::vSetThisCycleRunCCJPlan5F18(bool bVar)
{
try {
    pthread_mutex_lock(&mutexSmem);
    bThisCycleRunCCJPlan5F18 = bVar;
    pthread_mutex_unlock(&mutexSmem);
    printf("[Debug] vSetThisCycleRunCCJPlan5F18:%d\n", bVar);
    return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vGetThisCycleRunCCJPlan5F18(void)
{
try {
  bool bTmp;
  pthread_mutex_lock(&mutexSmem);
  bTmp = bThisCycleRunCCJPlan5F18;
  pthread_mutex_unlock(&mutexSmem);
  return bTmp;
} catch (...) {}
}


//---------------------------------------------------------------------------
bool SMEM::vSetThisCycleRunCCJActure5FA2(bool bVar)
{
try {
    pthread_mutex_lock(&mutexSmem);
    bThisCycleRunCCJActure5FA2 = bVar;
    pthread_mutex_unlock(&mutexSmem);
    return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vGetThisCycleRunCCJActure5FA2(void)  //ver. arwen act
{
try {
  bool bTmp;
  pthread_mutex_lock(&mutexSmem);
  bTmp = bThisCycleRunCCJActure5FA2;
  pthread_mutex_unlock(&mutexSmem);
  return bTmp;
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSaveCurrentAsGreenStartTime(void)
{
try {
  clock_gettime(CLOCK_REALTIME, &actGreenStartTimeForScreenActuateArwenStatus);
  return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
unsigned int SMEM::vGetActRunningGreenTime(void)
{
try {
  int iRet;
  clock_gettime(CLOCK_REALTIME, &actGreenCurrentTimeForScreenActuateArwenStatus);
  iRet = (actGreenCurrentTimeForScreenActuateArwenStatus.tv_sec - actGreenStartTimeForScreenActuateArwenStatus.tv_sec);
  if(iRet >=0 && iRet <= 999) {
    return iRet;
  } else {
    return 0;
  }
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSetLastRevSyncTime(time_t aIn)
{
try {
    pthread_mutex_lock(&mutexSmem);
    _RecRevSyncTime = aIn;
    pthread_mutex_unlock(&mutexSmem);
    return true;

} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vCheckRevSyncStatus(void)
{
try {
  bool bRevSyncComm_OK = false;
  char cTMP[128];
  static unsigned int uiRevSyncOKPlus = 0;
  static unsigned int uiRevSyncFailPlus = 0;
  time_t _now = time(NULL);
  struct tm *now = localtime(&_now);

  unsigned int uiTimeA, uiTimeB;
  int iDiffTime;
  uiTimeA = (now->tm_hour) * 3600 + (now->tm_min) * 60 + (now->tm_sec);
  printf("Rev: uiTimeNow:%d,%d:%d:%d\n", uiTimeA, now->tm_hour,now->tm_min,now->tm_sec);
  struct tm *revSync = localtime(&_RecRevSyncTime);
  uiTimeB = (revSync->tm_hour) * 3600 + (revSync->tm_min) * 60 + (revSync->tm_sec);
  printf("Rev: uiTimeRev:%d,%d:%d:%d\n", uiTimeB,revSync->tm_hour,revSync->tm_min,revSync->tm_sec);

  iDiffTime = uiTimeA - uiTimeB;
  iDiffTime = abs(iDiffTime);  //last communcation time.

  printf("ucRevSyncTimeDiffTolarence:%d\n", ucRevSyncTimeDiffTolarence);

  if(iDiffTime <= ucRevSyncTimeDiffTolarence) {
    if(now->tm_mon == revSync->tm_mon && now->tm_mday == revSync->tm_mday) {
      bRevSyncComm_OK = true;
    }
  }

  if(bRevSyncComm_OK == true &&
     _NewestRevSyncStatus.bYMD_OK == true &&
     _NewestRevSyncStatus.bTime_OK == true &&
     _NewestRevSyncStatus.bRevStep_OK == true &&
     _NewestRevSyncStatus.bRevSegSet_OK == true &&
     _NewestRevSyncStatus.bRevTime_OK == true        ) {
       uiRevSyncOKPlus++;
       uiRevSyncFailPlus = 0;
  } else {
       uiRevSyncOKPlus = 0;
       uiRevSyncFailPlus++;

       printf("[rev] error msg:\nbRevSyncComm_OK:%d _NewestRevSyncStatus.bYMD_OK:%d _NewestRevSyncStatus.bTime_OK:%d _NewestRevSyncStatus.bRevStep_OK:%d _NewestRevSyncStatus.bRevSegSet_OK:%d _NewestRevSyncStatus.bRevTime_OK:%d\n",
       bRevSyncComm_OK,
       _NewestRevSyncStatus.bYMD_OK,
       _NewestRevSyncStatus.bTime_OK,
       _NewestRevSyncStatus.bRevStep_OK,
       _NewestRevSyncStatus.bRevSegSet_OK,
       _NewestRevSyncStatus.bRevTime_OK        );

  }

  if(uiRevSyncOKPlus >= ucRevSyncCountineStatusAdj) {
    if(iRevSyncStatus != 1) {
      iLastRevSyncStatus = iRevSyncStatus;
      iRevSyncStatus  = 1;

      sprintf(cTMP, "RevSync Change Status form:%d to %d, %d, %d, %d, %d, %d, %d",
      iLastRevSyncStatus, iRevSyncStatus,
      bRevSyncComm_OK, _NewestRevSyncStatus.bYMD_OK, _NewestRevSyncStatus.bTime_OK,
      _NewestRevSyncStatus.bRevStep_OK, _NewestRevSyncStatus.bRevSegSet_OK, _NewestRevSyncStatus.bRevTime_OK
      );
//OTTDEUG      smem.vWriteMsgToDOM(cTMP);

    }
  } else if(uiRevSyncFailPlus >= ucRevSyncCountineStatusAdj) {
    if(iRevSyncStatus != -1) {
      iLastRevSyncStatus = iRevSyncStatus;
      iRevSyncStatus = -1;
      sprintf(cTMP, "RevSync Change Status form:%d to %d [C:%d, D:%d, T:%d, S:%d, Seg:%d, CFG:%d]",
        iLastRevSyncStatus, iRevSyncStatus,
        bRevSyncComm_OK, _NewestRevSyncStatus.bYMD_OK, _NewestRevSyncStatus.bTime_OK,
        _NewestRevSyncStatus.bRevStep_OK, _NewestRevSyncStatus.bRevSegSet_OK, _NewestRevSyncStatus.bRevTime_OK
      );
      smem.vWriteMsgToDOM(cTMP);

      stc.vResetReverseTime();
    }
  } else {
    if(iRevSyncStatus != 0) {
      iLastRevSyncStatus = iRevSyncStatus;
      iRevSyncStatus = 0;
      sprintf(cTMP, "RevSync Change Status form:%d to %d, %d, %d, %d, %d, %d, %d",
      iLastRevSyncStatus, iRevSyncStatus,
      bRevSyncComm_OK, _NewestRevSyncStatus.bYMD_OK, _NewestRevSyncStatus.bTime_OK,
      _NewestRevSyncStatus.bRevStep_OK, _NewestRevSyncStatus.bRevSegSet_OK, _NewestRevSyncStatus.bRevTime_OK
      );

//OTTDEUG      smem.vWriteMsgToDOM(cTMP);
    }
  }

  ucRevSyncByteStatus = 0;
  if(ucRevSyncEnable > 0) { ucRevSyncByteStatus += 1; }
  if(iRevSyncStatus >= 0) { ucRevSyncByteStatus += 2; }
  if(bRevSyncComm_OK == true) { ucRevSyncByteStatus += 4; }
  if(_NewestRevSyncStatus.bYMD_OK == true) { ucRevSyncByteStatus += 8; }
  if(_NewestRevSyncStatus.bTime_OK == true) { ucRevSyncByteStatus += 16; }
  if(_NewestRevSyncStatus.bRevStep_OK == true) { ucRevSyncByteStatus += 32; }
  if(_NewestRevSyncStatus.bRevSegSet_OK == true) { ucRevSyncByteStatus += 64; }
  if(_NewestRevSyncStatus.bRevTime_OK == true) { ucRevSyncByteStatus += 128; }

/*OTDebug
  printf("***********************************************************\n");
  printf("Comm_OK:%d, YMD_OK:%d, Time_OK:%d, Step_OK:%d, SegSet_OK:%d, Time_OK:%d\n", bRevSyncComm_OK, _NewestRevSyncStatus.bYMD_OK, _NewestRevSyncStatus.bTime_OK, _NewestRevSyncStatus.bRevStep_OK, _NewestRevSyncStatus.bRevSegSet_OK, _NewestRevSyncStatus.bRevTime_OK);
  printf("RevSecCounter:%d\n", stc.vGetRevTimerSec() );
  printf("-----------------------------------------------------------\n");
*/

/*
  sprintf(cTMP, "cycleRevSync Change Status form:%d to %d, %d, %d, %d, %d, %d, %d",
  iLastRevSyncStatus, iRevSyncStatus,
  bRevSyncComm_OK, _NewestRevSyncStatus.bYMD_OK, _NewestRevSyncStatus.bTime_OK,
  _NewestRevSyncStatus.bRevStep_OK, _NewestRevSyncStatus.bRevSegSet_OK, _NewestRevSyncStatus.bRevTime_OK
  );
  smem.vWriteMsgToDOM(cTMP);

  sprintf(cTMP, "bYMD_OK:%d, bTime_OK:%d, bRevStep_OK:%d, bRevSegSet_OK:%d, bRevTime_OK:%d\n", _NewestRevSyncStatus.bYMD_OK, _NewestRevSyncStatus.bTime_OK, _NewestRevSyncStatus.bRevStep_OK, _NewestRevSyncStatus.bRevSegSet_OK, _NewestRevSyncStatus.bRevTime_OK);
  smem.vWriteMsgToDOM(cTMP);
*/



} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSetRevSyncStatus(sRevSyncStatus sIn)
{
try {
    pthread_mutex_lock(&mutexSmem);
    _NewestRevSyncStatus = sIn;
    pthread_mutex_unlock(&mutexSmem);
    return true;
} catch (...) {}
}
//---------------------------------------------------------------------------
sRevSyncStatus SMEM::vGetRevSyncStatus(void)
{
try {
  sRevSyncStatus _tmp;
  pthread_mutex_lock(&mutexSmem);
  _tmp = _NewestRevSyncStatus;
  pthread_mutex_unlock(&mutexSmem);
  return _tmp;
} catch (...) {}
}



//---------------------------------------------------------------------------
bool SMEM::vSetRevSyncVar(int iVar)
{
try {
    pthread_mutex_lock(&mutexSmem);
    iRevSyncStatus = iVar;
    pthread_mutex_unlock(&mutexSmem);
    return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
int SMEM::vGetRevSyncVar(void)  //ver. arwen act
{
try {
  int iVar;
  pthread_mutex_lock(&mutexSmem);
  iVar = iRevSyncStatus;
  pthread_mutex_unlock(&mutexSmem);
  return iVar;
} catch (...) {}
}


//---------------------------------------------------------------------------
bool SMEM::vSetCommEnable(bool bVar)
{
try {
    pthread_mutex_lock(&mutexSmem);
    bCommEnableSwitch = bVar;
    pthread_mutex_unlock(&mutexSmem);
    return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vGetCommEnable(void)
{
try {
//  int iVar;
//  pthread_mutex_lock(&mutexSmem);
//  bVar = bCommEnableSwitch;
//  pthread_mutex_unlock(&mutexSmem);
  return bCommEnableSwitch;
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSet5F18EffectTime(unsigned int uiVar)
{
try {
  ui5F18EffectTime = uiVar;
  return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSet5F18EffectTimeCountDown(void)
{
try {
  if(ui5F18EffectTime > 0) {
    ui5F18EffectTime--;

    printf("new 5F18CountDown Time:%d\n", ui5F18EffectTime);

    if(ui5F18EffectTime == 0) {  //change to zero,
      if(ucSmem92TC_ControlStrategy == 0x30) {  //in dyn mode
        ucSmem92TC_ControlStrategy = 0x01;
      }

//OT1000310
      if(ucSmem92TC_ControlStrategy == 0x02) {  //in plan mode
        ucSmem92TC_ControlStrategy = 0x01;
      }

    }

  }
  return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
unsigned int SMEM::vGet5F18EffectTime(void)
{
try {
  return ui5F18EffectTime;
} catch (...) {}
}

//OT20110526
//---------------------------------------------------------------------------
bool SMEM::vSetLastGetProtocolTime(void)
{
try{
  pthread_mutex_lock(&mutexSmem);
  lastGetProtocolTime = _SystemTime;
  pthread_mutex_unlock(&mutexSmem);
  return true;
}catch(...){}
}

//---------------------------------------------------------------------------
time_t SMEM::vGetLastGetProtocolTime(void)
{
try{
  time_t _ret;
  pthread_mutex_lock(&mutexSmem);
  _ret = lastGetProtocolTime;
  pthread_mutex_unlock(&mutexSmem);
  return _ret;
}catch(...){}
}

//---------------------------------------------------------------------------
bool SMEM::vSetSystemTime(time_t _tmp)
{
try {
    pthread_mutex_lock(&mutexSmem);
    _SystemTime = _tmp;
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
time_t SMEM::vGetSystemTime(void)
{
try {
//    pthread_mutex_lock(&mutexSmem);
    _SystemTimeTmp = _SystemTime;
//    pthread_mutex_unlock(&mutexSmem);
    return _SystemTimeTmp;
  } catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSetShowRedCountVar(unsigned char uc1, unsigned char uc2, unsigned char uc3, unsigned char uc4, unsigned char uc5, unsigned char uc6, unsigned char uc7, unsigned char uc8)
{
try {
    pthread_mutex_lock(&mutexSmem);
    ucLastShowRedCountVar[0] = uc1;
    ucLastShowRedCountVar[1] = uc2;
    ucLastShowRedCountVar[2] = uc3;
    ucLastShowRedCountVar[3] = uc4;
    ucLastShowRedCountVar[4] = uc5;
    ucLastShowRedCountVar[5] = uc6;
    ucLastShowRedCountVar[6] = uc7;
    ucLastShowRedCountVar[7] = uc8;
    ucLastShowRedCountTime = _SystemTime;
    pthread_mutex_unlock(&mutexSmem);
    return true;
  } catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vLogTrafficLightTime(void)
{
try {
  _lastGetTrafficLightTime = _SystemTime;

  return true;
} catch (...) {}
}

//---------------------------------------------------------------------------
time_t SMEM::vGetLastTrafficeLightTime(void)
{
try{
  time_t _ret;
  pthread_mutex_lock(&mutexSmem);
  _ret = _lastGetTrafficLightTime;
  pthread_mutex_unlock(&mutexSmem);
  return _ret;
}catch(...){}
}

//---------------------------------------------------------------------------
bool SMEM::vSetTrafficLightBoardConnectStatus(bool bIn)
{
try {
  bTrafficeLightBoardConnectStatus = bIn;
  return bTrafficeLightBoardConnectStatus;
} catch (...) {}
}

//OT20131225
//---------------------------------------------------------------------------
bool SMEM::vAddCCJHeartBeatCount(unsigned short int val)
{
try {
  if(val == 0) {
    usiCCJHeartBeatCount = 0;
  } else {
    usiCCJHeartBeatCount += val;
  }
  return true;
} catch (...) {}
}


//OT20140211
//---------------------------------------------------------------------------
bool SMEM::vGetPlanForceTOD(unsigned short int pid)
{
try {
  return bPlanForceTOD[pid];
} catch (...) {}
}

//---------------------------------------------------------------------------
bool SMEM::vSetPlanForceTOD(unsigned short int pid, bool status)
{
try {
  bPlanForceTOD[pid] = status;
  return true;
} catch (...) {}
}
//------------jacky20151210----------------------------------
bool SMEM::SetRequestKeypad(unsigned char Req)
{
try{
    pthread_mutex_lock(&mutexSmem);
    setReq=Req;
    pthread_mutex_unlock(&mutexSmem);
    return true;
 } catch (...) {}
}

unsigned char SMEM::GetRequestKeypad(void)
{
try{
    unsigned short int temp;
    pthread_mutex_lock(&mutexSmem);
    temp=setReq;
    pthread_mutex_unlock(&mutexSmem);
    return temp;
 } catch (...) {}
}
//----------------------------Eason_Ver3.3---------------------------------------------
bool SMEM::vSendPedAndRedCountEverySec(void)    // 每秒送V3 紅燈倒數 行人倒數
{
try {

    unsigned char PgCtl;
    unsigned char PrCtl;
    unsigned char ucTmp;
    unsigned short int stc_subphase;
    unsigned short int stc_subphase_step;
    unsigned short int PGPR[4];

    printf("\n");
    MESSAGEOK _MsgOK;
    if(ucCSTCControlStrategy == 70 || ucCSTCControlStrategy == 80 || ucCSTCControlStrategy == 90 || ucCSTCControlStrategy == 95){ //動態 手動 全紅 閃光

        _MsgOK = oDataToMessageOK.vPackageINFOToredcountV3(0x99 ,0x99 ,0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0);
        _MsgOK.InnerOrOutWard = cOutWard;
        writeJob.WritePhysicalOut(_MsgOK.packet, _MsgOK.packetLength, DEVICEREDCOUNTVERV3);

        _MsgOK = oDataToMessageOK.vPackageINFOToredcountV3(0x99 ,0x99 ,0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 1);
        _MsgOK.InnerOrOutWard = cOutWard;
        writeJob.WritePhysicalOut(_MsgOK.packet, _MsgOK.packetLength, DEVICEREDCOUNTVERV3);
    }
    else
    {
        _MsgOK = oDataToMessageOK.vPackageINFOToredcountV3(0 ,redcount_remainder[0], 0, redcount_remainder[1], 0, redcount_remainder[2], 0, redcount_remainder[3], 0);
        _MsgOK.InnerOrOutWard = cOutWard;

                // for(int i;i<_MsgOK.packetLength;i++) printf("%d ",_MsgOK.packet[i]);
        // printf("usiHCRedcountMapping[0]:%d\n", usiHCRedcountMapping[0]);
        writeJob.WritePhysicalOut(_MsgOK.packet, _MsgOK.packetLength, DEVICEREDCOUNTVERV3);

        // printf("usiHCRedcountMapping[1]:%d\n", usiHCRedcountMapping[1]);

        _MsgOK = oDataToMessageOK.vPackageINFOToredcountV3(0 ,redcount_remainder[4], 0, redcount_remainder[5], 0, redcount_remainder[6], 0, redcount_remainder[7], 1);
        _MsgOK.InnerOrOutWard = cOutWard;
        // printf("usiHCRedcountMapping[0]:%d\n", usiHCRedcountMapping[0]);
        writeJob.WritePhysicalOut(_MsgOK.packet, _MsgOK.packetLength, DEVICEREDCOUNTVERV3);
    }



//-----------------------------Green man-----------------------------------------------------------------
    if(ucCSTCControlStrategy == 70 || ucCSTCControlStrategy == 80 || ucCSTCControlStrategy == 90 || ucCSTCControlStrategy == 95) {  //ALLDYNAMIC MANUAL  FLASH  ALLRED
        for(int i = 0; i < 8; i++)  Pr_remainder[i] = 0xFF;
        for(int i = 0; i < 8; i++)  Pg_remainder[i] = 0xFF;        
    }
    
    PgCtl = 0;
    PrCtl = 0;
    ucTmp = 0x80;
    for(int i = 0; i < 4; i++) {
        if(Pg_remainder[i] > 0) {
            PgCtl += ucTmp;
            PGPR[i] = Pg_remainder[i];
            // printf("\n\nPgCtl=%d \n\n",PgCtl);
        } else {
            PrCtl += ucTmp;
            PGPR[i] = Pr_remainder[i];
        }
        ucTmp = ucTmp >> 1;
    }
        _MsgOK = oDataToMessageOK.vPackageINFOToGreenmanCount(PgCtl, PrCtl, PGPR[0], PGPR[1], PGPR[2], PGPR[3], 0);
        _MsgOK.InnerOrOutWard = cOutWard;


        unsigned short int phase_order_flash  = stc.vGetUSIData(CSTC_exec_plan_phase_order);
        if (phase_order_flash == 0x80 || phase_order_flash == 0xB0)
        {   if(FlashClear == true){
                _MsgOK = oDataToMessageOK.vPackageINFOToGreenmanCount(0xF0, 0x00, 0x00, 0x00, 0x00, 0x00 , 0);
                _MsgOK.InnerOrOutWard = cOutWard;
                
                writeJob.WritePhysicalOut(_MsgOK.packet, _MsgOK.packetLength, DEVICEREDCOUNTVERV3);   //jacky20140331

                _MsgOK = oDataToMessageOK.vPackageINFOToGreenmanCount(0xF0, 0x00, 0x00, 0x00, 0x00, 0x00 , 1);
                _MsgOK.InnerOrOutWard = cOutWard;
                
                writeJob.WritePhysicalOut(_MsgOK.packet, _MsgOK.packetLength, DEVICEREDCOUNTVERV3);   //jacky20140331

                FlashClear = false;
            }
            else{
                
                writeJob.WritePhysicalOut(_MsgOK.packet, _MsgOK.packetLength, DEVICEREDCOUNTVERV3);   //jacky20140331
            }
        }
        else {
//            if(GetGreenManonoff5014() == 0){
                
                writeJob.WritePhysicalOut(_MsgOK.packet, _MsgOK.packetLength, DEVICEREDCOUNTVERV3);   //jacky20140331
//            }
            FlashClear = true;
        }

    //OT20140329
    PgCtl = 0;
    PrCtl = 0;
    ucTmp = 0x80;
    for(int i = 4; i < 8; i++) {
        if(Pg_remainder[i] > 0) {
            PgCtl += ucTmp;
            PGPR[i-4] = Pg_remainder[i];
        } else {
            PrCtl += ucTmp;
            PGPR[i-4] = Pr_remainder[i];
        }
        ucTmp = ucTmp >> 1;
    }
   // printf("\n");
  //  if(GetPriorityswitch() == true || GetGreenManonoff5014() == 1){
  //       _MsgOK = oDataToMessageOK.vPackageINFOToGreenmanCount(PgCtl, PrCtl, 255, 255, 255, 255, 1);
  //       _MsgOK.InnerOrOutWard = cOutWard;
        
  //       writeJob.WritePhysicalOut(_MsgOK.packet, _MsgOK.packetLength, DEVICEREDCOUNTVERTAICHUNG);
  //   }
  //   else {
        _MsgOK = oDataToMessageOK.vPackageINFOToGreenmanCount(PgCtl, PrCtl, PGPR[0], PGPR[1], PGPR[2], PGPR[3], 1);
        _MsgOK.InnerOrOutWard = cOutWard;

        //jacky20131225 FLASH
     /*   unsigned short int*/ phase_order_flash  = stc.vGetUSIData(CSTC_exec_plan_phase_order);
        if (phase_order_flash == 0x80 || phase_order_flash == 0xB0)
        {   if(FlashClear == true){
                _MsgOK = oDataToMessageOK.vPackageINFOToGreenmanCount(0xF0, 0x00, 0x00, 0x00, 0x00, 0x00 , 0);
                _MsgOK.InnerOrOutWard = cOutWard;
                
                writeJob.WritePhysicalOut(_MsgOK.packet, _MsgOK.packetLength, DEVICEREDCOUNTVERV3);   //jacky20140331

                _MsgOK = oDataToMessageOK.vPackageINFOToGreenmanCount(0xF0, 0x00, 0x00, 0x00, 0x00, 0x00 , 1);
                _MsgOK.InnerOrOutWard = cOutWard;
                
                writeJob.WritePhysicalOut(_MsgOK.packet, _MsgOK.packetLength, DEVICEREDCOUNTVERV3);   //jacky20140331

                FlashClear = false;
            }
            else{
                
                writeJob.WritePhysicalOut(_MsgOK.packet, _MsgOK.packetLength, DEVICEREDCOUNTVERV3);   //jacky20140331
            }
        }
        else {
// //            if(GetGreenManonoff5014() == 0){
                
                writeJob.WritePhysicalOut(_MsgOK.packet, _MsgOK.packetLength, DEVICEREDCOUNTVERV3);   //jacky20140331
// //            }
            FlashClear = true;
        }
    // }
        printf("*********************************************************************\n");
        printf("send redcount remainder:");
    for(int i = 0; i < 8; i++) {
        if(redcount_remainder[i] > 0) {
            redcount_remainder[i]--;
            printf("%d ",redcount_remainder[i]);
        }
    }
    printf("\n");
    //OT20131221
    printf("send People remainder:");
    for(int i = 0; i < 8; i++) {
        if(Pg_remainder[i] > 0) {
            Pg_remainder[i]--;
            printf("G %d ", Pg_remainder[i]);
        }else if(Pr_remainder[i] > 0){
            Pr_remainder[i]--;
            printf("R %d ", Pr_remainder[i]);
        }
    }
    printf("\n");
    return true;
  } catch (...) {}
}
//----------------------Eason_Ver3.3------------------------------
bool SMEM::vSetRedCountRemainder(unsigned short int usiRedTime[])
{
try {
    for(int i = 0; i < 8; i++) {
        if(usiRedTime[i] > 1) {
            redcount_remainder[i] = usiRedTime[i] - 1;
        } else {
            redcount_remainder[i] = 0;
        }
    }
    return true;
  } catch (...) {}
}
//---------------------Eason_Ver3.3-----------------------------
bool SMEM::vSetPgRemainder(unsigned short int usiPgTime[])
{
try {
    for(int i = 0; i < 8; i++) {
        if(usiPgTime[i] > 1) {
            Pg_remainder[i] = usiPgTime[i] - 1;
        } else {
            Pg_remainder[i] = 0;
        }
    }
    return true;
  } catch (...) {}
}
//---------------------Eason_Ver3.3-----------------------------
bool SMEM::vSetPrRemainder(unsigned short int usiPrTime[])
{
try {
    for(int i = 0; i < 8; i++) {
        if(usiPrTime[i] > 1) {
            Pr_remainder[i] = usiPrTime[i] - 1;
        } else {
            Pr_remainder[i] = 0;
        }
    }
    return true;
  } catch (...) {}
}
//---------------------Eason_Ver3.3-----------------------------
bool SMEM::GreenmanManual(bool (*bCountIF)[5][8])
{
try {
    for(int i = 0; i < 8; i++) {
        for(int j = 0; j < 5; j++) {
            for(int k = 0; k < 8; k++) {
                CheckbCountIF[i][j][k] = bCountIF[i][j][k];
                //printf("in_GreenmanManual");
            }
        }
    }

    return true;
  } catch (...) {}
}