//---------------------------------------------------------------------------

#ifndef PTRAFFIC92COMMH
#define PTRAFFIC92COMMH

#include "var.h"
#include "SMEM.h"

//---------------------------------------------------------------------------
class PTRAFFIC92COMM
{
    public:

        PTRAFFIC92COMM(void);
        ~PTRAFFIC92COMM(void);

        bool DoWorkViaPTraffic92(MESSAGEOK);                                    //根據PTraffic92來的訊息作事情

        bool vRebootIPC_0F10(MESSAGEOK);
        bool vReportRebootMsg_0F90(void);
        bool vSetIPCTime_0F12(MESSAGEOK);                                       //設定時間
        bool vReportSecDif_0F92(void);
        bool vQueryIPCTime_0F42(MESSAGEOK);
        bool vReportIPCTime_0FC2(void);
        bool vReportIPCTime_0F02(void);
        bool vSetCommandSet_0F13(MESSAGEOK);
        unsigned char ucCommandSet;                                             //should save to mem
        bool vQueryVersion_0F43(void);
        bool vReportVersion_0FC3(void);
        bool vSetDBPassword_0F15(MESSAGEOK);
        bool vQueryPassword_0F45(void);
        bool vReportPassword_0FC5(void);

        bool vSetDBOperatingMode_0F16(MESSAGEOK);
        bool vQueryDBOperatingMode_0F46(void);
        bool vReportDBOperatingMode_0FC6(void);

        bool vQueryEquipmentNo_0F40(MESSAGEOK);
        bool vReportEquipmentNo_0FC0(unsigned char);
        bool vReportPowerOff_0F00(void);

        int vQueryProtocolStatus_0F47(MESSAGEOK);
        bool vReportProtocolStatus_0FC7(unsigned char, int);
        bool bProtocol0FStatus[256];
        bool bProtocol5FStatus[256];
//        bool bProtocol6FStatus[256];

        bool vSetCommunicationReset_0F11(MESSAGEOK);                            //通訊重置設定
        bool vReportCommunicationReset_0F91();                                  //回報通訊重置
        bool vQueryCommunicationReset_0F41(MESSAGEOK);                          //查然後回報HW狀態
        bool vReportCommunicationStatus_0FC1();                                 //回報

        bool vSetHardwareCycle_0F14(MESSAGEOK);                                 //設定HWStat回傳週期
        struct itimerspec _itimer_HardwareCycle;                                //HWStat回傳週 期用的Interval

        bool vSetHardWareReportCycleTransTo_itimer(unsigned short int);         //把92protocol解讀成_itimer
        bool vQueryHardwareCycle_0F44(MESSAGEOK);                               //查回傳週期
        bool vReportHardwareCycle_0FC4();                                       //回報回傳週期

        bool vReportHardwareCycle_0F04(MESSAGEOK);

        //OT Pass
        bool vPass_0F8E(MESSAGEOK);

    private:

        int SecDif;

        int vReturnToCenterACK(unsigned char , unsigned char);
        int vReturnToCenterNACK(unsigned char, unsigned char, unsigned char, unsigned char);

};
//---------------------------------------------------------------------------
#endif
