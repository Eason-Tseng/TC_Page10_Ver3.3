#ifndef screenPassModeSetH
#define screenPassModeSetH

//---------------------------------------------------------------------------
#include "SCREEN.h"
//---------------------------------------------------------------------------
class ScreenPassModeSet:public SCREEN
{
    public:
      ScreenPassModeSet(void);
      ~ScreenPassModeSet(void);

      void DisplayPassModeSet(void);
      void doKeyWork(BYTE);                                                     //�ھ�KEY���ȧ@��

    private:

      unsigned int cPosition;                                                   //�{�b����m
      int cSelect;                                                              //�W�U����   0:��� 1:�ɶ�

      DISP_WORD cPassModeSwitch;
      unsigned char ucTMP_PassModeSwitch;
      DISP_WORD cPassServerLCN[4];
      unsigned char ucTMP_PassServerLCN[4];
      unsigned short int usiPassServerLCN;

      void initDispWord(void);                                                  //��l�Ʀ��e�����y�е��Ѽ�
//      void DisplaySetPassMode(int,int);

      BYTE passModeSetBitmap[3840];
      void loadBitmapFromFile(void);                                            //�N����Load�i�O����

      void doKey0Work(void);
      void doKey1Work(void);
      void doKey2Work(void);
      void doKey3Work(void);
      void doKey4Work(void);
      void doKey5Work(void);
      void doKey6Work(void);
      void doKey7Work(void);
      void doKey8Work(void);
      void doKey9Work(void);
      void doKeyAWork(void);
      void doKeyBWork(void);
      void doKeyCWork(void);
      void doKeyDWork(void);
      void doKeyEWork(void);
      void doKeyFWork(void);
      void doKeyF1Work(void);
      void doKeyF2Work(void);
      void doKeyF3Work(void);
      void doKeyF4Work(void);
      void doKeyUPWork(void);
      void doKeyDOWNWork(void);
      void doKeyLEFTWork(void);
      void doKeyRIGHTWork(void);
      void doKeyEnterWork(void);

      void vDisplayPassModeSwitch(void);
      void vDisplayPassModeServer(void);

      void vDataToTmpArray();
      void vTmpArrayToData();

      bool vCheckRationality();
      bool vInitUCTMPData(void);

};
//---------------------------------------------------------------------------
extern ScreenPassModeSet screenPassModeSet;
#endif