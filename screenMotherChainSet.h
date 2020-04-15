#ifndef ScreenMotherChainSetH
#define ScreenMotherChainSetH

#include "SCREEN.h"
//---------------------------------------------------------------------------
class ScreenMotherChainSet:public SCREEN
{
    public:
      ScreenMotherChainSet(void);
      ~ScreenMotherChainSet(void);

      void DisplayMotherChainSet(void);
      void DoKeyWork(BYTE);                                                     //�ھ�KEY���ȧ@��

    private:
      DISP_WORD cMotherChainSetWord[6];
      /*
      0:  ucChainStartPhaseWord;
      1:  ucChainStartStepWord;
      2:  ucChainEndPhaseWord;
      3:  ucChainEndStepWord;
      4:  ucChainManuleEnable;
      5: ChainTODEnable;
      */

      BYTE ucMotherChainSet[6];

/*
      BYTE ucChainStartPhaseWord;
      BYTE ucChainStartStepWord;
      BYTE ucChainEndPhaseWord;
      BYTE ucChainEndStepWord;

      BYTE ucChainManuleEnable;
      BYTE ucChainTODEnable;
*/

      int cPosition;                                                            //���k���ʪ���m
//      int cSelect;                                                              //�W�U����   0:��� 1:�ɶ�

      void InitDispWord(void);                                                  //��l�Ʀ��e�����y�е��Ѽ�
      void DisplayData(void);

      BYTE chainMotherSetBitmap[3840];
      void LoadBitmapFromFile(void);                                            //�N����Load�i�O����

      void DoKey0Work(void);
      void DoKey1Work(void);
      void DoKey2Work(void);
      void DoKey3Work(void);
      void DoKey4Work(void);
      void DoKey5Work(void);
      void DoKey6Work(void);
      void DoKey7Work(void);
      void DoKey8Work(void);
      void DoKey9Work(void);
      void DoKeyF1Work(void);
      void DoKeyF4Work(void);
      void doKeyUPWork(void);
      void doKeyDOWNWork(void);
      void DoKeyLEFTWork(void);
      void DoKeyRIGHTWork(void);
      void DoKeyEnterWork(void);
};
//---------------------------------------------------------------------------
extern ScreenMotherChainSet screenMotherChainSet;
#endif