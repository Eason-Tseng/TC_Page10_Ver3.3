/*
    LCX_403-5 �O���X�ʥd:
    1.�i�����]�w�ɶ����O,���w�]�w�ѼƮɶ�
    2.�ɶ��ѼƷ|�۰ʦV�W�[,��255�Y����ñN�O���I���w�](By DipSwitch)
    3.�i�����]�w��X��Ѽ�(@20,@21,@22,@23,@24,@25)
    4.�@��Address��8�ӿO�I,���O�N��  ��,��,����,���,����,�k��,��H��,��H��
    5.�O�I�]�w�C�����������]�@��(���׬O�_�ݭn�Ψ�),�]�N�O�C�����n�]�w@20~@25
    6.�O���X�ʥd���ɶ��ѼƷ|���_�����W��,�� a.�]�w�ɶ��Ѽ� b.����@20~@25����X�𱱨� �|���ܨ��
*/

#ifndef lightH
#define lightH

#define BYTE unsigned char
//----------------------------------------------------------------------
class Light
{
    public:
      Light (void);
      ~Light (void);

      //Base
      bool GetAuthority(unsigned long);      //�o�챱���v
      bool ReleaseAuthority(void);           //���񱱨��v
      bool SetTimeOfCard(BYTE);              //�]�w�O���X�ʥd�ɶ��Ѽ�
      bool SetLight(BYTE,BYTE,BYTE);         //�]�w�O����X��

      //Advance
      bool SetLight(BYTE *);                 //�I�O
      bool SetAllLight(void);                //�I�O���G
      bool SetAllDark(void);                 //�I�O���t
      bool SetCardDefault(void);             //�N�O���X�ʥd�]default:�ϥγ]�w�ɶ�(254��)�覡
      bool SayHelloToCard(void);             //�N�O���X�ʥd�ɶ��Ѽ��k0,���|��default

    private:
      unsigned long LPTBASEPORT;             //IO��m
      bool haveGetPower;                     //�O�_��o����LPT���v��

};
//----------------------------------------------------------------------
extern Light light;
#endif