/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�TC264DA���İ�
����    д��chiusir
��E-mail��chiusir@163.com
������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2020��10��28��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��dev.env.��AURIX Development Studio1.2.2�����ϰ汾
��Target �� TC264DA/TC264D
��Crystal�� 20.000Mhz
��SYS PLL�� 200MHz
________________________________________________________________
����iLLD_1_0_1_11_0�ײ����,

ʹ�����̵�ʱ�򣬽������û�пո��Ӣ��·����
����CIFΪTC264DA�����⣬�����Ĵ������TC264D
����Ĭ�ϳ�ʼ����EMEM��512K������û�ʹ��TC264D��ע�͵�EMEM_InitConfig()��ʼ��������
������\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c��164�����ҡ�
=================================================================
����������Ƶ��ַ��https://space.bilibili.com/95313236
=================================================================
����ͷ�ӿ�                  �������ۻ���OV7725ģ��
�� ���ݶ˿ڣ�P02.0-P02.7�ڣ���8λ��������ͷ�����ݶ˿ڣ�
�� ʱ�����أ��ⲿ�жϵ�0�飺P00_4��
�� ���źţ�    �ⲿ�жϵ�3�飺P15_1��
-----------------------------------------------------------------
�Ƽ�GPT12ģ�飬������ʵ��5·�����������������������ݴ�������������źŲɼ�������ѡ����·���ɣ�
P33_7, P33_6   ����TCĸ�������1
P02_8, P33_5   ����TCĸ�������2
P10_3, P10_1   ����TCĸ�������3
P20_3, P20_0   ����TCĸ�������4
-----------------------------------------------------------------
��е�ѹ�ɼ�ģ�������˷�ģ��
�Ƽ�ʹ��AN0-7������·ADC����������chirp�����źż���ų���е�ѹ�ɼ���
AN0-3          ����TC���ĸ���˷�ģ����ߵ��
-----------------------------------------------------------------
Ĭ�ϵ���ӿ�
ʹ��GTMģ�飬ATOM�ĸ�ͨ���ɲ���4*8��32·PWM�����Ҹ���Ƶ�ʺ�ռ�ձȿɵ����Ƽ�ʹ��ATOM0��0-7ͨ����
��һ��˫·ȫ������
P23_1         ����TCĸ��MOTOR1_P
P32_4         ����TCĸ��MOTOR1_N
P21_2         ����TCĸ��MOTOR2_P
P22_3         ����TCĸ��MOTOR2_N
�ڶ���˫·ȫ������
P21_4         ����TCĸ��MOTOR3_P
P21_3         ����TCĸ��MOTOR3_N
P20_8         ����TCĸ��MOTOR4_P
P21_5         ����TCĸ��MOTOR4_N
-----------------------------------------------------------------
Ĭ�϶���ӿ�
ʹ��GTMģ�飬ATOM�ĸ�ͨ���ɲ���4*8��32·PWM�����Ҹ���Ƶ�ʺ�ռ�ձȿɵ����Ƽ�ʹ��ATOM2��
P33_10        ����TCĸ����1
P33_13        ����TCĸ����2
-----------------------------------------------------------------
Ĭ����Ļ��ʾ�ӿڣ��û�����ʹ��TFT����OLEDģ��
TFTSPI_CS     P20_14     ����TCĸ�� CS�ܽ� Ĭ�����ͣ����Բ���
TFTSPI_SCK    P20_11     ����TCĸ�� SPI SCK�ܽ�
TFTSPI_SDI    P20_10     ����TCĸ�� SPI MOSI�ܽ�
TFTSPI_DC     P20_12     ����TCĸ�� D/C�ܽ�
TFTSPI_RST    P20_13     ����TCĸ�� RESET�ܽ�
-----------------------------------------------------------------
OLED_CK       P20_14     ����TCĸ��OLED CK�ܽ�
OLED_DI       P20_11     ����TCĸ��OLED DI�ܽ�
OLED_RST      P20_10     ����TCĸ��OLED RST�ܽ�
OLED_DC       P20_12     ����TCĸ��OLED DC�ܽ�
OLED_CS       P20_13     ����TCĸ��OLED CS�ܽ� Ĭ�����ͣ����Բ���
----------------------------------------------------------------
Ĭ�ϰ����ӿ�
KEY0p         P22_0      ����TCĸ���ϰ���0
KEY1p         P22_1      ����TCĸ���ϰ���1
KEY2p         P22_2      ����TCĸ���ϰ���2
DSW0p         P33_9      ����TCĸ���ϲ��뿪��0
DSW1p         P33_11     ����TCĸ���ϲ��뿪��1
-----------------------------------------------------------------
Ĭ��LED�ӿ�
LED0p         P10_6      ����TCĸ����İ���LED0 ����
LED1p         P10_5      ����TCĸ����İ���LED1 ����
LED2p         P20_6      ����TCĸ����LED0
LED3p         P20_7      ����TCĸ����LED1
-----------------------------------------------------------------
��ʱ��
������CCU6ģ��  ÿ��ģ��������������ʱ��  ������ʱ���ж�
�Ƽ�ʹ��CCU6ģ�飬STM����ϵͳʱ�ӻ�����ʱ��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <IfxCpu.h>
#include <IfxScuWdt.h>
#include <LQ_ADC.h>
#include <LQ_CAMERA.h>
#include <LQ_CCU6.h>
#include <LQ_GPIO_LED.h>
#include <LQ_MotorServo.h>
#include <Main.h>
#include <Platform_Types.h>

// ��ʱ�� 5ms��50ms��־λ
volatile uint8 cpu1Flage5ms = 0;
volatile uint8 cpu1Flage50ms = 0;

// �����ٶ�
volatile sint16 targetSpeed = 10;

// ���ϱ�־λ
volatile uint8 evadibleFlage = 0;

// �������ݵ�ѹ
volatile uint16 batvoltage = 0;

volatile uint8 dotcnt=0;
volatile uint8 dotlie[100];
volatile uint8 dothang[100];
volatile short sumlie=0,sumhang=0;
volatile uint8 batchargeflg = 1;
volatile uint8 beaconFlashCnt= 1;


/*************************************************************************
*  �������ƣ�void Seek_Beacon (void)
*  ����˵����CPU1������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��  // ����������Ƶ��ַ��https://space.bilibili.com/95313236
*************************************************************************
*  �������Ǳ����������ܿƼ����޹�˾�����ű�����߳��ϵͳ���Գ��򣬽����ο�
*  ע�����
*  �����������;�Զ����繦�ܣ����޹�ѹ��������
*  ��絽8.5V�����Զ��뿪�����������7V���Ʋ���磻
*  ��ر�֤�������������������ͬʱ�۲��ѹ��ʾ���Է����壻
*  ���ģ������������ɵ��ģ�飻
*  ��������Ϊ100F*5����
*  С��Ϊ���̵ĺ���С���֣�
*  ��Ƭ������ϵͳΪTC264DA��ĸ�壻
*  ����Ϊ˫·DRV8701����ģ�飻
*************************************************************************/
void Seek_Beacon (void)
{
  uint8 nr=0; //��
  uint8 nc=0; //��
  
  dotcnt=0;
  for (nr = 1; nr < LCDH - 1; nr++)
  {
    for (nc = 1; nc < LCDW - 1; nc++)
    {
      if ((Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] > 1))
      {
          dothang[dotcnt]=nr;
          dotlie[dotcnt++]=nc;
      }
    }
  }
  return;
}
/*************************************************************************
*  �������ƣ�int core1_main (void)
*  ����˵����CPU1������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��  // ����������Ƶ��ַ��https://space.bilibili.com/95313236
*************************************************************************/
int core1_main (void)
{
  uint8 tm=0;

  // ����CPU���ж�
  IfxCpu_enableInterrupts();
  
  // �رտ��Ź�
  IfxScuWdt_disableCpuWatchdog (IfxScuWdt_getCpuWatchdogPassword ());
  
  // �ȴ�CPU0 ��ʼ�����
  while(!IfxCpu_acquireMutex(&mutexCpu0InitIsOk));
  
  //��м���ص�ѹ ADC�ɼ���ʼ��
  ADC_InitConfig(ADC7, 80000);//��ʼ��   ���ʹ������ĸ��  ����ѹ��ĵ�ص�ѹ��������Կ�ĸ��ԭ��ͼ
  // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
  mutexCpu0TFTIsOk=1;         // CPU1�� 0ռ��/1�ͷ� TFT
  // ������������������ʼ��
  MotorInit();    // ���
  ServoInit();    // ���
  EncInit();      // ������
  // ����ͷ��ʼ��
//  CAMERA_Init(50);
  // ��ʱ����ʼ��,ԭʼ�жϺ�����CCU6.C�� */
  CCU6_InitConfig(CCU61, CCU6_Channel0, 50000);// 50ms
  MotorCtrl(0,0);// ���û�дﵽ10V���ϣ���ȴ����
  while(batvoltage<1000)// ����10V
  {
      batvoltage= ADC_Read(ADC7); //
      batvoltage= batvoltage * 11 / 25;  // x/4095*3.3*100*5.7

      if(KEY_Read(KEY1)==0)
          break;
  }
  batchargeflg=0;
  while(1)//��ѭ��
  {
    // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
    LED_Ctrl(LED1, RVS);     // LED��˸ ָʾ��������״̬
    if (Camera_Flag == 2)    // 20ms���һ�Σ�������100ms
    {
      Camera_Flag = 0;       // �������ͷ�ɼ���ɱ�־λ  �����������򲻻��ٴβɼ�����
      Get_Use_Image();       // ȡ����������ʾ����ͼ������
      Get_Bin_Image(1);      // ת��Ϊ01��ʽ���ݣ�0��1ԭͼ��2��3������ȡ
      //Bin_Image_Filter();  // �˲������汻Χ�����ݽ����޸�Ϊͬһ��ֵ
      Seek_Beacon();         // ͨ��������ȡ�ҵ��Ƶ����ĵ�
      if(dotcnt) // �����а�ɫ��
      {
          beaconFlashCnt=0;  //��˸���
          sumlie=0,sumhang=0;
          for(tm=0;tm<dotcnt;tm++)
          {
              sumlie+=dotlie[tm];
              sumhang+=dothang[tm];
          }
          sumlie =sumlie/dotcnt;
          sumhang=sumhang/dotcnt;

          MotorDuty1 = MtTargetDuty + ECPULSE1 * 2 - (sumlie-93)* 40;// ���PWM
          MotorDuty2 = MtTargetDuty - ECPULSE2 * 2 + (sumlie-93)* 40;// ˫���
          if(MotorDuty1>ATOM_PWM_MAX) MotorDuty1 =  ATOM_PWM_MAX;//�޷�
          if(MotorDuty2>ATOM_PWM_MAX) MotorDuty2 =  ATOM_PWM_MAX;//�޷�
      }
      else // û�з����а�ɫ��
      {
          beaconFlashCnt++;
          if(beaconFlashCnt>5)           // �������6*20=120msû�з��ֵƣ���ȫ��ԭ�ش�ת
          {
              MotorDuty1 =  ATOM_PWM_MAX;// ���PWM
              MotorDuty2 = -ATOM_PWM_MAX;// ˫���ȫ��ԭ�ش�ת
          }
      }
      
      MotorCtrl(MotorDuty1, MotorDuty2);// ���ֵ������
    }
//    batvoltage= ADC_Read(ADC7); //
//    batvoltage= batvoltage * 11 / 25;  // x/4095*3.3*100*5.7
    //if(batvoltage<800)// ����10V
  }
}
