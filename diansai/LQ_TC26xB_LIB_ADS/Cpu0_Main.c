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
�� ���źţ��ⲿ�жϵ�3�飺P15_1��
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
Ĭ�Ϸ������ӿ�
BEEPp         P33_8      ����TCĸ���Ϸ������ӿ�
-----------------------------------------------------------------
��ʱ��
������CCU6ģ��  ÿ��ģ��������������ʱ��  ������ʱ���ж�
�Ƽ�ʹ��CCU6ģ�飬STM����ϵͳʱ�ӻ�����ʱ��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <include.h>//����ģ���ͷ�ļ�
#include <IfxCpu.h>
#include <IfxScuCcu.h>
#include <IfxScuWdt.h>
#include <IfxStm.h>
#include <IfxStm_reg.h>
#include <LQ_CAMERA.h>
#include <LQ_CCU6.h>
#include <LQ_GPIO_KEY.h>
#include <LQ_GPIO_LED.h>
#include <LQ_MotorServo.h>
#include <LQ_SOFTI2C.h>
#include <LQ_TFT18.h>
#include <LQ_UART.h>
#include <LQ_Inductor.h>
#include <Main.h>
#include "LQ_ImageProcess.h"


// �������ݵ�ѹ
extern volatile uint16 batvoltage;
//extern volatile uint8 dotcnt;
//extern volatile uint8 dotlie[100];
//extern volatile uint8 dothang[100];
extern volatile short sumlie,sumhang;
extern volatile uint8 batchargeflg;
extern unsigned short Threshold;
extern uint8 lqv;

App_Cpu0 g_AppCpu0; // brief CPU 0 global data
IfxCpu_mutexLock mutexCpu0InitIsOk = 1;   // CPU0 ��ʼ����ɱ�־λ
volatile char mutexCpu0TFTIsOk=0;         // CPU1 0ռ��/1�ͷ� TFT

/*************************************************************************
*  �������ƣ�int core0_main (void)
*  ����˵����CPU0������
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
int core0_main (void)
{
  char txt[16];
  uint16 cnt=0;
  // �ر�CPU���ж�
  IfxCpu_disableInterrupts();
  
  // �رտ��Ź�����������ÿ��Ź�ι����Ҫ�ر�
  IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
  IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
  
  // ��ȡ����Ƶ��
  g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
  g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
  g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
  g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);
  
  TFTSPI_Init(0);               // TFT1.8��ʼ��0:����  1������
  TFTSPI_CLS(u16BLACK);         // ����
  TFTSPI_P16x16Str(0,0,(unsigned char*)"�����������ܿƼ�",u16RED,u16BLUE);// �ַ�����ʾ
  
  // ������ʼ��
  GPIO_KEY_Init();
  // LED������P10.6��P10.5��ʼ��
  GPIO_LED_Init();
  
  // ����P14.0�ܽ����,P14.1���룬������115200
  UART_InitConfig(UART0_RX_P14_1,UART0_TX_P14_0, 115200);
  
  // ����CPU���ж�
  IfxCpu_enableInterrupts();
  
  // ֪ͨCPU1��CPU0��ʼ�����
  IfxCpu_releaseMutex(&mutexCpu0InitIsOk);
  // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
  mutexCpu0TFTIsOk=0;// CPU1�� 0ռ��/1�ͷ� TFT

  // �����������Ժ���Ϊ��ѭ�����궨���������ͱ������õģ���������治�����У�
  // TestServo();  // ���Լ��궨�����TFT1.8���
  // TestMotor();  // ���Լ��궨�����TFT1.8���
  // TestEncoder();  // ���Ա�������������,TFT1.8��UART���
  while (1)	//��ѭ��
  {
    // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
    if(KEY_Read(KEY1))
    {
        if(batchargeflg)
        {
            TFTSPI_P8X16Str(0, 4, "Charging...", u16WHITE, u16RED); // �����ʾ
            sprintf(txt, "%02d.%02dV ", batvoltage / 100, batvoltage % 100);// x/4095*3.3*100*5.7
            TFTSPI_P8X16Str(0, 1, txt, u16WHITE, u16BLUE);   // �ַ�����ʾ
        }
        else
        {
            sprintf(txt, "Lamp(%02d,%03d)", sumhang, sumlie);
            TFTSPI_P8X16Str(0, 0, txt, u16RED, u16BLUE);     // ��ʾ����ƫ�����

            sprintf(txt, "%02d.%02dV ", batvoltage / 100, batvoltage % 100);// x/4095*3.3*100*5.7
            TFTSPI_P8X16Str(0, 3, txt, u16WHITE, u16BLUE);   // �ַ�����ʾ
            // ����Ͷ��������ʾ
            sprintf(txt, "M1:%04d, M2:%04d ", MotorDuty1, MotorDuty2);
            TFTSPI_P8X16Str(0, 4, txt, u16RED, u16BLUE);     // ���1-2��ֵ
            sprintf(txt, "E1:%04d, E2:%04d ", ECPULSE1, ECPULSE2);
            TFTSPI_P8X16Str(0, 5, txt, u16RED, u16BLUE);     // ������1-2��ֵ
            sprintf(txt, "thrd:%03d :%03d", Threshold,lqv);
            TFTSPI_P8X16Str(0, 6, txt, u16RED, u16BLUE);     // ������1-2��ֵ
        }
    }
    else  //����K0��
    {
        TFTSPI_ShowBeacon();//��ʾ��������
        sprintf(txt, "thrd:%03d :%03d", Threshold,lqv);
        TFTSPI_P6X8Str(0, 15, txt, u16RED, u16BLUE);     // ������1-2��ֵ
    }
    if(++cnt>1) // ��ֹ����̫��
    {
        cnt=0;
        if(0==KEY_Read(KEY0))
          {
              if(lqv>1) lqv--;   // ��ֵ��һ
          }
          else if(0==KEY_Read(KEY2))
          {
             if(lqv<100) lqv++;  // ��ֵ��һ
          }
    }
    //delayms(2);
  }
}
