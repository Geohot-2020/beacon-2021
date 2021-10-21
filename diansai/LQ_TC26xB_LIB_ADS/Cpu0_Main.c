/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC264DA核心板
【编    写】chiusir
【E-mail】chiusir@163.com
【软件版本】V1.1 版权所有，单位使用请先联系授权
【最后更新】2020年10月28日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】AURIX Development Studio1.2.2及以上版本
【Target 】 TC264DA/TC264D
【Crystal】 20.000Mhz
【SYS PLL】 200MHz
________________________________________________________________
基于iLLD_1_0_1_11_0底层程序,

使用例程的时候，建议采用没有空格的英文路径，
除了CIF为TC264DA独有外，其它的代码兼容TC264D
本库默认初始化了EMEM：512K，如果用户使用TC264D，注释掉EMEM_InitConfig()初始化函数。
工程下\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c第164行左右。
=================================================================
程序配套视频地址：https://space.bilibili.com/95313236
=================================================================
摄像头接口                  龙邱神眼或者OV7725模块
● 数据端口：P02.0-P02.7口，共8位，接摄像头的数据端口；
● 时钟像素：外部中断第0组：P00_4；
● 场信号：外部中断第3组：P15_1；
-----------------------------------------------------------------
推荐GPT12模块，共可以实现5路正交解码增量编码器（兼容带方向编码器）信号采集，任意选择四路即可；
P33_7, P33_6   龙邱TC母板编码器1
P02_8, P33_5   龙邱TC母板编码器2
P10_3, P10_1   龙邱TC母板编码器3
P20_3, P20_0   龙邱TC母板编码器4
-----------------------------------------------------------------
电感电压采集模块或者麦克风模块
推荐使用AN0-7，共八路ADC，可以满足chirp声音信号及电磁车电感电压采集；
AN0-3          龙邱TC接四个麦克风模块或者电感
-----------------------------------------------------------------
默认电机接口
使用GTM模块，ATOM四个通道可产生4*8共32路PWM，而且各自频率和占空比可调，推荐使用ATOM0的0-7通道；
第一组双路全桥驱动
P23_1         龙邱TC母板MOTOR1_P
P32_4         龙邱TC母板MOTOR1_N
P21_2         龙邱TC母板MOTOR2_P
P22_3         龙邱TC母板MOTOR2_N
第二组双路全桥驱动
P21_4         龙邱TC母板MOTOR3_P
P21_3         龙邱TC母板MOTOR3_N
P20_8         龙邱TC母板MOTOR4_P
P21_5         龙邱TC母板MOTOR4_N
-----------------------------------------------------------------
默认舵机接口
使用GTM模块，ATOM四个通道可产生4*8共32路PWM，而且各自频率和占空比可调，推荐使用ATOM2；
P33_10        龙邱TC母板舵机1
P33_13        龙邱TC母板舵机2
-----------------------------------------------------------------
默认屏幕显示接口，用户可以使用TFT或者OLED模块
TFTSPI_CS     P20_14     龙邱TC母板 CS管脚 默认拉低，可以不用
TFTSPI_SCK    P20_11     龙邱TC母板 SPI SCK管脚
TFTSPI_SDI    P20_10     龙邱TC母板 SPI MOSI管脚
TFTSPI_DC     P20_12     龙邱TC母板 D/C管脚
TFTSPI_RST    P20_13     龙邱TC母板 RESET管脚
-----------------------------------------------------------------
OLED_CK       P20_14     龙邱TC母板OLED CK管脚
OLED_DI       P20_11     龙邱TC母板OLED DI管脚
OLED_RST      P20_10     龙邱TC母板OLED RST管脚
OLED_DC       P20_12     龙邱TC母板OLED DC管脚
OLED_CS       P20_13     龙邱TC母板OLED CS管脚 默认拉低，可以不用
----------------------------------------------------------------
默认按键接口
KEY0p         P22_0      龙邱TC母板上按键0
KEY1p         P22_1      龙邱TC母板上按键1
KEY2p         P22_2      龙邱TC母板上按键2
DSW0p         P33_9      龙邱TC母板上拨码开关0
DSW1p         P33_11     龙邱TC母板上拨码开关1
-----------------------------------------------------------------
默认LED接口
LED0p         P10_6      龙邱TC母板核心板上LED0 翠绿
LED1p         P10_5      龙邱TC母板核心板上LED1 蓝灯
LED2p         P20_6      龙邱TC母板上LED0
LED3p         P20_7      龙邱TC母板上LED1
-----------------------------------------------------------------
默认蜂鸣器接口
BEEPp         P33_8      龙邱TC母板上蜂鸣器接口
-----------------------------------------------------------------
定时器
有两个CCU6模块  每个模块有两个独立定时器  触发定时器中断
推荐使用CCU6模块，STM用作系统时钟或者延时；
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <include.h>//各个模块的头文件
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


// 超级电容电压
extern volatile uint16 batvoltage;
//extern volatile uint8 dotcnt;
//extern volatile uint8 dotlie[100];
//extern volatile uint8 dothang[100];
extern volatile short sumlie,sumhang;
extern volatile uint8 batchargeflg;
extern unsigned short Threshold;
extern uint8 lqv;

App_Cpu0 g_AppCpu0; // brief CPU 0 global data
IfxCpu_mutexLock mutexCpu0InitIsOk = 1;   // CPU0 初始化完成标志位
volatile char mutexCpu0TFTIsOk=0;         // CPU1 0占用/1释放 TFT

/*************************************************************************
*  函数名称：int core0_main (void)
*  功能说明：CPU0主函数
*  参数说明：无
*  函数返回：无
*  修改时间：2020年3月10日
*  备    注：  // 程序配套视频地址：https://space.bilibili.com/95313236
*************************************************************************
*  本程序是北京龙邱智能科技有限公司测试信标灯无线充电系统测试程序，仅供参考
*  注意事项：
*  本程序加有中途自动补电功能，并无过压保护；，
*  充电到8.5V以上自动离开充电区，低于7V遇灯补充电；
*  务必保证驱动及电机正常工作，同时观察电压表示数以防过冲；
*  充电模块采用龙邱店铺傻充模块；
*  超级电容为100F*5串；
*  小车为店铺的黑马小三轮；
*  单片机控制系统为TC264DA子母板；
*  驱动为双路DRV8701驱动模块；
*************************************************************************/
int core0_main (void)
{
  char txt[16];
  uint16 cnt=0;
  // 关闭CPU总中断
  IfxCpu_disableInterrupts();
  
  // 关闭看门狗，如果不设置看门狗喂狗需要关闭
  IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
  IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
  
  // 读取总线频率
  g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
  g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
  g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
  g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);
  
  TFTSPI_Init(0);               // TFT1.8初始化0:横屏  1：竖屏
  TFTSPI_CLS(u16BLACK);         // 清屏
  TFTSPI_P16x16Str(0,0,(unsigned char*)"北京龙邱智能科技",u16RED,u16BLUE);// 字符串显示
  
  // 按键初始化
  GPIO_KEY_Init();
  // LED灯所用P10.6和P10.5初始化
  GPIO_LED_Init();
  
  // 串口P14.0管脚输出,P14.1输入，波特率115200
  UART_InitConfig(UART0_RX_P14_1,UART0_TX_P14_0, 115200);
  
  // 开启CPU总中断
  IfxCpu_enableInterrupts();
  
  // 通知CPU1，CPU0初始化完成
  IfxCpu_releaseMutex(&mutexCpu0InitIsOk);
  // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
  mutexCpu0TFTIsOk=0;// CPU1： 0占用/1释放 TFT

  // 以下三个测试函数为死循环，标定舵机、电机和编码器用的，开启后后面不会运行！
  // TestServo();  // 测试及标定舵机，TFT1.8输出
  // TestMotor();  // 测试及标定电机，TFT1.8输出
  // TestEncoder();  // 测试编码器正交解码,TFT1.8和UART输出
  while (1)	//主循环
  {
    // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
    if(KEY_Read(KEY1))
    {
        if(batchargeflg)
        {
            TFTSPI_P8X16Str(0, 4, "Charging...", u16WHITE, u16RED); // 充电提示
            sprintf(txt, "%02d.%02dV ", batvoltage / 100, batvoltage % 100);// x/4095*3.3*100*5.7
            TFTSPI_P8X16Str(0, 1, txt, u16WHITE, u16BLUE);   // 字符串显示
        }
        else
        {
            sprintf(txt, "Lamp(%02d,%03d)", sumhang, sumlie);
            TFTSPI_P8X16Str(0, 0, txt, u16RED, u16BLUE);     // 显示赛道偏差参数

            sprintf(txt, "%02d.%02dV ", batvoltage / 100, batvoltage % 100);// x/4095*3.3*100*5.7
            TFTSPI_P8X16Str(0, 3, txt, u16WHITE, u16BLUE);   // 字符串显示
            // 电机和舵机参数显示
            sprintf(txt, "M1:%04d, M2:%04d ", MotorDuty1, MotorDuty2);
            TFTSPI_P8X16Str(0, 4, txt, u16RED, u16BLUE);     // 电机1-2数值
            sprintf(txt, "E1:%04d, E2:%04d ", ECPULSE1, ECPULSE2);
            TFTSPI_P8X16Str(0, 5, txt, u16RED, u16BLUE);     // 编码器1-2数值
            sprintf(txt, "thrd:%03d :%03d", Threshold,lqv);
            TFTSPI_P8X16Str(0, 6, txt, u16RED, u16BLUE);     // 编码器1-2数值
        }
    }
    else  //按下K0键
    {
        TFTSPI_ShowBeacon();//显示赛道画面
        sprintf(txt, "thrd:%03d :%03d", Threshold,lqv);
        TFTSPI_P6X8Str(0, 15, txt, u16RED, u16BLUE);     // 编码器1-2数值
    }
    if(++cnt>1) // 防止按键太快
    {
        cnt=0;
        if(0==KEY_Read(KEY0))
          {
              if(lqv>1) lqv--;   // 阈值减一
          }
          else if(0==KEY_Read(KEY2))
          {
             if(lqv<100) lqv++;  // 阈值加一
          }
    }
    //delayms(2);
  }
}
