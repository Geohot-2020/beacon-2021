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
● 场信号：    外部中断第3组：P15_1；
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
定时器
有两个CCU6模块  每个模块有两个独立定时器  触发定时器中断
推荐使用CCU6模块，STM用作系统时钟或者延时；
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

// 定时器 5ms和50ms标志位
volatile uint8 cpu1Flage5ms = 0;
volatile uint8 cpu1Flage50ms = 0;

// 期望速度
volatile sint16 targetSpeed = 10;

// 避障标志位
volatile uint8 evadibleFlage = 0;

// 超级电容电压
volatile uint16 batvoltage = 0;

volatile uint8 dotcnt=0;
volatile uint8 dotlie[100];
volatile uint8 dothang[100];
volatile short sumlie=0,sumhang=0;
volatile uint8 batchargeflg = 1;
volatile uint8 beaconFlashCnt= 1;


/*************************************************************************
*  函数名称：void Seek_Beacon (void)
*  功能说明：CPU1主函数
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
void Seek_Beacon (void)
{
  uint8 nr=0; //行
  uint8 nc=0; //列
  
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
*  函数名称：int core1_main (void)
*  功能说明：CPU1主函数
*  参数说明：无
*  函数返回：无
*  修改时间：2020年3月10日
*  备    注：  // 程序配套视频地址：https://space.bilibili.com/95313236
*************************************************************************/
int core1_main (void)
{
  uint8 tm=0;

  // 开启CPU总中断
  IfxCpu_enableInterrupts();
  
  // 关闭看门狗
  IfxScuWdt_disableCpuWatchdog (IfxScuWdt_getCpuWatchdogPassword ());
  
  // 等待CPU0 初始化完成
  while(!IfxCpu_acquireMutex(&mutexCpu0InitIsOk));
  
  //电感及电池电压 ADC采集初始化
  ADC_InitConfig(ADC7, 80000);//初始化   如果使用龙邱母板  则测分压后的电池电压，具体可以看母板原理图
  // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
  mutexCpu0TFTIsOk=1;         // CPU1： 0占用/1释放 TFT
  // 电机、舵机，编码器初始化
  MotorInit();    // 电机
  ServoInit();    // 舵机
  EncInit();      // 编码器
  // 摄像头初始化
//  CAMERA_Init(50);
  // 定时器初始化,原始中断函数在CCU6.C中 */
  CCU6_InitConfig(CCU61, CCU6_Channel0, 50000);// 50ms
  MotorCtrl(0,0);// 充电没有达到10V以上，则等待充电
  while(batvoltage<1000)// 大于10V
  {
      batvoltage= ADC_Read(ADC7); //
      batvoltage= batvoltage * 11 / 25;  // x/4095*3.3*100*5.7

      if(KEY_Read(KEY1)==0)
          break;
  }
  batchargeflg=0;
  while(1)//主循环
  {
    // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
    LED_Ctrl(LED1, RVS);     // LED闪烁 指示程序运行状态
    if (Camera_Flag == 2)    // 20ms检测一次，灯闪是100ms
    {
      Camera_Flag = 0;       // 清除摄像头采集完成标志位  如果不清除，则不会再次采集数据
      Get_Use_Image();       // 取出赛道及显示所需图像数据
      Get_Bin_Image(1);      // 转换为01格式数据，0、1原图；2、3边沿提取
      //Bin_Image_Filter();  // 滤波，三面被围的数据将被修改为同一数值
      Seek_Beacon();         // 通过边沿提取找到灯的重心点
      if(dotcnt) // 发现有白色点
      {
          beaconFlashCnt=0;  //闪烁检测
          sumlie=0,sumhang=0;
          for(tm=0;tm<dotcnt;tm++)
          {
              sumlie+=dotlie[tm];
              sumhang+=dothang[tm];
          }
          sumlie =sumlie/dotcnt;
          sumhang=sumhang/dotcnt;

          MotorDuty1 = MtTargetDuty + ECPULSE1 * 2 - (sumlie-93)* 40;// 电机PWM
          MotorDuty2 = MtTargetDuty - ECPULSE2 * 2 + (sumlie-93)* 40;// 双电机
          if(MotorDuty1>ATOM_PWM_MAX) MotorDuty1 =  ATOM_PWM_MAX;//限幅
          if(MotorDuty2>ATOM_PWM_MAX) MotorDuty2 =  ATOM_PWM_MAX;//限幅
      }
      else // 没有发现有白色点
      {
          beaconFlashCnt++;
          if(beaconFlashCnt>5)           // 五次以上6*20=120ms没有发现灯，则全速原地打转
          {
              MotorDuty1 =  ATOM_PWM_MAX;// 电机PWM
              MotorDuty2 = -ATOM_PWM_MAX;// 双电机全速原地打转
          }
      }
      
      MotorCtrl(MotorDuty1, MotorDuty2);// 四轮电机驱动
    }
//    batvoltage= ADC_Read(ADC7); //
//    batvoltage= batvoltage * 11 / 25;  // x/4095*3.3*100*5.7
    //if(batvoltage<800)// 大于10V
  }
}
