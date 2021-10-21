/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 ��ƽ    ̨�������������ܿƼ�TC264DA���İ�
 ����    д��ZYF/chiusir
 ��E-mail  ��chiusir@163.com
 �������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
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
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ
 *  ��    ע��TC264ֻ��15���̶���GPIO֧���ⲿ�ж� ��15���̶�GPIO��Ϊ4�飬ÿ��ֻ��ѡ������һ����Ϊ�ⲿ�жϹܽ�ʹ��
 *           0��P15_4  P33_7  P15_5                             1��P14_3  P15_8
 *           2��P10_2  P02_1  P00_4  P20_0  P11_10              3��P10_3  P14_1  P02_0  P20_9  P15_1
 *			�ⲿ�жϷ�������LQ_GPIO.c��  �ж����ȼ�������LQ_GPIO.h�� ���������޸�
 *
 ________________________________________________________________
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 ����ͷ�ܽ�  TC264DA�ܽ�   ˵��              DMA��ʽ�ɼ����ùܽ�ͨ����������
 D0        P02_0       ����ͷ����bit0
 D1        P02_1       ����ͷ����bit1
 D2        P02_2       ����ͷ����bit2
 D3        P02_3       ����ͷ����bit3
 D4        P02_4       ����ͷ����bit4
 D5        P02_5       ����ͷ����bit5
 D6        P02_6       ����ͷ����bit6
 D7        P02_7       ����ͷ����bit7
 CLK       P00_4       ����ʱ��        �ⲿ�жϵ�2�飺P00_4
 VSNC      P15_1       ���ź�             �ⲿ�жϵ�3�飺P15_1

 SCL       P11_2       ����ģ��SCCB SCL
 SDA       P11_3       ����ģ��SCCB SDA
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <IfxCpu.h>
#include <IfxPort_reg.h>
#include <LQ_CAMERA.h>
#include <LQ_DMA.h>
#include <LQ_GPIO.h>
#include <LQ_GPIO_LED.h>
#include <LQ_TFT18.h>
#include <LQ_UART.h>
#include <LQ_STM.h>
#include <Platform_Types.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <include.h>

/** ͼ��ԭʼ���ݴ�� */
unsigned char Image_Data[IMAGEH][IMAGEW];

/** ѹ����֮�����ڴ����Ļ��ʾ����  */
unsigned char Image_Use[LCDH][LCDW];

/** ��ֵ��������OLED��ʾ������ */
unsigned char Bin_Image[LCDH][LCDW];

unsigned short Threshold = 0;
uint8 lqv = 35;
sint16 OFFSET0 = 0;      //��Զ������������ֵ�ۺ�ƫ����
sint16 OFFSET1 = 0;      //�ڶ���
sint16 OFFSET2 = 0;      //�����������
sint16 TXV = 0;          //���ε���߶ȣ��Ҹ߶�

/*!
 * @brief    ����ͷ��������
 *
 * @param
 *
 * @return
 *
����MT9V034  ע����Ҫʹ��  ������Ű棨��ɫ��ת����
 *
 * @example
 *
 * @date     2019/10/22 ���ڶ�
 */
void Test_CAMERA (void)
{

#ifdef USEOLED
    OLED_Init();
    OLED_CLS();                   //LCD����
#else
    TFTSPI_Init(0);               //TFT1.8��ʼ��0:����  1������
    TFTSPI_CLS(u16BLUE);          //����
#endif

    /* ����ͷ��ʼ�� */
    CAMERA_Init(50);

    while (1)
    {
        if (Camera_Flag == 2)
        {
            //�ϱ����ݸ���λ�� �����ٶȱȽ��� ע����λ��ͼ���������Ϊ120*188
            //CAMERA_Reprot();

            /* ��ȡ����ʹ�õ����� */
            Get_Use_Image();

            /* �������ͷ�ɼ���ɱ�־λ  �����������򲻻��ٴβɼ����� */
            Camera_Flag = 0;

#if 0       //��ʾԭʼͼ��
            //TFT1.8��̬��ʾ����ͷͼ��
            TFTSPI_Road(0, 0, LCDH, LCDW, (unsigned char *)Image_Use);

#else       //��ʾ��ֵ��ͼ��
            /* ��ֵ�� */
            Get_Bin_Image(2);

            // ��ʾ����ͷͼ��
            TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *) Bin_Image);
#endif

            LED_Ctrl(LED0, RVS);

        }
    }
}

/*!
 * @brief    �����ϱ���λ��
 *
 * @param    ��
 *
 * @return   ��
 *
��λ����֡ͷ������������
 *
 * @see      CAMERA_Reprot();
 *
 * @date     2019/9/24 ���ڶ�
 */
void CAMERA_Reprot (void)
{
    short j, i;

    UART_PutChar(UART0, 0xfe);  //֡ͷ
    UART_PutChar(UART0, 0xef);  //֡ͷ

    for (i = 0; i < IMAGEH; i++)
    {
        for (j = 0; j < IMAGEW; j++)
        {
            if (Image_Data[i][j] == 0xfe)  //��ֹ������֡β
            {
                Image_Data[i][j] = 0xff;
            }
            UART_PutChar(UART0, Image_Data[i][j]); //��������

        }
    }
    UART_PutChar(UART0, 0xef);  //֡β
    UART_PutChar(UART0, 0xfe);  //֡β

}

/*!
 * @brief    ����ͷ��ʼ��
 *
 * @param    fps:  ֡��
 *
 * @return   ��
 *
����ͷ��һЩ��������LQ_MT9V034.c�еĺ궨�����޸�
 *
 * @see      CAMERA_Init(50);   //��ʼ��MT9V034  50֡ ע��ʹ�ð�ɫ������Ű�ת����
 *
 * @date     2019/10/22 ���ڶ�
 */
void CAMERA_Init (unsigned char fps)
{
    //�ر�CPU���ж�
    IfxCpu_disableInterrupts();
    
    /* ��ʼ������ͷ ����IO */
    PIN_InitConfig(P02_0, PIN_MODE_INPUT_PULLDOWN, 0);
    PIN_InitConfig(P02_1, PIN_MODE_INPUT_PULLDOWN, 0);
    PIN_InitConfig(P02_2, PIN_MODE_INPUT_PULLDOWN, 0);
    PIN_InitConfig(P02_3, PIN_MODE_INPUT_PULLDOWN, 0);
    PIN_InitConfig(P02_4, PIN_MODE_INPUT_PULLDOWN, 0);
    PIN_InitConfig(P02_5, PIN_MODE_INPUT_PULLDOWN, 0);
    PIN_InitConfig(P02_6, PIN_MODE_INPUT_PULLDOWN, 0);
    PIN_InitConfig(P02_7, PIN_MODE_INPUT_PULLDOWN, 0);

    MT9V034_Init(fps);

    /* DMA ����Դ��ʼ�� */
    PIN_Exti(P00_4, PIN_IRQ_MODE_FALLING);

    /* DMA ��ʼ�� */
    DMA_CameraInitConfig((unsigned long) (&MODULE_P02.IN.U), (unsigned long) Image_Data, PIN_INT2_PRIORITY);
    
    /* ʹ�ܳ��ж� */
    PIN_Exti(P15_1, PIN_IRQ_MODE_RISING);

    /* ��ʼ����� ���ж� */
    IfxCpu_enableInterrupts();

}

/*************************************************************************
 *  �������ƣ�void Get_Use_Image (void)
 *  ����˵����������ͷ�ɼ���ԭʼͼ�����ŵ�����ʶ�������С
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��10��28��
 *  ��    ע��  IMAGEWΪԭʼͼ��Ŀ��ȣ�����Ϊ188��OV7725Ϊ320
 *       IMAGEHΪԭʼͼ��ĸ߶ȣ�����Ϊ120��OV7725Ϊ240
 *************************************************************************/
void Get_Use_Image(void)
{
    short i = 0, j = 0, row = 0, line = 0;

    for (i = 0; i < IMAGEH; i ++)          //���۸� 120
    //for (i = 0; i < IMAGEH; i += 2)          //���۸� 120 / 2  = 60��
    // for (i = 0; i < IMAGEH; i += 3)       //OV7725�� 240 / 3  = 80��
    {
        for (j = 0; j <= IMAGEW; j ++)     //���ۿ�188
        //for (j = 0; j <= IMAGEW; j += 2)     //���ۿ�188 / 2  = 94��
        // for (j = 0; j <= IMAGEW; j += 3)  //OV7725��320 / 3  = 106��
        {
            Image_Use[row][line] = Image_Data[i][j];
            line++;
        }
        line = 0;
        row++;
    }
}

/*************************************************************************
 *  �������ƣ�void Get_Bin_Image (unsigned char mode)
 *  ����˵����ͼ���ֵ����Bin_Image[][]
 *  ����˵����mode  ��
 *    0��ʹ�ô����ֵ
 *    1��ʹ��ƽ����ֵ
 *    2: sobel ���ӸĽ���  �ֶ���ֵ��ͬʱ�����Ϊ��ȡ���ص�ͼ��
 *    3��sobel ���ӸĽ���   ��̬��ֵ��ͬʱ�����Ϊ��ȡ���ص�ͼ��
 *  �������أ���
 *  �޸�ʱ�䣺2020��10��28��
 *  ��    ע��  Get_Bin_Image(0); //ʹ�ô�򷨶�ֵ��
 *************************************************************************/
void Get_Bin_Image (unsigned char mode)
{
    unsigned short i = 0, j = 0;
    unsigned long  tv = 0;
    //char txt[16];

    if (mode == 0)
    {
        Threshold = GetOSTU(Image_Use);  //�����ֵ
    }
    else if (mode == 1)
    {
        //�ۼ�
        for (i = 0; i < LCDH; i++)
        {
            for (j = 0; j < LCDW; j++)
            {
                tv += Image_Use[i][j];   //�ۼ�
            }
        }
        Threshold =(unsigned short)(tv / LCDH / LCDW);   //��ƽ��ֵ,����Խ��ԽС��ȫ��Լ35��������ĻԼ160��һ������´�Լ100
        Threshold = Threshold + lqv;      //�˴���ֵ���ã����ݻ����Ĺ������趨
    }
    else if (mode == 2)
    {
        Threshold = 80;                          //�ֶ�������ֵ
        lq_sobel(Image_Use, Bin_Image, (unsigned char) Threshold);

        return;

    }
    else if (mode == 3)
    {
        lq_sobelAutoThreshold(Image_Use, Bin_Image);  //��̬������ֵ
        return;
    }
/*
#ifdef USEOLED
    sprintf(txt,"%03d",Threshold);//��ʾ��ֵ
	OLED_P6x8Str(80,0,txt);
#else
    sprintf(txt, "%03d", Threshold);  //��ʾ��ֵ
    TFTSPI_P6X8Str(0,7, txt, u16RED, u16BLUE);
#endif
*/
    /* ��ֵ�� */
    for (i = 0; i < LCDH; i++)
    {
        for (j = 0; j < LCDW; j++)
        {
            if (Image_Use[i][j] > Threshold) //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
                Bin_Image[i][j] = 1;
            else
                Bin_Image[i][j] = 0;
        }
    }
}
/*************************************************************************
 *  �������ƣ�short GetOSTU (unsigned char tmImage[LCDH][LCDW])
 *  ����˵�����������ֵ��С
 *  ����˵����tmImage �� ͼ������
 *  �������أ���
 *  �޸�ʱ�䣺2011��10��28��
 *  ��    ע��  GetOSTU(Image_Use);//�����ֵ
Ostu������������������ͨ��ͳ������ͼ���ֱ��ͼ������ʵ��ȫ����ֵT���Զ�ѡȡ�����㷨����Ϊ��
1) �ȼ���ͼ���ֱ��ͼ������ͼ�����е����ص㰴��0~255��256��bin��ͳ������ÿ��bin�����ص�����
2) ��һ��ֱ��ͼ��Ҳ����ÿ��bin�����ص����������ܵ����ص�
3) i��ʾ�������ֵ��Ҳ��һ���Ҷȼ�����0��ʼ���� 1
4) ͨ����һ����ֱ��ͼ��ͳ��0~i �Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ���ǰ������) ��ռ����ͼ��
        �ı���w0��        ��ͳ��ǰ�����ص�ƽ���Ҷ�u0��ͳ��i~255�Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ�����
        ������)  * ��ռ����ͼ��ı���w1����ͳ�Ʊ������ص�ƽ���Ҷ�u1��
5) ����ǰ�����غͱ������صķ��� g = w0*w1*(u0-u1) (u0-u1)
6) i++��ת��4)��ֱ��iΪ256ʱ��������
7) �����g��Ӧ��iֵ��Ϊͼ���ȫ����ֵ
ȱ��:OSTU�㷨�ڴ������ղ����ȵ�ͼ���ʱ��Ч�������Բ��ã���Ϊ���õ���ȫ��������Ϣ��
*************************************************************************/
short GetOSTU (unsigned char tmImage[LCDH][LCDW])
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��;
    signed short MinValue, MaxValue;
    signed short Threshold = 0;
    unsigned char HistoGram[256];              //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ

    for (j = 0; j < LCDH; j++)
    {
        for (i = 0; i < LCDW; i++)
        {
            HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //��ȡ���Ҷȵ�ֵ

    if (MaxValue == MinValue)
        return MaxValue;         // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue)
        return MinValue;        // ͼ����ֻ�ж�����ɫ

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  ��������

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //�Ҷ�ֵ����
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //ǰ�����ص���
        PixelFore = Amount - PixelBack;           //�������ص���
        OmegaBack = (float) PixelBack / Amount;   //ǰ�����ذٷֱ�
        OmegaFore = (float) PixelFore / Amount;   //�������ذٷֱ�
        PixelshortegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //�����Ҷ�ֵ
        MicroBack = (float) PixelshortegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
        MicroFore = (float) PixelshortegralFore / PixelFore;   //�����ҶȰٷֱ�
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //������䷽��
        if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //���������ֵ;
}

/*!
 * @brief    ����soble���ؼ�����ӵ�һ�ֱ��ؼ��
 *
 * @param    imageIn    ��������
 *           imageOut   �������      ����Ķ�ֵ����ı�����Ϣ
 *           Threshold  ��ֵ
 *
 * @return
 *
 * @note
 *
 * @example
 *
 * @date     2020/5/15
 */
void lq_sobel (unsigned char imageIn[LCDH][LCDW], unsigned char imageOut[LCDH][LCDW], unsigned char Threshold)
{
    /** �����˴�С */
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = LCDW - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = LCDH - KERNEL_SIZE / 2;
    short i, j, k;
    short temp[4];
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            /* ���㲻ͬ�����ݶȷ�ֵ  */
            temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
            - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]        // {-1, 0, 1},
            - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};

            temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
            - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]       // { 0,  0,  0},
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  1,  1}};

            temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
            - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]       //  1,  0, -1
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];    //  1,  1,  0

            temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
            - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]       // -1,  0,  1
            - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];    //  0,  1,  1

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
            temp[2] = abs(temp[2]);
            temp[3] = abs(temp[3]);

            /* �ҳ��ݶȷ�ֵ���ֵ  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            if (temp[0] > Threshold)
            {
                imageOut[i][j] = 1;
            }
            else
            {
                imageOut[i][j] = 0;
            }
        }
    }
}

/*!
 * @brief    ����soble���ؼ�����ӵ�һ���Զ���ֵ���ؼ��
 *
 * @param    imageIn    ��������
 *           imageOut   �������      ����Ķ�ֵ����ı�����Ϣ
 *
 * @return
 *
 * @note
 *
 * @example
 *
 * @date     2020/5/15
 */
void lq_sobelAutoThreshold (unsigned char imageIn[LCDH][LCDW], unsigned char imageOut[LCDH][LCDW])
{
    /** �����˴�С */
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = LCDW - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = LCDH - KERNEL_SIZE / 2;
    short i, j, k;
    short temp[4];
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            /* ���㲻ͬ�����ݶȷ�ֵ  */
            temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
            - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]       // {-1, 0, 1},
            - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};

            temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
            - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]       // { 0,  0,  0},
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  1,  1}};

            temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
            - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]       //  1,  0, -1
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];    //  1,  1,  0

            temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
            - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]       // -1,  0,  1
            - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];    //  0,  1,  1

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
            temp[2] = abs(temp[2]);
            temp[3] = abs(temp[3]);

            /* �ҳ��ݶȷ�ֵ���ֵ  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* ʹ�����ص����������ص�֮�͵�һ������    ��Ϊ��ֵ  */
            temp[3] = (short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
                    + (short) imageIn[i][j - 1] + (short) imageIn[i][j] + (short) imageIn[i][j + 1]
                    + (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];

            if (temp[0] > temp[3] / 12.0f)
            {
                imageOut[i][j] = 1;
            }
            else
            {
                imageOut[i][j] = 0;
            }

        }
    }
}

/*---------------------------------------------------------------
 ����    ����Bin_Image_Filter
 ����    �ܡ��������
 ����    ������
 ���� �� ֵ����
 ��ע�����
 ----------------------------------------------------------------*/
void Bin_Image_Filter (void)
{
    sint16 nr; //��
    sint16 nc; //��

    for (nr = 1; nr < LCDH - 1; nr++)
    {
        for (nc = 1; nc < LCDW - 1; nc = nc + 1)
        {
            if ((Bin_Image[nr][nc] == 0)
                    && (Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] > 2))
            {
                Bin_Image[nr][nc] = 1;
            }
            else if ((Bin_Image[nr][nc] == 1)
                    && (Bin_Image[nr - 1][nc] + Bin_Image[nr + 1][nc] + Bin_Image[nr][nc + 1] + Bin_Image[nr][nc - 1] < 2))
            {
                Bin_Image[nr][nc] = 0;
            }
        }
    }
}

/***************************************************************************
 *                                                                          *
 *  �������ƣ�Seek_Road(void)                                           *
 *  ����˵����Ѱ�Ұ�ɫ����ƫ��ֵ                                            *
 *  ����˵������                                                            *
 *  �������أ�ֵ�Ĵ�С                                                      *
 *  �޸�ʱ�䣺2017-07-16                                                    *
 *  ��    ע�����м�Ϊ0������һ���Ҳ��һ����ֵ����1�����                *
 *            ��������ӵ�һ�п�ʼ�������ڶ��н�����                        *
 *            ������Ϊ��������ֵԽ��˵��Խƫ��ߣ�                        *
 *            ������Ϊ��������ֵԽ��˵��Խƫ�ұߡ�                        *
 ***************************************************************************/
void Seek_Road (void)
{
    sint16 nr; //��
    sint16 nc; //��
    sint16 temp = 0; //��ʱ��ֵ
    //for(nr=1; nr<MAX_ROW-1; nr++)
    temp = 0;
    for (nr = 8; nr < 24; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (Bin_Image[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (Bin_Image[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET0 = temp;
    temp = 0;
    for (nr = 24; nr < 40; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (Bin_Image[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (Bin_Image[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET1 = temp;
    temp = 0;
    for (nr = 40; nr < 56; nr++)
    {
        for (nc = MAX_COL / 2; nc < MAX_COL; nc = nc + 1)
        {
            if (Bin_Image[nr][nc])
            {
                ++temp;
            }
        }
        for (nc = 0; nc < MAX_COL / 2; nc = nc + 1)
        {
            if (Bin_Image[nr][nc])
            {
                --temp;
            }
        }
    }
    OFFSET2 = temp;
    return;
}