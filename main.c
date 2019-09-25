#include <msp430.h> 
#include "HT1621.h"
#include "I2C.h"
#include "LCD_128.h"
#include "TCA6416A.h"
#define DCOFFSET  512
/*
 * 测量交流幅度值和直流偏置
 */

unsigned char SampleFlag=0;		//SampleFlag=1: P1.1采样测峰峰值,SampleFlag=2:P1.2采样测直流偏置,SampleFlag=3:测频模式
unsigned int Vpptmp[2]={DCOFFSET,DCOFFSET};
long Vppsum=0;					//P11接峰峰值输入
long Vdcsum=0;					//P12接直流偏置输入
long Vpp=0;			//Vpp测量值
long Vdc=0;			//直流偏置测量值
unsigned char updateFlag=0;
unsigned char sampletime=0;
unsigned int VppADCsum=0;		//峰峰值采样结果的求和
unsigned int VdcADCsum=0;		//直流偏置采样结果的求和
unsigned int Vppave=0,Vdcave=0;
unsigned int Freq=0;			//频率

void P11Sample_Init();			//P1.1口采样初始化
//void P12Sample_Init();			//P1.2口采样初始化
void MeasureVpp_Init();
void MeasureF_Init();
void Display_Vpp(unsigned int Volt);		//在大LCD上三位有效数字显示
void Display_Vdc(unsigned int Volt);		//在小LCD上显示电压
void Dsiplay_Freq(unsigned int freq);		//大号LCD上面四位数字显示频率
void I2C_IODect();		//检测I/O的函数
void Display_RealV(long Volt);
void Display_dc(long Volt);
long ComputeAC(long Volt);
long ComputeDC(long Volt);

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	BCSCTL1 = CALBC1_16MHZ;      /* Set DCO to 1MHz */
    DCOCTL = CALDCO_16MHZ;
    TCA6416A_Init();			//初始化I2C和TCA6426寄存器
    HT1621_init();				// 初始化lcd_128，这个千万不能漏！！！
    LCD_Clear();				//清空屏幕
    HT1621_Reflash(LCD_Buffer);	//记得改完RAM缓存之后更新显存

//	P11Sample_Init();
	SampleFlag=0;
	unsigned char Vppcount=0,Vdccount=0;
	
//	ADC10CTL0 |= ADC10SC;
//	MeasureF_Init();
	__bis_SR_register(GIE);

	while(1){
		PinIN();
		I2C_IODect();
		if(SampleFlag==1){		//测量幅度值
			if(updateFlag==1){
				Vppsum+=Vppave;
				Vppcount++;
				if(Vppcount==10){
					Vppsum=Vppsum/10;
					Vpp=(unsigned int)(2500*Vppsum/1024);
					Vppsum=0;
					Vppcount=0;		//归零
					//Vpptmp[0]=Vpptmp[1]=0;
					sampletime=0;

					//测完峰峰值切换测量偏置电压
					ADC10CTL0 &= ~ENC;
					ADC10CTL1 = CONSEQ_0 | ADC10SSEL_0 | ADC10DIV_0 | SHS_0 | INCH_2;
					ADC10AE0 = 0x4;
					Vpptmp[0]=0;Vpptmp[1]=0x3ff;
					__delay_cycles(30000);
					ADC10CTL0 |= ENC;
					SampleFlag=2;
					sampletime=0;
					ADC10CTL0 |= ADC10SC;
					Vpp-=25;
					Vpp=ComputeAC(Vpp);
					Display_RealV(Vpp);
					//Display_Vpp(Vpp);
				}
				updateFlag=0;
			}
		}
		else if(SampleFlag==2){
			if(updateFlag==1){
				Vdcsum+=Vdcave;
				Vdccount++;
				if(Vdccount==10){
					Vdcsum=Vdcsum/10;		//求和取平均
					Vdc=(unsigned int)(2500*Vdcsum/1024);
					Vdc=Vdc/2;
					Vdcsum=0;
					Vdccount=0;
					//Vpptmp[0]=Vpptmp[1]=0;
					sampletime=0;

					//测完直流偏置变成测量交流
					ADC10CTL0 &= ~ENC;
					ADC10CTL1 = CONSEQ_0 | ADC10SSEL_0 | ADC10DIV_0 | SHS_0 | INCH_1;
					ADC10AE0 = 0x2;

					Vpptmp[0]=Vpptmp[1]=DCOFFSET;
					__delay_cycles(3000);
					ADC10CTL0 |= ENC;

					SampleFlag=1;

					ADC10CTL0 |= ADC10SC;
					//Display_Vdc(Vdc);
					Vdc=ComputeDC(Vdc);
					Display_dc(Vdc);
				}
			updateFlag=0;		//等待下次取值完成
			}
		}
		else if(SampleFlag==3){
			Dsiplay_Freq(Freq);
		}
	}
}

long ComputeAC(long Volt)
{
	long tmp=Volt;
	tmp=1072*tmp+2952;
	Volt=tmp/100;
	if(Volt<1400){
		Volt=Volt-255;
	}
	else if(Volt>15100){
		Volt+=500;
	}
	return Volt;
}

long ComputeDC(long Volt)
{
	long tmp=Volt;
	tmp=3388*tmp-1354200;
	Volt=tmp/100;
	Volt-=1000;
	return Volt;
}

void I2C_IODect()			                 //检测事件确实发生了
{
	static unsigned char KEY_Past=0,KEY_Now=0;
	KEY_Past=KEY_Now;
	//----判断I2C_IO10所连的KEY1按键是否被按下------
	if((TCA6416A_InputBuffer&BIT8) == BIT8)
		KEY_Now |=BIT0;
	else
		KEY_Now &=~BIT0;
	if(((KEY_Past&BIT0)==BIT0)&&(KEY_Now&BIT0) !=BIT0){
		//KEY1被按下，切换成测量VPP模式
		_disable_interrupts();		//关闭中断
		unsigned char i=0;
		for(i=0;i<8;i++){
			PinOUT(i,1);
		}
		PinOUT(0,0);		//点亮LED1
		SampleFlag=1;
	    LCD_ClearSeg(43);	//清除KHZ
	    LCD_ClearSeg(47);
		LCD_DisplayDigit(20,3);
		LCD_ClearSeg(24);			//清除小数点
		LCD_ClearSeg(52);
		LCD_ClearSeg(8);
		LCD_ClearSeg(16);
		LCD_DisplayDigit(20,2);
		LCD_DisplayDigit(20,1);		//清除多余数字
		MeasureVpp_Init();
		P11Sample_Init();
		_enable_interrupts();
	}

	//----判断I2C_IO11所连的KEY2按键是否被按下------
	if((TCA6416A_InputBuffer&BIT9)== BIT9)
		KEY_Now |=BIT1;
	else
		KEY_Now &=~BIT1;
	if(((KEY_Past&BIT1)==BIT1)&&(KEY_Now&BIT1)!=BIT1){
		_NOP();
	}
		//I2C_IO11_Onclick();
	//----判断I2C_IO12所连的KEY3按键是否被按下------
	if((TCA6416A_InputBuffer&BITA) == BITA)
		KEY_Now |=BIT2;
	else
		KEY_Now &=~BIT2;
	if(((KEY_Past&BIT2)==BIT2)&&(KEY_Now&BIT2) ==0)
	{
		//KEY3被按下，切换测频模式
		_disable_interrupts();		//关闭中断
		unsigned char i=0;
		for(i=0;i<8;i++){
			PinOUT(i,1);
		}
		PinOUT(7,0);		//点亮LED8
		SampleFlag=3;
		LCD_ClearSeg(73);		//清除mV
		LCD_ClearSeg(77);
		LCD_ClearSeg(123);
		LCD_ClearSeg(107);
		for(i=7;i<11;i++){
			LCD_DisplayDigit(20,i);		//清除小LCD上面的数字
		}

		MeasureF_Init();
		ADC10CTL0 &= ~ENC;
		_enable_interrupts();

	}
	//----判断I2C_IO13所连的KEY4按键是否被按下------
	if((TCA6416A_InputBuffer&BITB) ==  BITB)
		KEY_Now |=BIT3;
	else
		KEY_Now &=~BIT3;
	if(((KEY_Past&BIT3) == BIT3)&& (KEY_Now&BIT3) == 0)    //
	{
		//I2C_IO13_Onclick();
		_NOP();
	}
}

void Display_dc(long Volt)
{
	//输入参数范围：1000-16000
	if(Volt<1000){			//for example:678,show 0.06
		LCD_DisplaySeg(123);		//显示小数点
		LCD_ClearSeg(107);			//清除小数点
		unsigned char tmp=0;
		tmp=Volt/100;				//百位
		LCD_DisplayDigit(tmp,9);
		tmp=(Volt%100)/10;			//十位
		LCD_DisplayDigit(tmp,8);
		tmp=Volt%10;				//个位
		LCD_DisplayDigit(tmp,7);
	}
	else if(Volt<10000){		//for example:1234,show 12.3mV
		LCD_DisplaySeg(107);		//显示小数点
		LCD_ClearSeg(123);			//清除小数点
		unsigned char tmp=0;
		tmp=Volt/1000;			//百位
		LCD_DisplayDigit(tmp,9);
		tmp=(Volt%1000)/100;	//十位
		LCD_DisplayDigit(tmp,8);
		tmp=(Volt%100)/10;		//个位
		LCD_DisplayDigit(tmp,7);
	}
	else{			//for example:10000
		LCD_ClearSeg(107);			//清除小数点
		LCD_ClearSeg(123);			//清除小数点
		unsigned char tmp=0;
		tmp=Volt/10000;
		LCD_DisplayDigit(tmp,9);
		tmp=(Volt%10000)/1000;
		LCD_DisplayDigit(tmp,8);
		tmp=(Volt%1000)/100;
		LCD_DisplayDigit(tmp,7);
	}
	HT1621_Reflash(LCD_Buffer);	//更新LCD显示

}
void Display_Vpp(unsigned int Volt)
{
	unsigned char tmp=0;
	tmp=Volt%10;	//个位
	LCD_DisplayDigit(tmp,6);
	if(Volt>=10){
		tmp=(Volt%100)/10;	//十位
		LCD_DisplayDigit(tmp,5);
		if(Volt>=100){
			tmp=(Volt%1000)/100;	//百位
			LCD_DisplayDigit(tmp,4);
			if(Volt>=1000){
				tmp=Volt/1000;	//千位
				LCD_DisplayDigit(tmp,3);
			}
			else{	//小于1000mV，清除千位
				LCD_DisplayDigit(50,3);
			}
		}
		else{	//小于100mV，清除百位，千位
			LCD_DisplayDigit(50,4);
			LCD_DisplayDigit(50,3);
		}
	}
	else{		//小于10mV，清除十位，百位，千位
		LCD_DisplayDigit(50,5);
		LCD_DisplayDigit(50,4);
		LCD_DisplayDigit(50,3);
	}
	HT1621_Reflash(LCD_Buffer);	//更新LCD显示	这个函数有入口参数，LCD_Buffer是显存
}

void MeasureF_Init()
{
	//TA1设定为1000ms中断
    TA1CCTL0 = CM_0 | CCIS_0 | OUTMOD_0 | CCIE;
    //TA1CCR0 = 32767;			//1s中断
    TA1CCR0=16383;			//500ms中断
    TA1CTL = TASSEL_1 | ID_0 | MC_1;

	//TA0设定为P1.0输入
    TA0CTL = TASSEL_0 | ID_0 | MC_2;
    P1SEL = BIT0;

    SampleFlag=3;
    TA0CTL |= TACLR	;

    LCD_DisplaySeg(43);	//显示KHZ
    LCD_DisplaySeg(47);
    HT1621_Reflash(LCD_Buffer);
}

void MeasureVpp_Init()
{
	//5ms定时中断
    TA1CCTL0 = CM_0 | CCIS_0 | OUTMOD_0 | CCIE;
    TA1CCR0 = 163;
    TA1CTL = TASSEL_1 | ID_0 | MC_1;

	LCD_DisplaySeg(73);
	LCD_DisplaySeg(77);			//点亮mV液晶块
	HT1621_Reflash(LCD_Buffer);
}


void Display_Vdc(unsigned int Volt)
{
	//在小号LCD上面显示三位有效数字直流偏置
	unsigned char tmp=0;
	tmp=Volt%10;	//个位
	LCD_DisplayDigit(tmp,7);
	if(Volt>=10){
		tmp=(Volt%100)/10;	//十位
		LCD_DisplayDigit(tmp,8);
		if(Volt>=100){
			tmp=(Volt%1000)/100;	//百位
			LCD_DisplayDigit(tmp,9);
			if(Volt>=1000){
				tmp=Volt/1000;	//千位
				LCD_DisplayDigit(tmp,10);
			}
			else{	//小于1000mV，清除千位
				LCD_DisplayDigit(50,10);
			}
		}
		else{	//小于100mV，清除百位，千位
			LCD_DisplayDigit(50,10);
			LCD_DisplayDigit(50,9);
		}
	}
	else{		//小于10mV，清除十位，百位，千位
		LCD_DisplayDigit(50,10);
		LCD_DisplayDigit(50,9);
		LCD_DisplayDigit(50,8);
	}
	HT1621_Reflash(LCD_Buffer);	//更新LCD显示	这个函数有入口参数，LCD_Buffer是显存
}

void Dsiplay_Freq(unsigned int freq)
{
	//大号LCD上面始终以四位有效数字显示频率
	if(freq<10){
		//for example:freq=9,show 0.00900KHz
		LCD_DisplayDigit(freq,4);
		LCD_DisplayDigit(0,5);
		LCD_DisplayDigit(0,6);
		LCD_DisplayDigit(0,3);
		LCD_DisplayDigit(0,2);
		LCD_DisplayDigit(0,1);
		LCD_DisplaySeg(8);			//显示小数点
		LCD_ClearSeg(52);
		LCD_ClearSeg(60);
		LCD_ClearSeg(16);
		LCD_ClearSeg(24);
	}
	else if(freq<100){
		//for example:freq=19,show 0.01900Khz
		LCD_DisplayDigit(freq%10,4);		//个位
		LCD_DisplayDigit(freq/10,3);		//十位
		LCD_DisplayDigit(0,2);
		LCD_DisplayDigit(0,1);
		//LCD_DisplayDigit(freq/10,3);		//十位
		LCD_DisplayDigit(0,5);
		LCD_DisplayDigit(0,6);
		LCD_DisplaySeg(8);			//显示小数点
		LCD_ClearSeg(52);
		LCD_ClearSeg(60);
		LCD_ClearSeg(16);
		LCD_ClearSeg(24);

	}
	else if(freq<1000){
		//for example:freq=123,show 0.1230KHz
		LCD_DisplayDigit(freq%10,5);
		LCD_DisplayDigit((freq%100)/10,4);
		LCD_DisplayDigit(freq/100,3);
		LCD_DisplayDigit(0,2);
		LCD_DisplayDigit(0,6);
		LCD_DisplayDigit(10,1);
		LCD_DisplaySeg(16);			//显示小数点
		LCD_ClearSeg(52);
		LCD_ClearSeg(8);
		LCD_ClearSeg(24);
		LCD_ClearSeg(60);
	}
	else if(freq<10000){
		//for example :freq=1234,show 1.234Khz
		LCD_DisplayDigit(freq%10,6);		//个位
		LCD_DisplayDigit((freq%100)/10,5);
		LCD_DisplayDigit((freq%1000)/100,4);
		LCD_DisplayDigit(freq/1000,3);
		LCD_DisplayDigit(20,2);
		LCD_DisplayDigit(20,1);
		LCD_DisplaySeg(24);			//显示小数点
		LCD_ClearSeg(52);
		LCD_ClearSeg(60);
		LCD_ClearSeg(8);
		LCD_ClearSeg(16);
	}
	else{
		//for example:freq=12345,show 12.34Khz
		LCD_DisplayDigit((freq%100)/10,6);
		LCD_DisplayDigit((freq%1000)/100,5);
		LCD_DisplayDigit((freq%10000)/1000,4);
		LCD_DisplayDigit(freq/10000,3);
		LCD_DisplayDigit(20,2);
		LCD_DisplayDigit(20,1);
		LCD_DisplaySeg(52);			//显示小数点
		LCD_ClearSeg(24);
		LCD_ClearSeg(60);
		LCD_ClearSeg(8);
		LCD_ClearSeg(16);
	}
	HT1621_Reflash(LCD_Buffer);	//更新LCD显示
}

void Display_RealV(long Volt)
{
	//自动转换量程显示三位有效数字，显示电压峰峰值
	//输入参数范围：1000-16000
	if(Volt<1000){			//for example:678，show6.79mV
		LCD_DisplaySeg(52);		//显示小数点
		LCD_ClearSeg(60);			//清除小数点
		unsigned char tmp=0;
		tmp=Volt/100;				//百位
		LCD_DisplayDigit(tmp,4);
		tmp=(Volt%100)/10;			//十位
		LCD_DisplayDigit(tmp,5);
		tmp=Volt%10;				//个位
		LCD_DisplayDigit(tmp,6);
	}
	else if(Volt<10000){		//for example:1234,show 12.34mV
		LCD_DisplaySeg(60);		//显示小数点
		LCD_ClearSeg(52);			//清除小数点
		unsigned char tmp=0;
		tmp=Volt/1000;			//百位
		LCD_DisplayDigit(tmp,4);
		tmp=(Volt%1000)/100;	//十位
		LCD_DisplayDigit(tmp,5);
		tmp=(Volt%100)/10;		//个位
		LCD_DisplayDigit(tmp,6);
	}
	else{			//for example:10000			show 100mV
		LCD_ClearSeg(52);			//清除小数点
		LCD_ClearSeg(60);			//清除小数点
		unsigned char tmp=0;
		tmp=Volt/10000;
		LCD_DisplayDigit(tmp,4);
		tmp=(Volt%10000)/1000;
		LCD_DisplayDigit(tmp,5);
		tmp=(Volt%1000)/100;
		LCD_DisplayDigit(tmp,6);
	}
	HT1621_Reflash(LCD_Buffer);	//更新LCD显示
}

void P11Sample_Init()
{
    ADC10CTL0 &= ~ENC;
    ADC10CTL0 = ADC10IE|ADC10ON | REFON | REF2_5V | ADC10SHT_2 | SREF_1;
    ADC10CTL1 = CONSEQ_0 | ADC10SSEL_0 | ADC10DIV_0 | SHS_0 | INCH_1;
    ADC10AE0 = 0x2;
    __delay_cycles(30000);
    ADC10CTL0 |= ENC;
    ADC10CTL0 |= ADC10SC;
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR_HOOK(void)
{
	//5ms定时中断
	static unsigned char TAcount=0;
	if(SampleFlag==1){
		VppADCsum+=Vpptmp[0]-Vpptmp[1];
		Vpptmp[0]=Vpptmp[1]=DCOFFSET;
		TAcount++;
		if(TAcount==6){
			//采样了六次峰峰值
			Vppave=VppADCsum/6;
			VppADCsum=0;
			TAcount=0;
			updateFlag=1;
		}
	}
	else if(SampleFlag==2){
		VdcADCsum+=Vpptmp[0]+Vpptmp[1];
		sampletime++;
		if(sampletime==6){
			Vdcave=VdcADCsum/6;
			sampletime=0;
			VdcADCsum=0;			//归零，开启新的一轮检测
			Vpptmp[0]=0;Vpptmp[1]=0x3ff;
			updateFlag=1;
		}
	}
	else if(SampleFlag==3){
		//测频,1s定时中断
		Freq=TAR*2;
		TA0CTL |= TACLR	;
	}
}

#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR_HOOK(void)
{
	if(SampleFlag==2){
		if(ADC10MEM>Vpptmp[0]){
			Vpptmp[0]=ADC10MEM;		//更新最大值
		}
		else if(ADC10MEM<Vpptmp[1]){
			Vpptmp[1]=ADC10MEM;		//更新最小值
		}
	}
	else if(SampleFlag==1){
		if(ADC10MEM>Vpptmp[0]){
			Vpptmp[0]=ADC10MEM;		//更新最大值
		}
		else if(ADC10MEM<Vpptmp[1]){
			Vpptmp[1]=ADC10MEM;		//更新最小值
		}
	}
	ADC10CTL0 |= ADC10SC;
}
