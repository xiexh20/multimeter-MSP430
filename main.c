#include <msp430.h> 
#include "HT1621.h"
#include "I2C.h"
#include "LCD_128.h"
#include "TCA6416A.h"
#define DCOFFSET  512
/*
 * ������������ֵ��ֱ��ƫ��
 */

unsigned char SampleFlag=0;		//SampleFlag=1: P1.1��������ֵ,SampleFlag=2:P1.2������ֱ��ƫ��,SampleFlag=3:��Ƶģʽ
unsigned int Vpptmp[2]={DCOFFSET,DCOFFSET};
long Vppsum=0;					//P11�ӷ��ֵ����
long Vdcsum=0;					//P12��ֱ��ƫ������
long Vpp=0;			//Vpp����ֵ
long Vdc=0;			//ֱ��ƫ�ò���ֵ
unsigned char updateFlag=0;
unsigned char sampletime=0;
unsigned int VppADCsum=0;		//���ֵ������������
unsigned int VdcADCsum=0;		//ֱ��ƫ�ò�����������
unsigned int Vppave=0,Vdcave=0;
unsigned int Freq=0;			//Ƶ��

void P11Sample_Init();			//P1.1�ڲ�����ʼ��
//void P12Sample_Init();			//P1.2�ڲ�����ʼ��
void MeasureVpp_Init();
void MeasureF_Init();
void Display_Vpp(unsigned int Volt);		//�ڴ�LCD����λ��Ч������ʾ
void Display_Vdc(unsigned int Volt);		//��СLCD����ʾ��ѹ
void Dsiplay_Freq(unsigned int freq);		//���LCD������λ������ʾƵ��
void I2C_IODect();		//���I/O�ĺ���
void Display_RealV(long Volt);
void Display_dc(long Volt);
long ComputeAC(long Volt);
long ComputeDC(long Volt);

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	BCSCTL1 = CALBC1_16MHZ;      /* Set DCO to 1MHz */
    DCOCTL = CALDCO_16MHZ;
    TCA6416A_Init();			//��ʼ��I2C��TCA6426�Ĵ���
    HT1621_init();				// ��ʼ��lcd_128�����ǧ����©������
    LCD_Clear();				//�����Ļ
    HT1621_Reflash(LCD_Buffer);	//�ǵø���RAM����֮������Դ�

//	P11Sample_Init();
	SampleFlag=0;
	unsigned char Vppcount=0,Vdccount=0;
	
//	ADC10CTL0 |= ADC10SC;
//	MeasureF_Init();
	__bis_SR_register(GIE);

	while(1){
		PinIN();
		I2C_IODect();
		if(SampleFlag==1){		//��������ֵ
			if(updateFlag==1){
				Vppsum+=Vppave;
				Vppcount++;
				if(Vppcount==10){
					Vppsum=Vppsum/10;
					Vpp=(unsigned int)(2500*Vppsum/1024);
					Vppsum=0;
					Vppcount=0;		//����
					//Vpptmp[0]=Vpptmp[1]=0;
					sampletime=0;

					//������ֵ�л�����ƫ�õ�ѹ
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
					Vdcsum=Vdcsum/10;		//���ȡƽ��
					Vdc=(unsigned int)(2500*Vdcsum/1024);
					Vdc=Vdc/2;
					Vdcsum=0;
					Vdccount=0;
					//Vpptmp[0]=Vpptmp[1]=0;
					sampletime=0;

					//����ֱ��ƫ�ñ�ɲ�������
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
			updateFlag=0;		//�ȴ��´�ȡֵ���
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

void I2C_IODect()			                 //����¼�ȷʵ������
{
	static unsigned char KEY_Past=0,KEY_Now=0;
	KEY_Past=KEY_Now;
	//----�ж�I2C_IO10������KEY1�����Ƿ񱻰���------
	if((TCA6416A_InputBuffer&BIT8) == BIT8)
		KEY_Now |=BIT0;
	else
		KEY_Now &=~BIT0;
	if(((KEY_Past&BIT0)==BIT0)&&(KEY_Now&BIT0) !=BIT0){
		//KEY1�����£��л��ɲ���VPPģʽ
		_disable_interrupts();		//�ر��ж�
		unsigned char i=0;
		for(i=0;i<8;i++){
			PinOUT(i,1);
		}
		PinOUT(0,0);		//����LED1
		SampleFlag=1;
	    LCD_ClearSeg(43);	//���KHZ
	    LCD_ClearSeg(47);
		LCD_DisplayDigit(20,3);
		LCD_ClearSeg(24);			//���С����
		LCD_ClearSeg(52);
		LCD_ClearSeg(8);
		LCD_ClearSeg(16);
		LCD_DisplayDigit(20,2);
		LCD_DisplayDigit(20,1);		//�����������
		MeasureVpp_Init();
		P11Sample_Init();
		_enable_interrupts();
	}

	//----�ж�I2C_IO11������KEY2�����Ƿ񱻰���------
	if((TCA6416A_InputBuffer&BIT9)== BIT9)
		KEY_Now |=BIT1;
	else
		KEY_Now &=~BIT1;
	if(((KEY_Past&BIT1)==BIT1)&&(KEY_Now&BIT1)!=BIT1){
		_NOP();
	}
		//I2C_IO11_Onclick();
	//----�ж�I2C_IO12������KEY3�����Ƿ񱻰���------
	if((TCA6416A_InputBuffer&BITA) == BITA)
		KEY_Now |=BIT2;
	else
		KEY_Now &=~BIT2;
	if(((KEY_Past&BIT2)==BIT2)&&(KEY_Now&BIT2) ==0)
	{
		//KEY3�����£��л���Ƶģʽ
		_disable_interrupts();		//�ر��ж�
		unsigned char i=0;
		for(i=0;i<8;i++){
			PinOUT(i,1);
		}
		PinOUT(7,0);		//����LED8
		SampleFlag=3;
		LCD_ClearSeg(73);		//���mV
		LCD_ClearSeg(77);
		LCD_ClearSeg(123);
		LCD_ClearSeg(107);
		for(i=7;i<11;i++){
			LCD_DisplayDigit(20,i);		//���СLCD���������
		}

		MeasureF_Init();
		ADC10CTL0 &= ~ENC;
		_enable_interrupts();

	}
	//----�ж�I2C_IO13������KEY4�����Ƿ񱻰���------
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
	//���������Χ��1000-16000
	if(Volt<1000){			//for example:678,show 0.06
		LCD_DisplaySeg(123);		//��ʾС����
		LCD_ClearSeg(107);			//���С����
		unsigned char tmp=0;
		tmp=Volt/100;				//��λ
		LCD_DisplayDigit(tmp,9);
		tmp=(Volt%100)/10;			//ʮλ
		LCD_DisplayDigit(tmp,8);
		tmp=Volt%10;				//��λ
		LCD_DisplayDigit(tmp,7);
	}
	else if(Volt<10000){		//for example:1234,show 12.3mV
		LCD_DisplaySeg(107);		//��ʾС����
		LCD_ClearSeg(123);			//���С����
		unsigned char tmp=0;
		tmp=Volt/1000;			//��λ
		LCD_DisplayDigit(tmp,9);
		tmp=(Volt%1000)/100;	//ʮλ
		LCD_DisplayDigit(tmp,8);
		tmp=(Volt%100)/10;		//��λ
		LCD_DisplayDigit(tmp,7);
	}
	else{			//for example:10000
		LCD_ClearSeg(107);			//���С����
		LCD_ClearSeg(123);			//���С����
		unsigned char tmp=0;
		tmp=Volt/10000;
		LCD_DisplayDigit(tmp,9);
		tmp=(Volt%10000)/1000;
		LCD_DisplayDigit(tmp,8);
		tmp=(Volt%1000)/100;
		LCD_DisplayDigit(tmp,7);
	}
	HT1621_Reflash(LCD_Buffer);	//����LCD��ʾ

}
void Display_Vpp(unsigned int Volt)
{
	unsigned char tmp=0;
	tmp=Volt%10;	//��λ
	LCD_DisplayDigit(tmp,6);
	if(Volt>=10){
		tmp=(Volt%100)/10;	//ʮλ
		LCD_DisplayDigit(tmp,5);
		if(Volt>=100){
			tmp=(Volt%1000)/100;	//��λ
			LCD_DisplayDigit(tmp,4);
			if(Volt>=1000){
				tmp=Volt/1000;	//ǧλ
				LCD_DisplayDigit(tmp,3);
			}
			else{	//С��1000mV�����ǧλ
				LCD_DisplayDigit(50,3);
			}
		}
		else{	//С��100mV�������λ��ǧλ
			LCD_DisplayDigit(50,4);
			LCD_DisplayDigit(50,3);
		}
	}
	else{		//С��10mV�����ʮλ����λ��ǧλ
		LCD_DisplayDigit(50,5);
		LCD_DisplayDigit(50,4);
		LCD_DisplayDigit(50,3);
	}
	HT1621_Reflash(LCD_Buffer);	//����LCD��ʾ	�����������ڲ�����LCD_Buffer���Դ�
}

void MeasureF_Init()
{
	//TA1�趨Ϊ1000ms�ж�
    TA1CCTL0 = CM_0 | CCIS_0 | OUTMOD_0 | CCIE;
    //TA1CCR0 = 32767;			//1s�ж�
    TA1CCR0=16383;			//500ms�ж�
    TA1CTL = TASSEL_1 | ID_0 | MC_1;

	//TA0�趨ΪP1.0����
    TA0CTL = TASSEL_0 | ID_0 | MC_2;
    P1SEL = BIT0;

    SampleFlag=3;
    TA0CTL |= TACLR	;

    LCD_DisplaySeg(43);	//��ʾKHZ
    LCD_DisplaySeg(47);
    HT1621_Reflash(LCD_Buffer);
}

void MeasureVpp_Init()
{
	//5ms��ʱ�ж�
    TA1CCTL0 = CM_0 | CCIS_0 | OUTMOD_0 | CCIE;
    TA1CCR0 = 163;
    TA1CTL = TASSEL_1 | ID_0 | MC_1;

	LCD_DisplaySeg(73);
	LCD_DisplaySeg(77);			//����mVҺ����
	HT1621_Reflash(LCD_Buffer);
}


void Display_Vdc(unsigned int Volt)
{
	//��С��LCD������ʾ��λ��Ч����ֱ��ƫ��
	unsigned char tmp=0;
	tmp=Volt%10;	//��λ
	LCD_DisplayDigit(tmp,7);
	if(Volt>=10){
		tmp=(Volt%100)/10;	//ʮλ
		LCD_DisplayDigit(tmp,8);
		if(Volt>=100){
			tmp=(Volt%1000)/100;	//��λ
			LCD_DisplayDigit(tmp,9);
			if(Volt>=1000){
				tmp=Volt/1000;	//ǧλ
				LCD_DisplayDigit(tmp,10);
			}
			else{	//С��1000mV�����ǧλ
				LCD_DisplayDigit(50,10);
			}
		}
		else{	//С��100mV�������λ��ǧλ
			LCD_DisplayDigit(50,10);
			LCD_DisplayDigit(50,9);
		}
	}
	else{		//С��10mV�����ʮλ����λ��ǧλ
		LCD_DisplayDigit(50,10);
		LCD_DisplayDigit(50,9);
		LCD_DisplayDigit(50,8);
	}
	HT1621_Reflash(LCD_Buffer);	//����LCD��ʾ	�����������ڲ�����LCD_Buffer���Դ�
}

void Dsiplay_Freq(unsigned int freq)
{
	//���LCD����ʼ������λ��Ч������ʾƵ��
	if(freq<10){
		//for example:freq=9,show 0.00900KHz
		LCD_DisplayDigit(freq,4);
		LCD_DisplayDigit(0,5);
		LCD_DisplayDigit(0,6);
		LCD_DisplayDigit(0,3);
		LCD_DisplayDigit(0,2);
		LCD_DisplayDigit(0,1);
		LCD_DisplaySeg(8);			//��ʾС����
		LCD_ClearSeg(52);
		LCD_ClearSeg(60);
		LCD_ClearSeg(16);
		LCD_ClearSeg(24);
	}
	else if(freq<100){
		//for example:freq=19,show 0.01900Khz
		LCD_DisplayDigit(freq%10,4);		//��λ
		LCD_DisplayDigit(freq/10,3);		//ʮλ
		LCD_DisplayDigit(0,2);
		LCD_DisplayDigit(0,1);
		//LCD_DisplayDigit(freq/10,3);		//ʮλ
		LCD_DisplayDigit(0,5);
		LCD_DisplayDigit(0,6);
		LCD_DisplaySeg(8);			//��ʾС����
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
		LCD_DisplaySeg(16);			//��ʾС����
		LCD_ClearSeg(52);
		LCD_ClearSeg(8);
		LCD_ClearSeg(24);
		LCD_ClearSeg(60);
	}
	else if(freq<10000){
		//for example :freq=1234,show 1.234Khz
		LCD_DisplayDigit(freq%10,6);		//��λ
		LCD_DisplayDigit((freq%100)/10,5);
		LCD_DisplayDigit((freq%1000)/100,4);
		LCD_DisplayDigit(freq/1000,3);
		LCD_DisplayDigit(20,2);
		LCD_DisplayDigit(20,1);
		LCD_DisplaySeg(24);			//��ʾС����
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
		LCD_DisplaySeg(52);			//��ʾС����
		LCD_ClearSeg(24);
		LCD_ClearSeg(60);
		LCD_ClearSeg(8);
		LCD_ClearSeg(16);
	}
	HT1621_Reflash(LCD_Buffer);	//����LCD��ʾ
}

void Display_RealV(long Volt)
{
	//�Զ�ת��������ʾ��λ��Ч���֣���ʾ��ѹ���ֵ
	//���������Χ��1000-16000
	if(Volt<1000){			//for example:678��show6.79mV
		LCD_DisplaySeg(52);		//��ʾС����
		LCD_ClearSeg(60);			//���С����
		unsigned char tmp=0;
		tmp=Volt/100;				//��λ
		LCD_DisplayDigit(tmp,4);
		tmp=(Volt%100)/10;			//ʮλ
		LCD_DisplayDigit(tmp,5);
		tmp=Volt%10;				//��λ
		LCD_DisplayDigit(tmp,6);
	}
	else if(Volt<10000){		//for example:1234,show 12.34mV
		LCD_DisplaySeg(60);		//��ʾС����
		LCD_ClearSeg(52);			//���С����
		unsigned char tmp=0;
		tmp=Volt/1000;			//��λ
		LCD_DisplayDigit(tmp,4);
		tmp=(Volt%1000)/100;	//ʮλ
		LCD_DisplayDigit(tmp,5);
		tmp=(Volt%100)/10;		//��λ
		LCD_DisplayDigit(tmp,6);
	}
	else{			//for example:10000			show 100mV
		LCD_ClearSeg(52);			//���С����
		LCD_ClearSeg(60);			//���С����
		unsigned char tmp=0;
		tmp=Volt/10000;
		LCD_DisplayDigit(tmp,4);
		tmp=(Volt%10000)/1000;
		LCD_DisplayDigit(tmp,5);
		tmp=(Volt%1000)/100;
		LCD_DisplayDigit(tmp,6);
	}
	HT1621_Reflash(LCD_Buffer);	//����LCD��ʾ
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
	//5ms��ʱ�ж�
	static unsigned char TAcount=0;
	if(SampleFlag==1){
		VppADCsum+=Vpptmp[0]-Vpptmp[1];
		Vpptmp[0]=Vpptmp[1]=DCOFFSET;
		TAcount++;
		if(TAcount==6){
			//���������η��ֵ
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
			VdcADCsum=0;			//���㣬�����µ�һ�ּ��
			Vpptmp[0]=0;Vpptmp[1]=0x3ff;
			updateFlag=1;
		}
	}
	else if(SampleFlag==3){
		//��Ƶ,1s��ʱ�ж�
		Freq=TAR*2;
		TA0CTL |= TACLR	;
	}
}

#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR_HOOK(void)
{
	if(SampleFlag==2){
		if(ADC10MEM>Vpptmp[0]){
			Vpptmp[0]=ADC10MEM;		//�������ֵ
		}
		else if(ADC10MEM<Vpptmp[1]){
			Vpptmp[1]=ADC10MEM;		//������Сֵ
		}
	}
	else if(SampleFlag==1){
		if(ADC10MEM>Vpptmp[0]){
			Vpptmp[0]=ADC10MEM;		//�������ֵ
		}
		else if(ADC10MEM<Vpptmp[1]){
			Vpptmp[1]=ADC10MEM;		//������Сֵ
		}
	}
	ADC10CTL0 |= ADC10SC;
}
