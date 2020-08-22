#include "c8051f020.h" 
#include <intrins.h>

sbit	HD7279_DAT=P1^7;
sbit	HD7279_CLK=P1^6;

#define NOSELECT7279  	P5 |= 0x80		//SPICS4(P57)=1
#define SELECT7279  	P5 &= ~(0x80)  	//SPICS4(P57)=0;
#define Set7279DAT  	HD7279_DAT=1
#define Clr7279DAT  	HD7279_DAT=0
#define Set7279CLK  	HD7279_CLK=1
#define Clr7279CLK  	HD7279_CLK=0
void Delay1ms(unsigned char T);
void Delay1s(unsigned char T);
void Delay1us(unsigned char T);
void Send7279Byte(unsigned char ch)//发送
{	
	char i;
	SELECT7279;     	//置CS低电平 
	Delay1us(50);		//延时50μ
	
	for (i=0;i<8;i++)
	{	
		if (ch&0x80)	//输出7位到HD7279A的DATA端 
		{
			Set7279DAT;
		}
		else
		{
			Clr7279DAT;
		}
		Set7279CLK;		//置CLK高电平 
		ch=ch<<1;		//待发数据左移 
		Delay1us(8);	//延时8μ
		Clr7279CLK;		//置CLK低电平 
		Delay1us(8);	//延时50μ
	}
	Clr7279DAT;			//发送完毕，DATA端置低，返回 
}
unsigned char Receive7279Byte(void)//接收
{
	unsigned char i,ch;
	ch=0;		
	Set7279DAT;			//DATA端置为高电平，输入状态
	Delay1us(50);		//延时50μ
	for (i=0;i<8;i++)
	{
		Set7279CLK;		//置CLK高电平
		Delay1us(8);	//延时8μ
		ch=ch<<1;		//接收数据左移1位
		if (HD7279_DAT)
			ch+=1;		//接收1位数据
		Clr7279CLK;		//置CLK低电平
		Delay1us(8);	//延时8μ
	}
	Clr7279DAT;			//接收完毕，DATA端重新置成低电平(输出状态)
	return ch;
}



unsigned char code BdSeg[]={
					0x7e,0x30,0x6d,0x79, // 0 1 2 3 
                   	0x33,0x5b,0x5f,0x70, // 4 5 6 7
                   	0x7f,0x7b,0x77,0x1f, // 8 9 a b
                   	0x4e,0x3d,0x4f,0x47, // c d e f
                   	0x00,0x01}; 
char GetKeyValue(void)//获取键值
{
	char KeyValue;
	if (CPT1CN&0x40) return -1;	//无键按下 
	Send7279Byte(0x15);	//发读键盘指令 
	KeyValue=Receive7279Byte();
	NOSELECT7279;     	//置CS高电平 
	return KeyValue; 
}

void WaitKeyOff(void)
{
	while  (!(CPT1CN&0x40));
}

