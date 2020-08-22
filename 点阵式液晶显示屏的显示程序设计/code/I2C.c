#include <c8051f020.h>					// SFR declarations
#include <intrins.h>
//------------------------------------------------------------------------------------
// Global CONSTANTS
//------------------------------------------------------------------------------------

#define 	WRITE	0x00						// SMBus WRITE command
#define		READ 	0x01						// SMBus READ command

// Device addresses (7 bits, lsb is a don't care)
#define		CLOCK3530_ADDRESS_RESET		0x60		//1 ack
#define		CLOCK3530_ADDRESS_STATUS	0x62		//2 ack
#define		CLOCK3530_ADDRESS_DATEHOUR	0x64		//8 ack  year month day week hour minute second
#define		CLOCK3530_ADDRESS_HOUR		0x66		//4 ack  hour minute second
#define		CLOCK3530_ADDRESS_INT1		0x68		//3 ack  
#define		CLOCK3530_ADDRESS_INT2		0x6A		//3 ack  

union 
{
unsigned char ClockString[7];
struct RealClock
	{
		unsigned char Year,Month,Day,Week,Hour,Minute,Second;
	} RT;
} RealTime;


// SMBus states:
// MT = Master Transmitter
// MR = Master Receiver
#define	SMB_BUS_ERROR	0x00			// (all modes) BUS ERROR
#define	SMB_START		0x08			// (MT & MR) START transmitted
#define	SMB_RP_START	0x10			// (MT & MR) repeated START
#define	SMB_MTADDACK	0x18			// (MT) Slave address + W transmitted;
										//  ACK received
#define	SMB_MTADDNACK	0x20			// (MT) Slave address + W transmitted;
										//  NACK received
#define	SMB_MTDBACK		0x28			// (MT) data byte transmitted; ACK rec'vd
#define	SMB_MTDBNACK	0x30			// (MT) data byte transmitted; NACK rec'vd
#define	SMB_MTARBLOST	0x38			// (MT) arbitration lost
#define	SMB_MRADDACK	0x40			// (MR) Slave address + R transmitted;
										//  ACK received
#define	SMB_MRADDNACK	0x48			// (MR) Slave address + R transmitted;
										//  NACK received
#define	SMB_MRDBACK		0x50			// (MR) data byte rec'vd; ACK transmitted
#define	SMB_MRDBNACK	0x58			// (MR) data byte rec'vd; NACK transmitted


//-----------------------------------------------------------------------------------
//Global VARIABLES
//-----------------------------------------------------------------------------------
char COMMAND;			// Holds the slave address + R/W bit for use in the SMBus ISR.

unsigned char *I2CDataBuff;						

char BYTE_NUMBER;							// Used by ISR to check what data has just been
												// sent - High address byte, Low byte, or data byte

unsigned char HIGH_ADD, LOW_ADD;		// High & Low byte for EEPROM memory address
				
bit SM_BUSY;								// This bit is set when a send or receive
												// is started. It is cleared by the
												// ISR when the operation is finished.


//------------------------------------------------------------------------------------
// Function PROTOTYPES
//------------------------------------------------------------------------------------

void SMBus_ISR (void);								

//------------------------------------------------------------------------------------
// MAIN Routine
//------------------------------------------------------------------------------------
//
// Main routine configures the crossbar and SMBus, and tests
// the SMBus interface between the three EEPROMs


void ResetRealClock(void)
{
	while (SM_BUSY);									// Wait for SMBus to be free.
	SM_BUSY = 1;										// Occupy SMBus (set to busy)
	SMB0CN = 0x44;										// SMBus enabled, ACK on acknowledge cycle
	BYTE_NUMBER = 0;									// 2 address bytes.
	COMMAND = (CLOCK3530_ADDRESS_RESET | READ);		// Chip select + READ
	STA = 1;											// Start transfer
	while (SM_BUSY);									// Wait for transfer to finish
}

//======================дS-3530A�ڲ�ʵʱ���ݼĴ�������=====================
//���ܣ����趨�ꡢ�¡��ա����ڡ�ʱ���֡�������д��S-3530A                  |
//��ڣ��������ݷ����ꡢ�¡��ա����ڡ�ʱ���֡�����Ĵ���                   |
//���ڣ�NONE                                                               |
//==========================================================================
void SetRealClock(void)
{
	while (SM_BUSY);									// Wait for SMBus to be free.
	SM_BUSY = 1;										// Occupy SMBus (set to busy)
	SMB0CN = 0x44;										// SMBus enabled, ACK on acknowledge cycle
	BYTE_NUMBER = 7;									// 2 address bytes.
	COMMAND = (CLOCK3530_ADDRESS_DATEHOUR | WRITE);		// Chip select + WRITE
	I2CDataBuff = &RealTime.ClockString[0];				// Data to be writen
	STA = 1;											// Start transfer
}

//==================��S-3530Aʵʱ���ݼĴ����ӳ���===========================
//���ܣ���S-3530A���뵱ǰʱ������                                          |
//��ڣ�NONE                                                               |
//���ڣ��������ݷ����ꡢ�¡��ա����ڡ�ʱ���֡�����Ĵ���                   |
//==========================================================================
void GetRealClock(void)
{
	while (SM_BUSY);									// Wait for SMBus to be free.
	SM_BUSY = 1;										// Occupy SMBus (set to busy)
	SMB0CN = 0x44;										// SMBus enabled, ACK on acknowledge cycle
	BYTE_NUMBER = 7;									// 2 address bytes.
	COMMAND = (CLOCK3530_ADDRESS_DATEHOUR | READ);		// Chip select + READ
	I2CDataBuff = &RealTime.ClockString[0];				// Data to be writen
	STA = 1;											// Start transfer
	while (SM_BUSY);									// Wait for transfer to finish
}

//============================д״̬�Ĵ�������==============================
//���ܣ���/дS-3530A״̬�Ĵ�������S-3530A��������                          |
//��ڣ�NONE           ���ڣ�NONE                                          |
//==========================================================================
unsigned char  GetRealClockStatus(void)
{
	unsigned char result; 
	while (SM_BUSY);									// Wait for SMBus to be free.
	SM_BUSY = 1;										// Occupy SMBus (set to busy)
	SMB0CN = 0x44;										// SMBus enabled, ACK on acknowledge cycle
	BYTE_NUMBER = 1;									
	COMMAND = (CLOCK3530_ADDRESS_STATUS | READ);		
	I2CDataBuff = &result;								
	STA = 1;											// Start transfer
	while (SM_BUSY);									// Wait for transfer to finish
	return result;
}
void SetRealClockStatus(unsigned char status)
{
	while (SM_BUSY);									// Wait for SMBus to be free.
	SM_BUSY = 1;										// Occupy SMBus (set to busy)
	SMB0CN = 0x44;										// SMBus enabled, ACK on acknowledge cycle
	BYTE_NUMBER = 1;									
	COMMAND = (CLOCK3530_ADDRESS_STATUS | WRITE);		
	I2CDataBuff = &status;								
	STA = 1;											// Start transfer
}
/*
void  SetRealClockINT1(unsigned int Int1)
{
	while (SM_BUSY);									// Wait for SMBus to be free.
	SM_BUSY = 1;										// Occupy SMBus (set to busy)
	SMB0CN = 0x44;										// SMBus enabled, ACK on acknowledge cycle
	BYTE_NUMBER = 2;									
	COMMAND = (CLOCK3530_ADDRESS_INT1 | WRITE);		
	I2CDataBuff = (unsigned char*)&Int1;								
	STA = 1;											// Start transfer
}
*/
#include "INTRINS.H"

unsigned char revolve(unsigned char val)
{
char i;
unsigned char val1=0;
	for (i=0;i<8;i++)
	{
		if (val&0x1)
			val1++;
		val1=_crol_(val1,1);
		val=_cror_(val,1);
	}
	val1=_cror_(val1,1);
	return val1;
}

/*--  ����:  ʱ  --*/

char code Shi[]=
{
	0x00,0x00,0xFC,0x44,0x44,0xFC,0x00,0x08,0x48,0x88,0x08,0xFF,0x08,0x08,0x08,0x00,
	0x00,0x00,0x1F,0x04,0x04,0x0F,0x00,0x00,0x00,0x11,0x20,0x1F,0x00,0x00,0x00,0x00,
};

/*--  ����:  ��  --*/
char code Zhong[]=
{
	0x00,0x60,0x38,0xE7,0x24,0x24,0x04,0x00,0xF8,0x88,0x88,0xFF,0x88,0x88,0xF8,0x00,
	0x00,0x01,0x01,0x3F,0x11,0x09,0x01,0x00,0x01,0x00,0x00,0x3F,0x00,0x00,0x01,0x00,
};

char code mao1[]=          //ð��
{
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};
void LCD_WriteHZ(char x,char y,char *Dot);
void LCD_DispChar(char x,char y,char ch); //128*64  ȡֵx=0-128 y=0-8
void 	InitLCD(void);
void Delay1ms(unsigned char T);

void TestI2C (void)
{
unsigned char var ;
char KeyValue;
	WDTCN = 0xde;									// disable watchdog timer
	WDTCN = 0xad;
	
	OSCICN |= 0x03;								// Set internal oscillator to highest setting
														// (16 MHz)

	XBR0 |= 0x07;									// Route SMBus to GPIO pins through crossbar
	XBR2 |= 0x44;									// Enable crossbar and weak pull-ups

    P0MDOUT |= 0x1D; 
    P1MDOUT |= 0x01; 
	
	SMB0CN = 0x44;									// Enable SMBus with ACKs on acknowledge cycle
	SMB0CR = -80;									// SMBus clock rate = 100kHz.

	EIE1 |= 2;										// SMBus interrupt enable
	EA = 1;											// Global interrupt enable

	SM_BUSY = 0;									// Free SMBus for first transfer.

//	SetRealClockINT1(0x8000);
	var = GetRealClockStatus();
	ResetRealClock();
	var = GetRealClockStatus();
	SetRealClockStatus(0xc2);
	var = GetRealClockStatus();
	GetRealClock();
	RealTime.RT.Year=0x15;	
	RealTime.RT.Month=0x7;	
	RealTime.RT.Day=0x06;	
	RealTime.RT.Week=0x05;	
	RealTime.RT.Hour=0x14;	
	RealTime.RT.Minute=0x40;	
	RealTime.RT.Second=0x50;	
	SetRealClock();
	GetRealClock();
	InitLCD();
	LCD_WriteHZ(0,0,Shi); 
	LCD_WriteHZ(16,0,Zhong); 
//��0��2����ʾ00:00:00
	LCD_DispChar(0,2,0); //128*64  ȡֵx=0-128 y=0-8
	LCD_DispChar(8,2,0);
	LCD_DispChar(16,2,10);
    LCD_WriteHZ(17,2,mao1);
	LCD_DispChar(24,2,0);
	LCD_DispChar(32,2,0);
	LCD_DispChar(40,2,0xa);
	LCD_WriteHZ(41,2,mao1);
	LCD_DispChar(48,2,0);
	LCD_DispChar(56,2,0);
//��0��4����ʾ02/01/01
	LCD_DispChar(0,4,0); //128*64  ȡֵx=0-128 y=0-8
	LCD_DispChar(8,4,2);
	LCD_DispChar(16,4,0xb);
	LCD_WriteHZ(17,4,mao1);
	LCD_DispChar(24,4,0);
	LCD_DispChar(32,4,1);
	LCD_DispChar(40,4,0xb);
	LCD_WriteHZ(41,4,mao1);
	LCD_DispChar(48,4,0);
	LCD_DispChar(56,4,1);
	for (;;)
	{     //GetRealClock();
	  
		KeyValue=GetKeyValue();
		   WaitKeyOff();
		   	
        if(KeyValue==1)
       RealTime.RT.Year++;
	   else if(KeyValue==5)
	   RealTime.RT.Year--;
	   else if(KeyValue==2)
       RealTime.RT.Month++;
        else if(KeyValue==6)
		RealTime.RT.Month--;
		else if(KeyValue==3)
       RealTime.RT.Day++;
        else if(KeyValue==7)
       RealTime.RT.Day--;
       else if(KeyValue==8)
       RealTime.RT.Hour++;
	   else if(KeyValue==12)
       RealTime.RT.Hour--;
         else if(KeyValue==9)
         RealTime.RT.Minute++;
         else if(KeyValue==13)
        RealTime.RT.Minute--;
       else if(KeyValue==10)
         RealTime.RT.Second++;
         else if(KeyValue==14)
         RealTime.RT.Second--;
     else if(KeyValue==0)break;
	 else if(KeyValue==15) {
	for (;;){ GetRealClock();
		LCD_DispChar(0,2,(RealTime.RT.Hour>>4)&0x03);//(RealTime.RT.Hour>>4)&0x0f); //128*64  ȡֵx=0-128 y=0-8
		LCD_DispChar(8,2,RealTime.RT.Hour&0x0f);
		LCD_DispChar(24,2,(RealTime.RT.Minute>>4)&0x0f);
		LCD_DispChar(32,2,RealTime.RT.Minute&0x0f);
		LCD_DispChar(48,2,(RealTime.RT.Second>>4)&0x0f);
		LCD_DispChar(56,2,RealTime.RT.Second&0x0f);
		//��0��4����ʾ02/01/01
		LCD_DispChar(0,4,(RealTime.RT.Year>>4)&0x0f); 
		LCD_DispChar(8,4,RealTime.RT.Year&0x0f);
		LCD_DispChar(24,4,(RealTime.RT.Month>>4)&0x0f);
		LCD_DispChar(32,4,RealTime.RT.Month&0x0f);
		LCD_DispChar(48,4,(RealTime.RT.Day>>4)&0x0f);
		LCD_DispChar(56,4,RealTime.RT.Day&0x0f);
		Delay1ms(100);
		KeyValue=GetKeyValue();
	
		if(KeyValue!=-1)
		break;}	}
		LCD_DispChar(0,2,(RealTime.RT.Hour>>4)&0x03);//(RealTime.RT.Hour>>4)&0x0f); //128*64  ȡֵx=0-128 y=0-8
		LCD_DispChar(8,2,RealTime.RT.Hour&0x0f);
		LCD_DispChar(24,2,(RealTime.RT.Minute>>4)&0x0f);
		LCD_DispChar(32,2,RealTime.RT.Minute&0x0f);
		LCD_DispChar(48,2,(RealTime.RT.Second>>4)&0x0f);
		LCD_DispChar(56,2,RealTime.RT.Second&0x0f);
		//��0��4����ʾ02/01/01
		LCD_DispChar(0,4,(RealTime.RT.Year>>4)&0x0f); 
		LCD_DispChar(8,4,RealTime.RT.Year&0x0f);
		LCD_DispChar(24,4,(RealTime.RT.Month>>4)&0x0f);
		LCD_DispChar(32,4,RealTime.RT.Month&0x0f);
		LCD_DispChar(48,4,(RealTime.RT.Day>>4)&0x0f);
		LCD_DispChar(56,4,RealTime.RT.Day&0x0f);
		SetRealClock();
		Delay1ms(100);
		
		//KeyValue=GetKeyValue();
	//KeyValue=GetKeyValue();
    //if(KeyValue!=-1)
       // break;
	}
    
}


//------------------------------------------------------------------------------------
// Interrupt Service Routine
//------------------------------------------------------------------------------------
void SMBUS_ISR (void) interrupt 7
{
	switch (SMB0STA)
		{ 	// SMBus ״̬��SMB0STA �Ĵ���
			// ��������/��������ʼ�����ѷ���
		case SMB_START:
			SMB0DAT = COMMAND ; // װ��Ҫ���ʵĴ������ĵ�ַ
			STA = 0; 			// �ֶ����START λ
			break;
			//��������/�������ظ���ʼ�����ѷ���
			// ��״ֻ̬Ӧ�ڶ������ڼ�����ڴ洢����ַ�ѷ��Ͳ��õ�ȷ��֮�� ?
		case SMB_RP_START:
			SMB0DAT = COMMAND; // COMMAND ��Ӧ���ִӵ�ַ + R.
			STA = 0;
			break;
			// ���������ӵ�ַ + WRITE �ѷ����յ�ACK
		case SMB_MTADDACK:
			// �������������ֽ��ѷ����յ�ACK
		case SMB_MTDBACK:
			if (BYTE_NUMBER)
			{
					SMB0DAT = revolve(*I2CDataBuff);					// If R/W=WRITE, load byte to write.
					I2CDataBuff++;
					BYTE_NUMBER--;			
			}
			else
			{
				STO = 1;	SM_BUSY = 0;						// Free SMBus
			}
			break;
			// ���������ӵ�ַ + WRITE �ѷ����յ�NACK
			// ��������Ӧ����STOP + START ����
		case SMB_MTADDNACK:
			STO = 1;			STA = 1;
		break;
			// �������������ֽ��ѷ����յ�NACK
			// ��������Ӧ����STOP + START ����
		case SMB_MTDBNACK:
			STO = 1;			STA = 1;
		break;
			// ������������ʧ��
			// ��Ӧ��������������¿�ʼ�������
		case SMB_MTARBLOST:
			STO = 1;			STA = 1;
		break;

			// ���������ӵ�ַ + READ �ѷ���,�յ�ACK
		case SMB_MRADDACK:
			AA = 1; // ��Ӧ������ACK
			if (!BYTE_NUMBER)
			{	
				STO = 1;	SM_BUSY = 0; // �ͷ�SMBus
			}
		break;
			// ���������ӵ�ַ + READ �ѷ����յ�NACK
			// ��������Ӧ�����ظ���ʼ��������
		case SMB_MRADDNACK:
			STA = 1;
		break;
			// �յ������ֽ�ACK �ѷ���
			// ��״̬��Ӧ������ΪAA ����ǰһ״̬����0 ������ַ���ֹͣ����
		case SMB_MRDBACK:
			if (BYTE_NUMBER)
			{
				*I2CDataBuff=revolve(SMB0DAT);		
				I2CDataBuff++;
				BYTE_NUMBER--;			
			}
			if (!BYTE_NUMBER)	AA= 0;
		break;
			// �յ������ֽ�NACK �ѷ���
			// ����������ɶ����ݼĴ�������ֹͣ����
		case SMB_MRDBNACK:
			STO = 1;
			SM_BUSY = 0; // �ͷ�SMBus
		break;
			// �ڱ�Ӧ������������״̬��û������ͨ�Ÿ�λ
		default:
			STO = 1; // ͨ�Ÿ�λ
			SM_BUSY = 0; 
		break;
		}
	SI=0; // ����жϱ�־
}
/*
{
	switch (SMB0STA){			// Status code for the SMBus (SMB0STA register)
		case SMB_START:					
			SMB0DAT = COMMAND;					// COMMAND should hold slave address + R.
			break;
		case SMB_MTADDNACK:
			STO = 1;
			STA = 1;
			break;
		case SMB_RP_START:
//			SMB0DAT = COMMAND;					// COMMAND should hold slave address + R.
//			STA = 0;											
//			break;
		case SMB_MTADDACK:
		case SMB_MTDBACK:
			if (BYTE_NUMBER)
			{
					if (COMMAND & 0x01)					// If R/W=READ, 
					{
						STA = 1;
					}
					else
					{
						SMB0DAT = *I2CDataBuff;					// If R/W=WRITE, load byte to write.
						I2CDataBuff++;
						BYTE_NUMBER--;			
					}
			}
			else
			{
				STO = 1;
				SM_BUSY = 0;						// Free SMBus
			}
			break;
		
		// Master Transmitter: Data byte transmitted.  NACK received.
		// Slave not responding.  Send STOP followed by START to try again.
		case SMB_MTDBNACK:
			STO = 1;
			STA = 1;
			break;
		
		// Master Transmitter: Arbitration lost.
		// Should not occur.  If so, restart transfer.
		case SMB_MTARBLOST:
			STO = 1;
			STA = 1;
			break;

		// Master Receiver: Slave address + READ transmitted.  NACK received.
		// Slave not responding.  Send repeated start to try again.
		case SMB_MRADDNACK:
			STA = 1;
			break;

		// Data byte received.  ACK transmitted.
		// State should not occur because AA is set to zero in previous state.
		// Send STOP if state does occur.
		case SMB_MRDBACK:
			STO = 1;
			SM_BUSY = 0;
			break;

		// Master Receiver: Slave address + READ transmitted.  ACK received.
		// Set to transmit NACK after next transfer since it will be the last (only) byte.
		case SMB_MRADDACK:
//			AA = 0;										// NACK sent on acknowledge cycle.
//			break;

		// Data byte received.  NACK transmitted.
		// Read operation has completed.  Read data register and send STOP.
		case SMB_MRDBNACK:
			if (BYTE_NUMBER)
			{
					if (COMMAND & 0x01)					// If R/W=READ, 
					{
						*I2CDataBuff=SMB0DAT;		
						I2CDataBuff++;
					}
					BYTE_NUMBER--;			
			}
			else
			{
				STO = 1;
				SM_BUSY = 0;						// Free SMBus
			}
			break;
		// All other status codes meaningless in this application. Reset communication.
		default:
			STO = 1;										// Reset communication.
			SM_BUSY = 0;
			break;
		}
	
	SI=0;													// clear interrupt flag
}
*/
