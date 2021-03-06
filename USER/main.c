#include "stm32f10x.h"
#include "UARTs.h"
#include "PWM.h"
#include "string.h"
#include "math.h"
#include "motor.h"
#include "24l01.h" 
#include "JY901.h"
#include "delay.h"

#include "pidControl.h"


//			P6(TIM3-3)	P5(TIM3-2)
//            \       /
//             \     /
//P7(TIM3-4)--	FLIGHT	--P4(TIM3-1)
//             /     \
//            /       \
//			P2(TIM2-3)	P3(TIM2-4)

struct SAcc 		stcAcc;
struct SGyro		stcGyro;
struct SAngle 	stcAngle;

volatile int tickFlag=0;
u8 tmp_buf[33];		 
char buf[33];
float channel[4];
int key[2];
float motorPWM[6];
int timePWM[6];
int JZ_Angle[3];
int JZ=0;
int JZ_IMU=0;
u8 nrfbuffer[268];

u8 tt ;


motor mo;

u8 Rx[32];

struct SAcc baseAcc;
struct SGyro baseGyro;
struct SAngle baseAngle;



//u8 GyoData[128];
//u8 nrfobuff[128];

#define MASK (0xff)
#define IX (0)
#define IY (1)
#define IZ (2)
void froms16(u8* buffer,s16 tt)
{
	buffer[0]=tt&MASK;
	buffer[1]=(tt>>8)&MASK;
}
void ToGyoData(u8* buffer,struct SAcc* tACC,struct SGyro* tGyro,struct SAngle* tAngle)
{
	froms16(buffer+0,tACC->T);
	froms16(buffer+2,tACC->a[IX]);
	froms16(buffer+4,tACC->a[IY]);
	froms16(buffer+6,tACC->a[IZ]);
	
	froms16(buffer+8,tGyro->T);
	froms16(buffer+10,tGyro->w[IX]);
	froms16(buffer+12,tGyro->w[IY]);
	froms16(buffer+14,tGyro->w[IZ]);

	froms16(buffer+16,tAngle->T);
	froms16(buffer+18,tAngle->Angle[IX]);
	froms16(buffer+ 20,tAngle->Angle[IY]);
	froms16(buffer+22,tAngle->Angle[IZ]);
	
}
u8 START=0;
void SysTick_Handler(void)
{
	//tick++;
	tickFlag=1;
/*
	if(START)
	{
		Control();
		tt=Rx[20];
		TIM3->CCR4=mo.motor1*tt;
		TIM2->CCR4=mo.motor2*tt;
		TIM3->CCR1=mo.motor3*tt;
		TIM3->CCR3=mo.motor4*tt;	
	}
*/
}

void CopeSerialData(unsigned char ucData)
{
	
//	NRF24L01_TxPacket(GyoData);
//	GyoData[30]=ucData;
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
//		GyoData[30]=55;
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) 
	{	
	//	GyoData[30]=11;
	//	NRF24L01_TxPacket(GyoData);
		return;
	}//数据不满11个，则返回
	else
	{
		
		//GyoData[30]=11;
		switch(ucRxBuffer[1])
		{
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);stcAngle.Angle[0]=stcAngle.Angle[0]+32768;break;
		}
		ucRxCnt=0;
//		ToGyoData(GyoData,&stcAcc,&stcGyro,&stcAngle);
//		NRF24L01_TxPacket(GyoData);
	}
}

int main(void)
{
	init();
	
	SysTick_Config(SystemCoreClock/100000);

	TIM2_PWM_Init(1000-1,72);
	TIM3_PWM_Init(1000-1,72);
	Initial_UART1(115200);
	NRF24L01_Init();//different airplane have different address
	
//	TIM_SetCompare1()
	
	while(NRF24L01_Check())
	{ }
	
	
	NRF24L01_RX_Mode();
//	NRF24L01_TX_Mode();
	u8 checkFlag=0;
	u8 cnt=0;
	u8 cnter=0;
	
	baseAcc.a[IX]=0;
	baseAcc.a[IY]=0;
	baseAcc.a[IZ]=0;
				
	baseGyro.w[IX]=0;
	baseGyro.w[IY]=0;
	baseGyro.w[IZ]=0;

	baseAngle.Angle[IX]=0;
	baseAngle.Angle[IY]=0;
	baseAngle.Angle[IZ]=0;
				
	
	while(!checkFlag)
	{
		if(tickFlag)
		{
			if(cnter)
			{
				baseAcc.a[IX]+=(stcAcc.a[IX]/cnt);
				baseAcc.a[IY]+=(stcAcc.a[IY]/cnt);
				baseAcc.a[IZ]+=(stcAcc.a[IZ]/cnt);
				
				baseGyro.w[IX]+=(stcGyro.w[IX]/cnt);
				baseGyro.w[IY]+=(stcGyro.w[IY]/cnt);
				baseGyro.w[IZ]+=(stcGyro.w[IZ]/cnt);

				baseAngle.Angle[IX]+=(stcAngle.Angle[IX]/cnt);
				baseAngle.Angle[IY]+=(stcAngle.Angle[IY]/cnt);
				baseAngle.Angle[IZ]+=(stcAngle.Angle[IZ]/cnt);
				
				--cnter;
				if(cnter==0) checkFlag=1;
			}
			else
			{
				if(NRF24L01_RxPacket(Rx)) continue;
				if(Rx[21])
				{
					cnt=cnter=50;
				}
			}	
	
			tickFlag=0;
		}
	}
	

	while(1)
	{
		if(tickFlag)
		{
			Control();
			tt=Rx[20];
			
			TIM3->CCR4=mo.motor1*tt;
			TIM2->CCR4=mo.motor2*tt;
			TIM3->CCR1=mo.motor3*tt;
			TIM3->CCR3=mo.motor4*tt;	
			tickFlag=0;
		}
	}
}
