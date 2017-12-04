#include "timer.h"


//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
void TIM12_IT_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  ///ʹ��TIM12ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseInitStructure);//��ʼ��TIM12
	
	TIM_ITConfig(TIM12,TIM_IT_Update,ENABLE); //����ʱ��12�����ж�
	TIM_Cmd(TIM12,ENABLE); //ʹ�ܶ�ʱ��12
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM8_BRK_TIM12_IRQn ; //��ʱ��12�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
