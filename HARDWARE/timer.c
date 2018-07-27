#include "timer.h"
#include "arm_etm.h"
#include "stm32f10x.h"
#include "core_cm3.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK Mini STM32������
//ͨ�ö�ʱ�� ��������			   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/3/06
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	  

void bubble_sort_tim (int *a, int n) 
{
    int i, t, s = 1;
    while (s) 
    {
        s = 0;
        for (i = 1; i < n; i++) 
        {
            if (a[i] < a[i - 1]) 
            {
                t = a[i];
                a[i] = a[i - 1];
                a[i - 1] = t;
                s = 1;            
            }
        }
    }
}



//��ʱ��3�жϷ������	 
void TIM3_IRQHandler(void)
{ 
    int values[5] = {35,2,235,11,2};
    int values_1[5] = {12,11,5,7,4};
    int values_2[5] = {6,5,7,9,2};

    ETM_TraceMode();
    GPIOC->BSRR = (1 << 3);
    bubble_sort_tim(values_1,5);
    GPIOC->BRR = (1 << 3);
    ETM_SetupMode();

	if(TIM3->SR&0X0001)//����ж�
	{	
        bubble_sort_tim(values,5);
    }				   
    
    ETM_TraceMode();
    GPIOC->BSRR = (1 << 3);
    bubble_sort_tim(values_2,5);
    GPIOC->BRR = (1 << 3);
    ETM_SetupMode();


	TIM3->SR&=~(1<<0);//����жϱ�־λ 	    
}
//ͨ�ö�ʱ���жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;	//TIM3ʱ��ʹ��    
 	TIM3->ARR=arr;  	//�趨�������Զ���װֵ 
	TIM3->PSC=psc;  	//Ԥ��Ƶ������
	TIM3->DIER|=1<<0;   //��������ж�				
	TIM3->CR1|=0x01;    //ʹ�ܶ�ʱ��3
  	MY_NVIC_Init(1,3,TIM3_IRQn,2);//��ռ1�������ȼ�3����2									 
}

