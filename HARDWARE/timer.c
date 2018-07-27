#include "timer.h"
#include "arm_etm.h"
#include "stm32f10x.h"
#include "core_cm3.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK Mini STM32开发板
//通用定时器 驱动代码			   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/3/06
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
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



//定时器3中断服务程序	 
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

	if(TIM3->SR&0X0001)//溢出中断
	{	
        bubble_sort_tim(values,5);
    }				   
    
    ETM_TraceMode();
    GPIOC->BSRR = (1 << 3);
    bubble_sort_tim(values_2,5);
    GPIOC->BRR = (1 << 3);
    ETM_SetupMode();


	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}
//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;	//TIM3时钟使能    
 	TIM3->ARR=arr;  	//设定计数器自动重装值 
	TIM3->PSC=psc;  	//预分频器设置
	TIM3->DIER|=1<<0;   //允许更新中断				
	TIM3->CR1|=0x01;    //使能定时器3
  	MY_NVIC_Init(1,3,TIM3_IRQn,2);//抢占1，子优先级3，组2									 
}

