#include "led.h"

//////////////////////////////////////////////////////////////////////////////////
//±¾³ÌÐòÖ»¹©Ñ§Ï°Ê¹ÓÃ£¬Î´¾­×÷ÕßÐí¿É£¬²»µÃÓÃÓÚÆäËüÈÎºÎÓÃÍ¾
//ALIENTEK mini¢SSTM32¿ª·¢°å
//LEDÇý¶¯´úÂë
//ÕýµãÔ­×Ó@ALIENTEK
//¼¼ÊõÂÛÌ³:www.openedv.com
//ÐÞ¸ÄÈÕÆÚ:2012/9/2
//°æ±¾£ºV1.0
//°æÈ¨ËùÓÐ£¬µÁ°æ±Ø¾¿¡£
//Copyright(C) ¹ãÖÝÊÐÐÇÒíµç×Ó¿Æ¼¼ÓÐÏÞ¹«Ë¾ 2009-2019
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

//³õÊ¼»¯PB5ºÍPE5ÎªÊä³ö¿Ú.²¢Ê¹ÄÜÕâÁ½¸ö¿ÚµÄÊ±ÖÓ
//LED IO³õÊ¼»¯
void LED_Init(void)
{

 GPIO_InitTypeDef  GPIO_InitStructure;

 //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD, ENABLE);	 //Ê¹ÄÜPA,PD¶Ë¿ÚÊ±ÖÓ
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				 //LED0-->PA.8 ¶Ë¿ÚÅäÖÃ
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //ÍÆÍìÊä³ö
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO¿ÚËÙ¶ÈÎª50MHz
 GPIO_Init(GPIOC, &GPIO_InitStructure);					 //¸ù¾ÝÉè¶¨²ÎÊý³õÊ¼»¯GPIOA.8
 GPIO_SetBits(GPIOC,GPIO_Pin_8);						 //PA.8 Êä³ö¸ß

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	    		 //LED1-->PD.2 ¶Ë¿ÚÅäÖÃ, ÍÆÍìÊä³ö
 GPIO_Init(GPIOC, &GPIO_InitStructure);	  				 //ÍÆÍìÊä³ö £¬IO¿ÚËÙ¶ÈÎª50MHz
 GPIO_SetBits(GPIOC,GPIO_Pin_4); 						 //PD.2 Êä³ö¸ß
}
