/**********************************************************************************************************
 * @文件     usart2.c
 * @说明     usart2初始化：各种串口测试
 * @版本  	 V1.0
 * @作者     陈志鹏
 * @日期     2021.3
**********************************************************************************************************/
#include "main.h"

int fputc(int ch,FILE *p)  //函数默认的，在使用printf函数时自动调用
{
	USART_SendData(USART2,(u8)ch);	
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	return ch;
}

/**********************************************************************************************************
*函 数 名: USART2_Configuration
*功能说明: usart2配置函数(数据测试)
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void USART2_Configuration(void)
{
    USART_InitTypeDef usart2;
		GPIO_InitTypeDef  gpio;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); 
	
    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA,&gpio);

		usart2.USART_BaudRate = 115200;
		usart2.USART_WordLength = USART_WordLength_8b;
		usart2.USART_StopBits = USART_StopBits_1;
		usart2.USART_Parity = USART_Parity_No;
		usart2.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART2,&usart2);
		
		USART_Cmd(USART2,ENABLE);
}
