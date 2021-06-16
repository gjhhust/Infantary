/**********************************************************************************************************
 * @�ļ�     usart2.c
 * @˵��     usart2��ʼ�������ִ��ڲ���
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2021.3
**********************************************************************************************************/
#include "main.h"

int fputc(int ch,FILE *p)  //����Ĭ�ϵģ���ʹ��printf����ʱ�Զ�����
{
	USART_SendData(USART2,(u8)ch);	
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	return ch;
}

/**********************************************************************************************************
*�� �� ��: USART2_Configuration
*����˵��: usart2���ú���(���ݲ���)
*��    ��: ��
*�� �� ֵ: ��
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
