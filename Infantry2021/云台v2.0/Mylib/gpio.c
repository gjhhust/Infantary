#include "main.h"
#define RCC_Port RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB
void My_GPIO_Init(void)//��ʼ��ĳ��ֻ��Ҫ�����������͵�GPIO��
{
	GPIO_InitTypeDef gpio;
	RCC_AHB1PeriphClockCmd(RCC_Port,ENABLE);
	
	/*����ⴥ��*/
	gpio.GPIO_Pin = GPIO_Pin_1;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio);
	
	/*���������ѹ����*/
	gpio.GPIO_Pin = GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&gpio);
}
