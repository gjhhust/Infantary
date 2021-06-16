#include "main.h"
#define RCC_Port RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB
void My_GPIO_Init(void)//初始化某个只需要用来拉高拉低的GPIO口
{
	GPIO_InitTypeDef gpio;
	RCC_AHB1PeriphClockCmd(RCC_Port,ENABLE);
	
	/*相机外触发*/
	gpio.GPIO_Pin = GPIO_Pin_1;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio);
	
	/*发射机构电压采样*/
	gpio.GPIO_Pin = GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&gpio);
}
