#ifndef __MAIN_H
#define __MAIN_H


#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))  


/*Library*/
#include <stm32f4xx.h>	 
#include <stm32f4xx_conf.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"

/*Mylib*/
#include "gpio.h"
#include "can1.h"
#include "can2.h"
#include "tim4.h"
#include "tim7.h"
#include "usart1.h"
#include "usart2.h"
#include "usart6.h"
#include "MicroSw.h"
#include "SteeringEngine.h"
#include "frictionwheel.h"
#include "iwdg.h"
#include "laser.h"


/*Algorithm*/
#include "pid.h"
#include "algorithmOfCRC.h"
#include "KalmanFilter.h"

/*Task*/
#include "ZeroCheckTask.h"
#include "DataSendTask.h"
#include "DataReceivetask.h"
#include "ActionTask.h"
#include "GimbalTask.h"
#include "ShootTask.h"
#include "ChassisTask.h"
#include "Start_Task.h"


#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

/*步兵初始参数结构体*/
typedef struct
{
		unsigned short MagOpen;
		unsigned short MagClose;
	  unsigned short Pitch_init;
	  unsigned short Yaw_init;
		unsigned short Solo_Yaw_init;	//左单挑模式的Yaw_init
		unsigned short Low_FrictionSpeed;
		unsigned short Medium_FrictionSpeed;
		unsigned short High_FrictionSpeed;
}RobotInit_Struct;

void System_Config(void);
void System_Init(void);

void Infantry_Init(void);
void delay_ms(unsigned long t);
void delay_us(unsigned long t);

void Offline_Check_task(void *pvParameters);


#endif
