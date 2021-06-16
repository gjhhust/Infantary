/**********************************************************************************************************
 * @文件     main.c
 * @说明     主文件
 * @版本  	 V1.0
 * @作者     陈志鹏
 * @日期     2021.4
**********************************************************************************************************/
#include "main.h"

//unsigned char PowerState = 0;
extern ChassisSpeed_t chassis;
short Judgement_DisConnect;
/**********************************************************************************************************
*函 数 名: main
*功能说明: 主函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int main()
{
  System_Config();
	System_Init();
	
	startTast();
	vTaskStartScheduler();
	while(1)
	{
		
	}
}

/**********************************************************************************************************
*函 数 名: System_Config
*功能说明: 系统初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void System_Config(void)
{
	while(SysTick_Config(72000));	
	ADC_Configuration();//ADC初始化要放在串口之前，不然串口不能接收数据
	DAC1_Init();
	CAN1_Configuration();
	CAN2_Configuration();
	USART2_Configuration();
	UART4_Configuration();
	TIM2_Configuration();
	TIM4_Configuration();
	IWDG_Config(IWDG_Prescaler_64 ,625);
	i2c_init();
	ChargeIO_Configuration();
	Charge_Off;
	delay_ms(300);//等主控板初始化完成，防止主控板初始化过程中向底盘发送错误数据
}
/**********************************************************************************************************
*函 数 名: System_Init
*功能说明: 系统参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void System_Init(void)
{
	Pid_ChargeCal_Init();
	Pid_ChassisWheelInit();
	Chassis_Power_Control_Init();		//这个和pid的顺序不能调换
}
/**********************************************************************************************************
*函 数 名: Offline_Check_task
*功能说明: 掉线检测任务(任务优先级太低，在这里放串口通信会导致数据发不出去)
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint32_t Offline_Check_high_water;
extern short F405_DisConnect;//主控板掉线检测

extern ext_student_interactive_header_data_t custom_grapic_draw;
extern uint8_t seq;
void Offline_Check_task(void *pvParameters)
{
   while (1) {
    

		/*主控板掉线检测*/
		if(F405_DisConnect>5)
			F405_Rst();
		F405_DisConnect++;
		
		IWDG_Feed();//喂狗

		vTaskDelay(1); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Offline_Check_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


