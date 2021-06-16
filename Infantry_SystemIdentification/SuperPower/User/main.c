/**********************************************************************************************************
 * @�ļ�     main.c
 * @˵��     ���ļ�
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2021.4
**********************************************************************************************************/
#include "main.h"

//unsigned char PowerState = 0;
extern ChassisSpeed_t chassis;
short Judgement_DisConnect;
/**********************************************************************************************************
*�� �� ��: main
*����˵��: ������
*��    ��: ��
*�� �� ֵ: ��
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
*�� �� ��: System_Config
*����˵��: ϵͳ��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void System_Config(void)
{
	while(SysTick_Config(72000));	
	ADC_Configuration();//ADC��ʼ��Ҫ���ڴ���֮ǰ����Ȼ���ڲ��ܽ�������
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
	delay_ms(300);//�����ذ��ʼ����ɣ���ֹ���ذ��ʼ������������̷��ʹ�������
}
/**********************************************************************************************************
*�� �� ��: System_Init
*����˵��: ϵͳ������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void System_Init(void)
{
	Pid_ChargeCal_Init();
	Pid_ChassisWheelInit();
	Chassis_Power_Control_Init();		//�����pid��˳���ܵ���
}
/**********************************************************************************************************
*�� �� ��: Offline_Check_task
*����˵��: ���߼������(�������ȼ�̫�ͣ�������Ŵ���ͨ�Żᵼ�����ݷ�����ȥ)
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t Offline_Check_high_water;
extern short F405_DisConnect;//���ذ���߼��

extern ext_student_interactive_header_data_t custom_grapic_draw;
extern uint8_t seq;
void Offline_Check_task(void *pvParameters)
{
   while (1) {
    

		/*���ذ���߼��*/
		if(F405_DisConnect>5)
			F405_Rst();
		F405_DisConnect++;
		
		IWDG_Feed();//ι��

		vTaskDelay(1); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Offline_Check_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


