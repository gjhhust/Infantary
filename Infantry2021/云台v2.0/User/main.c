/**********************************************************************************************************
 * @�ļ�     main.c
 * @˵��     ���ļ�
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2019.10
**********************************************************************************************************/
#include "main.h"
extern KalmanFilter_t pitch_Kalman, yaw_Kalman;
extern KalmanFilter_Init_t K;
unsigned volatile long run_time_check = 0;	//���������ּ��׼���������
short fric_flag = 0;//Ħ���ֵ����ʼ����־
RobotInit_Struct Infantry;

extern short FrictionWheel_speed;
extern short KalMan_doneflag;
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
	Tim4_Configuration();
	delay_ms(100);
	delay_ms(5000);
	USART1_Configuration();
	delay_ms(100);
	USART6_Configuration();
	delay_ms(100);
	My_GPIO_Init();	//���ڸ��������ͬ��
	delay_ms(100);
	MicroSwConfigration();
	delay_ms(100);
  SteeringEngine_Configuration();
	delay_ms(100);
	TIM7_Configuration();
	delay_ms(100);
	CAN1_Configuration();
	delay_ms(100);
	CAN2_Configuration();
	delay_ms(100);
	IWDG_Config(IWDG_Prescaler_128 ,625);
	delay_ms(100);
}
/**********************************************************************************************************
*�� �� ��: System_Init
*����˵��: ϵͳ������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void System_Init(void)
{
	ZeroCheck_Init();
	Infantry_Init();
	Pid_ChassisPosition_Init();
	Pid_YawGyroPosSpeed_Init();
	Pid_PitchGyroPosSpeed_Init();
	Pid_PitchMotorPos_GyroSpeed_Init();
	Pid_YawMotorPos_GyroSpeed_Init();
	Pid_BodanMotor_Init();
	KalmanFilter_Init(&pitch_Kalman, &K);
	KalmanFilter_Init(&yaw_Kalman, &K);//PC�ȴ�����������STM32�Ῠ��arm32�ļ���(δ֪)
	KalMan_doneflag = 1;
}

/**********************************************************************************************************
*�� �� ��: Infantry_Init
*����˵��: ����������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Infantry_Init(void)
{	
/*����*/
//	Infantry.Yaw_init=4730;
//	Infantry.Pitch_init=6880;
//	Infantry.MagOpen=2295;
//	Infantry.MagClose=783;
//	Infantry.Solo_Yaw_init = 3760;
//	Infantry.Low_FrictionSpeed = 5560;
//	Infantry.Medium_FrictionSpeed = 5750;
//	Infantry.High_FrictionSpeed = 6780;	

/*�ĺ�*/	
	Infantry.Yaw_init=1040; 
	Infantry.Pitch_init=6732;
	Infantry.MagOpen=1000;
	Infantry.MagClose=2455;
	Infantry.Solo_Yaw_init = 20;
	Infantry.Low_FrictionSpeed = 5695;
	Infantry.Medium_FrictionSpeed = 5825;
	Infantry.High_FrictionSpeed = 6680;
}

/**********************************************************************************************************
*�� �� ��: delay_ms
*����˵��: ms��ʱ
*��    ��: ��
*�� �� ֵ: ��
************************** ++********************************************************************************/
void delay_ms(unsigned long t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=10300;
 		while(a--);
	}
}
void delay_us(unsigned long t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=37;
 		while(a--);
	}
}
/**********************************************************************************************************
*�� �� ��: Offline_Check_task
*����˵��: ���߼������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t Offline_Check_high_water;
extern short RC_DisConnect;//ң�������߼��
extern short F105_DisConect;//���ʰ���߼��
void Offline_Check_task(void *pvParameters)
{
   while (1) {
//	vTaskList((char *)&CPU_RunInfo); // ��ȡ��������ʱ����Ϣ
//	printf("---------------------------------------------\r\n");
//	printf("������ ����״̬ ���ȼ� ʣ��ջ �������\r\n");
//	printf("%s", CPU_RunInfo);
//	printf("---------------------------------------------\r\n");
	/*ң�������߼��*/
	if(RC_DisConnect>100)
		RC_Rst();
	RC_DisConnect++;
	/*���ݰ���߼��*/
	if(F105_DisConect>150)
		F105_Rst();
	F105_DisConect++;
	if(run_time_check >100000) run_time_check = 0;
		 
  vTaskDelay(5); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Offline_Check_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}



