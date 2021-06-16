/**********************************************************************************************************
 * @文件     main.c
 * @说明     主文件
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2019.10
**********************************************************************************************************/
#include "main.h"
extern KalmanFilter_t pitch_Kalman, yaw_Kalman;
extern KalmanFilter_Init_t K;
unsigned volatile long run_time_check = 0;	//用于做各种简易计数器计数
short fric_flag = 0;//摩擦轮电机初始化标志
RobotInit_Struct Infantry;

extern short FrictionWheel_speed;
extern short KalMan_doneflag;
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
	Tim4_Configuration();
	delay_ms(100);
	delay_ms(5000);
	USART1_Configuration();
	delay_ms(100);
	USART6_Configuration();
	delay_ms(100);
	My_GPIO_Init();	//用于辅瞄做相机同步
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
*函 数 名: System_Init
*功能说明: 系统参数初始化
*形    参: 无
*返 回 值: 无
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
	KalmanFilter_Init(&yaw_Kalman, &K);//PC先传数据再运行STM32会卡在arm32文件内(未知)
	KalMan_doneflag = 1;
}

/**********************************************************************************************************
*函 数 名: Infantry_Init
*功能说明: 步兵参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Infantry_Init(void)
{	
/*三号*/
//	Infantry.Yaw_init=4730;
//	Infantry.Pitch_init=6880;
//	Infantry.MagOpen=2295;
//	Infantry.MagClose=783;
//	Infantry.Solo_Yaw_init = 3760;
//	Infantry.Low_FrictionSpeed = 5560;
//	Infantry.Medium_FrictionSpeed = 5750;
//	Infantry.High_FrictionSpeed = 6780;	

/*四号*/	
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
*函 数 名: delay_ms
*功能说明: ms延时
*形    参: 无
*返 回 值: 无
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
*函 数 名: Offline_Check_task
*功能说明: 掉线检测任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint32_t Offline_Check_high_water;
extern short RC_DisConnect;//遥控器掉线检测
extern short F105_DisConect;//功率板掉线检测
void Offline_Check_task(void *pvParameters)
{
   while (1) {
//	vTaskList((char *)&CPU_RunInfo); // 获取任务运行时间信息
//	printf("---------------------------------------------\r\n");
//	printf("任务名 任务状态 优先级 剩余栈 任务序号\r\n");
//	printf("%s", CPU_RunInfo);
//	printf("---------------------------------------------\r\n");
	/*遥控器掉线检测*/
	if(RC_DisConnect>100)
		RC_Rst();
	RC_DisConnect++;
	/*电容板掉线检测*/
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



