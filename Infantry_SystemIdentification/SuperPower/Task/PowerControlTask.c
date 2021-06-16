/**********************************************************************************************************
 * @文件     PowerControlTask.c
 * @说明     功率控制
 * @版本  	 V1.0
 * @作者     陈志鹏
 * @日期     2020.1
**********************************************************************************************************/
#include "main.h"
float UCAP,UDCDC;
unsigned int SUM_UCAP,SUM_UDCDC,SUM_TEMP;
float AD_actual_value;		//电容实际电压
float temperate,temp3v3;
float I_Set = 0;
Pid_Typedef ChargeCtl;
enum POWERSTATE_Typedef PowerState = BAT;

extern unsigned short ADC_ConvertedValue[90];
extern INA260 INA260_1,INA260_2;
extern F405_typedef F405;
extern JudgeReceive_t JudgeReceive;
extern ChassisSpeed_t chassis;
extern F105_Typedef F105;
extern int P_State;

extern short CAP_CrossoverFlag;

/**********************************************************************************************************
*函 数 名: ChargeIO_Configuration
*功能说明: 充放电IO口初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ChargeIO_Configuration(void)
{
	GPIO_InitTypeDef gpio_init;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOC,ENABLE);
	/**********************充电使能***********************************/
	gpio_init.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	gpio_init.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&gpio_init);	
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);		//初始化时不充电
	GPIO_SetBits(GPIOA,GPIO_Pin_7);	//拉高Diode Mode
	/***********************放电选择使能***********************/
	gpio_init.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5;
	gpio_init.GPIO_Mode=GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOC,&gpio_init);	
	Bat_on;
	CAP_off;
}


/**********************************************************************************************************
*函 数 名: ADC_Filter
*功能说明: 数据转换(得到电容实际电压)
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ADC_Filter(void)
{
	int i;
	SUM_UCAP = 0;
	SUM_UDCDC = 0;
	SUM_TEMP = 0;
	for(i=0;i<90;i++)
	{ 
		if(i%3 == 0)
			SUM_UCAP += ADC_ConvertedValue[i];
		else if(i%3 == 1)
			SUM_UDCDC += ADC_ConvertedValue[i];
		else
			SUM_TEMP +=  ADC_ConvertedValue[i];
	}
	UCAP = ((SUM_UCAP/30*825)>>10)/1000.0f;
	UDCDC = ((SUM_UDCDC/30*825)>>10)/1000.0f;
	float temp_temperature;
	temp3v3 = ((SUM_TEMP/30*825)>>10)/1000.0f;
	temp_temperature = temp3v3;
	temp_temperature = (1.43-temp_temperature)/0.0043+25;
	temperate = temp_temperature*=100;
	AD_actual_value = UCAP*7.8;
}



/**********************************************************************************************************
*函 数 名: ChargeCal
*功能说明: 充放电PID控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int MaxChargePower = 45000; 		//最大充电功率，最小充电功率

const short MinCharegePower = 5000;
int ActualPower = 0;
float MaxChargeCurrent = 3.0f;
float My_i;
float ChargeCal()	//由剩余功率计算充电电流
{
	float i;
	MaxChargePower = 1000 * (JudgeReceive.MaxPower - 10) ;
	if(PowerState == BAT)   //0 使用电池放电 此时电池给电容充电，电池给底盘供电
	{
	  if(AD_actual_value > 21)		//电容实际电压 > 减缓充电功率阈值
		{
			ChargeCtl.SetPoint = MinCharegePower;
			MaxChargeCurrent = 1.0f;
		}
		else if(AD_actual_value < 21)
		{
			MaxChargeCurrent = 3.0f;
			ChargeCtl.SetPoint = LIMIT_MAX_MIN(MaxChargePower - INA260_2.Power,MaxChargePower,0);
		}
		
//		ChargeCtl.SetPoint = MySetPoint;
		
		//实际：电容充电功率  SetPoint: 希望充电的功率 单位mW
		ActualPower = INA260_1.Power - INA260_2.Power;
		i = PID_Calc(&ChargeCtl,ActualPower)/1000;
	}
	else if(PowerState == CAP)//1 使用电容放电 此时电池给电容充电，电容给底盘供电
	{
		ChargeCtl.SetPoint = MaxChargePower;
		i = PID_Calc(&ChargeCtl,INA260_1.Power)/1000;
	}
	return i;
}

//float AvailableChargePower;
//float ChargePowerMargin = 5;
//float ActualChargePower;
//float ChargeCurrent;
//float ChargeCurrentMax = 4.0f;
//short MinCharegePower = 5;

////返回还可以再增加的充电电流

//float ChargeCal(void)
//{
//	float addition_current;
//	AvailableChargePower = JudgeReceive.MaxPower - JudgeReceive.realChassispower - ChargePowerMargin;
//	if(AvailableChargePower < -10) //超功率5W
//	{
//		I_Set = 0;
//		addition_current = 0;
//	}
//	else
//	{
//		ChargeCurrentMax = AvailableChargePower / AD_actual_value;
//		if(AD_actual_value < 22)
//			ChargeCtl.SetPoint = AvailableChargePower;
//		else
//			ChargeCtl.SetPoint = MinCharegePower;
//		if(PowerState == BAT)
//		{
//			ActualChargePower = (INA260_1.Power - INA260_2.Power) / 1000.0f;				//统一以W为单位
//		}
//		else if(PowerState == CAP)
//		{
//			ActualChargePower = INA260_1.Power / 1000.0f;
//		}
//		
//		addition_current = LIMIT_MAX_MIN(PID_Calc(&ChargeCtl,ActualChargePower),ChargeCurrentMax,0);
//	}
//	return addition_current;
//}

/**********************************************************************************************************
*函 数 名: Charge_Set
*功能说明: 充电电流设置
*形    参: I_Set
*返 回 值: 无
**********************************************************************************************************/
void Charge_Set(float I_Set)
{
	short DAC_Set;
	I_Set = LIMIT_MAX_MIN(I_Set,MaxChargeCurrent,0);
	DAC_Set = I_Set/10.0*2/3.3*4096;
	DAC_Set = LIMIT_MAX_MIN(DAC_Set,4096,0);
	DAC_SetChannel2Data(DAC_Align_12b_R,DAC_Set);
}


/**********************************************************************************************************
*函 数 名: PidInit
*功能说明: 充电PID初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Pid_ChargeCal_Init()
{
	ChargeCtl.SetPoint = 40000;
	ChargeCtl.P = 0.01;
	ChargeCtl.I = 0;
	ChargeCtl.IMax = 0;
	ChargeCtl.OutMax = 3000;
}

/**********************************************************************************************************
*函 数 名: ChargeControl
*功能说明: 超级电容控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/

float AD_L = 10.0f, AD_H = 14.0f;
float test_current;
void ChargeControl(void)
{
	/******************充电控制*******************/
//放在chassistask中
//	if(JudgeReceive.remainEnergy < 20 && PowerState == BAT)
//    Charge_Off;
//	else
//		Charge_On;
	
	test_current = ChargeCal();
	I_Set += test_current;	
	if(PowerState == BAT)  //bat
	{
		if(CAP_CrossoverFlag == 1)			//启动/停止，大幅转向，有大电流，直接关闭电容充电
		{
			CAP_CrossoverFlag = 0;
			I_Set = 0;
		}
		else
			I_Set = LIMIT_MAX_MIN(I_Set,MaxChargeCurrent,0);
	}
	Charge_Set(I_Set);
	/******************放电控制*******************/
	if(F405.SuperPowerLimit == 0)			//操作手关闭超级电容
	{
		if(PowerState == CAP)		//刚从电容切回
		{
			if(JudgeReceive.remainEnergy > 20)		//缓存能量够用才切回
				{
					CAP_off;   
					Bat_on;
					PowerState = BAT;
				}
		}
	}

//AD_L AD_H有差值：刚开启电容时，由于电容有内阻，瞬间会有4V压降
	else if(F405.SuperPowerLimit > 0)				//操作手开启超级电容
	{
		if(AD_actual_value<AD_L)  	//电容实际电压<13.5
		{
			PowerState = BAT;						//电池放电
			CAP_off;   
			Bat_on;
		}
		else if(AD_actual_value>AD_H)  
		{
			PowerState = CAP;  					//电容放电
			Bat_off;
			CAP_on;
		}
	}
}

/**********************************************************************************************************
*函 数 名: PowerControl_task
*功能说明: 底盘任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint32_t PowerControl_high_water;
void PowerControl_task(void *pvParameters)
{
   while (1) {
			/******************数据获取*******************/
			INA_READ_Vol();			
			INA_READ_Current();
			INA_READ_Power();
			ADC_Filter();
			/*****************超级电容控制****************/
			ChargeControl();
	 
     vTaskDelay(1); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        PowerControl_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


