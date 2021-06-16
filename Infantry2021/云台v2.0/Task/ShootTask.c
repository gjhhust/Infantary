/**********************************************************************************************************
 * @文件     ShootTask.c
 * @说明     发弹控制
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2020.1
**********************************************************************************************************/
# include "main.h"
/*----------------------------------内部变量---------------------------*/
short MirocPosition;
short FrictionWheel_speed=0,BulletSpeed,BodanMotorCurrent,ShootAct_Init_Flag;
short BodanMotorCurrent;
int SendToTx2BullectCnt;
int Shoot_Init_flag = 0;
/*----------------------------------结构体------------------------------*/
Pid_Typedef PidBodanMotorPos,PidBodanMotorSpeed;
/*----------------------------------外部变量---------------------------*/
extern RC_Ctl_t RC_Ctl;
extern int Bodan_Pos;
extern F105_Typedef F105;//F105
extern F405_typedef F405;
extern Status_t Status;
extern Gimbal_Typedef Gimbal;
extern PC_Receive_t PC_Receive;
extern BodanMotorReceive_Typedef BodanReceive;
extern short armor_state;
extern RobotInit_Struct Infantry;
/**********************************************************************************************************
*函 数 名: FrictionSpeedChoose
*功能说明: 摩擦轮转速选择
*形    参: flag
*返 回 值: 无
**********************************************************************************************************/
void FrictionSpeedChoose(void)
{
	switch(F105.BulletSpeedLevel)
	{
		case 0:
		{
			FrictionWheel_speed = Infantry.Low_FrictionSpeed;
			break;
		}
		case 1:
		{
				FrictionWheel_speed = Infantry.Medium_FrictionSpeed;
			break;
		}
		case 2:
		{
				FrictionWheel_speed = Infantry.High_FrictionSpeed;
			break;
		}
		default:
		{
			FrictionWheel_speed = Infantry.Low_FrictionSpeed;
			break;
		}
	}
}

/**********************************************************************************************************
*函 数 名: Shoot_Fire_Cal
*功能说明: 正常模式
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Shoot_Fire_Cal()
{
	if(ShootAct_Init_Flag!=1)
		ShootAct_Init_Flag=1;
	if(F105.IsShootAble==1)
	{
		if(ABS(Bodan_Pos-PidBodanMotorPos.SetPoint)<8000)
		  PidBodanMotorPos.SetPoint=PidBodanMotorPos.SetPoint+MirocPosition;  
	}
		MirocPosition = 0;
}

/**********************************************************************************************************
*函 数 名: Shoot_Check_Cal
*功能说明: 检录模式[打开激光，摩擦轮/射频3~4发]
*形    参: rc
*返 回 值: 无
**********************************************************************************************************/
int delay_time = 0;
void Shoot_Check_Cal()
{ 
	delay_time++;
	if(ShootAct_Init_Flag!=2)
	ShootAct_Init_Flag=2;
	MirocPosition = 0;
	if(delay_time > 800)
		if(F105.IsShootAble==1 && Shoot_Init_flag == 1)
			if(ABS(Bodan_Pos-PidBodanMotorPos.SetPoint)<8000)
			{
				PidBodanMotorPos.SetPoint = PidBodanMotorPos.SetPoint+600;  
			}
}

/**********************************************************************************************************
*函 数 名: Shoot_Test_Cal
*功能说明: 测试模式，用于辅瞄上位机测试
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
void Shoot_Test_Cal()
{ 
	if(ShootAct_Init_Flag!=0xcf)
	ShootAct_Init_Flag=0xcf;
	MirocPosition = 0;
	if(F105.IsShootAble==1 && armor_state == ARMOR_AIMED)
		if(ABS(Bodan_Pos-PidBodanMotorPos.SetPoint)<8000)
		{
			PidBodanMotorPos.SetPoint = PidBodanMotorPos.SetPoint+Onegrid;  
		}
}

/**********************************************************************************************************
*函 数 名: Shoot_Powerdown_Cal
*功能说明: 锁定模式[微动开关/摩擦轮/拨弹转速]
*形    参: rc
*返 回 值: 无
**********************************************************************************************************/
void Shoot_Powerdown_Cal(void)
{
	if(ShootAct_Init_Flag!=4)
	  ShootAct_Init_Flag=4;
	PidBodanMotorPos.SetPoint=0;
	PidBodanMotorPos.SetPoint=Bodan_Pos;
}

/**********************************************************************************************************
*函 数 名: Shoot_Tx2_Cal
*功能说明: 辅瞄模式
*形    参: rc
*返 回 值: 无
**********************************************************************************************************/
extern short armor_state;
extern float k_onegrid;
void Shoot_Tx2_Cal()
{
	if(ShootAct_Init_Flag!=3)
	{
		MirocPosition = 0;
		ShootAct_Init_Flag=3;
		SendToTx2BullectCnt=PC_Receive.ReceiveFromTx2BullectCnt=0;
	}
/***********************************************Armor******************************************/	
	if(Status.GimbalMode == Gimbal_Armor_Mode)
	{
		if(F105.IsShootAble==1)
  	{
			if(ABS(PC_Receive.RCPitch - Gimbal.Pitch.MotorTransAngle)<1.5f && ABS(PC_Receive.RCYaw - Gimbal.Yaw.Gyro)<1.5f)	//已经辅瞄到位，自动开火
			{
				if(ABS(PidBodanMotorPos.SetPoint-Bodan_Pos)<5000)
				{
					MirocPosition = k_onegrid * Onegrid;
					SendToTx2BullectCnt++;
				}
			}
    }
	}
	
	if(Status.GimbalMode == Gimbal_Buff_Mode)
	{
		if(F105.IsShootAble==1 && armor_state == ARMOR_AIMED)
  	{
			if(ABS(PC_Receive.RCPitch - Gimbal.Pitch.MotorTransAngle)<1.5f && ABS(PC_Receive.RCYaw - Gimbal.Yaw.MotorTransAngle)<1.5f)	//已经辅瞄到位，自动开火
			{
				if(ABS(PidBodanMotorPos.SetPoint-Bodan_Pos)<8000)
				{
					MirocPosition = k_onegrid * Onegrid;
					SendToTx2BullectCnt++;
				}
			}
    }
	}	
//测试
//		if(armor_state == ARMOR_AIMED)
//  	{
//			if(ABS(PC_Receive.RCPitch - Gimbal.Pitch.MotorTransAngle)<1.5f && ABS(PC_Receive.RCYaw - Gimbal.Yaw.MotorTransAngle)<1.5f)	//已经辅瞄到位，自动开火
//			{
//				if(ABS(PidBodanMotorPos.SetPoint-Bodan_Pos)<8000)
//				{
//					MirocPosition = k_onegrid * Onegrid;
//					SendToTx2BullectCnt++;
//				}
//			}
//    }
//	}
	/*不限制发弹时间间隔*/
//	if(Status.GimbalMode == Gimbal_Buff_Mode)
//	{
//		if(F105.IsShootAble==1)
//  	{
//			if(ABS(PC_Receive.RCPitch - Gimbal.Pitch.MotorTransAngle)<0.8f && ABS(PC_Receive.RCYaw - Gimbal.Yaw.Gyro)<0.8f)	//已找到目标并且在一定范围内
//			{
//				if(ABS(PidBodanMotorPos.SetPoint-Bodan_Pos)<8000)
//				{
//					MirocPosition = k_onegrid * Onegrid;
//					SendToTx2BullectCnt++;
//				}
//			}
//    }
//	}
	
//	/*限制发弹时间间隔*/
//	if(Status.GimbalMode == Gimbal_Buff_Mode)
//	{
//		if(idata.IsShootAble==1)
//  	{
//			if(PC_Receive.ReceiveFromTx2BullectCnt>SendToTx2BullectCnt)
//			{
//				  if(idata.IsShootAble==1)
//					{
//						if(DaFuBodanTick %300 == 0)
//						{
//							DaFuBodanTick =0;
//							if(ABS(PidBodanMotorPos.SetPoint- ZeroCheck_BodanPosOutPut())<8000)
//							{
//					      MirocPosition = Onegrid;
//					      SendToTx2BullectCnt++;
//							}
//						}
//					}
//					DaFuBodanTick++;
//			}
//			else
//				DaFuBodanTick = 0;
//    }
//	}
	
//	if(F105.IsShootAble==1)
		PidBodanMotorPos.SetPoint=PidBodanMotorPos.SetPoint+MirocPosition;  
		MirocPosition = 0;
}

/**********************************************************************************************************
*函 数 名: BodanMotor_CurrentPid_Cal
*功能说明: 拨弹电机位置环双环计算输出
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BodanMotor_CurrentPid_Cal(void)
{
	switch(Status.ShootMode)//射击模式选择
	{
		case Shoot_Check_Mode:
			Shoot_Check_Cal();
			break;
		case Shoot_Fire_Mode:
			Shoot_Fire_Cal();
			break;
		case Shoot_Tx2_Mode:
			Shoot_Test_Cal();			//210424测试
//			Shoot_Tx2_Cal();
			break;
		case Shoot_Powerdown_Mode:
			Shoot_Powerdown_Cal();
			break;
		default:
			break;
	}
	PidBodanMotorSpeed.SetPoint = PID_Calc(&PidBodanMotorPos,Bodan_Pos);	 
	
	BodanMotorCurrent = (short)PID_Calc(&PidBodanMotorSpeed,BodanReceive.RealSpeed);
	
// 	Bodan_F105_CurSend(BodanMotorCurrent,&BulletSpeed,&SuperPower,&Rc_k_100);  
	BodanCan1Send((short)BodanMotorCurrent);
}

/**********************************************************************************************************
*函 数 名: Pid_BodanMotor
*功能说明: 拨弹电机位置速度环pid参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Pid_BodanMotor_Init(void)
{
  PidBodanMotorPos.P=0.25f;
	PidBodanMotorPos.I=0.0f;
	PidBodanMotorPos.D=0.0f;
	PidBodanMotorPos.IMax=1500.0f;
	PidBodanMotorPos.SetPoint=0.0f;
	PidBodanMotorPos.OutMax = 10000.0f;
	
	PidBodanMotorSpeed.P=5.0f;
	PidBodanMotorSpeed.I=0.0f;
	PidBodanMotorSpeed.D=0.0f;
	PidBodanMotorSpeed.IMax=500.0f;
	PidBodanMotorSpeed.SetPoint=0.0f;
	PidBodanMotorSpeed.OutMax = 10000.0f;
}


/**********************************************************************************************************
*函 数 名: Shoot_task
*功能说明: 拨弹任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
#define POWER_ON 1
#define POWER_OFF 0
uint32_t Shoot_high_water;
int Shoot_Power = 0;		//读取发射机构电压
int test_shoot_power;
int init_time = 0;
short SHOOT_HADBE_INIT = 0;	//已经上电后初始化过的标志
void Shoot_task(void *pvParameters)
{
   static long init_time;
	 static short SHOOT_HADBE_INIT;	//已经上电后初始化过的标志
   while (1) {
		 Shoot_Power = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
		 if(Shoot_Power == POWER_ON)
		 {
			if(init_time>5000)		
			{
				FrictionSpeedChoose();
				Shoot_Init_flag = 1;
				SHOOT_HADBE_INIT = 1;		//只要初始化过一次就会被标记，不清除
			}
			else		//初始化
			{
				Shoot_Init_flag = 0;
				FrictionWheel_speed = Init_FritionSpeed;
			}
			if(Status.ChassisMode == Chassis_Powerdown_Mode)
			{
				init_time = 0;
				Shoot_Init_flag = 0;				//禁止拨盘转
			}else if(Status.ChassisMode != Chassis_Powerdown_Mode && SHOOT_HADBE_INIT == 1)		//已经初始化成功一次了，并且切回了非掉电模式
			{
				init_time = 5000;
			}
			init_time++;
			FrictionWheel_Set(FrictionWheel_speed);
		 }
		 else if(Shoot_Power == POWER_OFF)
		 {
				Shoot_Init_flag = 0;				//禁止拨盘转即可
			  init_time = 0;
				SHOOT_HADBE_INIT = 0;
		 }
			
		 BodanMotor_CurrentPid_Cal();
     vTaskDelay(1); 
#if  INCLUDE_uxTaskGetStackHighWaterMark
        Shoot_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
