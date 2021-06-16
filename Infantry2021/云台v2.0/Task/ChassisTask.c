/**********************************************************************************************************
 * @文件     ChassisTask.c
 * @说明     底盘控制
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2019.10
**********************************************************************************************************/
#include "main.h"
/*----------------------------------内部变量---------------------------*/
short ChassisAct_Init_Flag=0;
float Theta,SinTheTa,CosTheTa,TanTheTa,Theta0,Speed_Theta;
float ResetPos;
short Be_shooted_flag;
const short FollowMaxSpeedw = 2000;			//跟随最高转速
const short RotateMaxSpeedw = 6000;			//小陀螺最高转速
/*----------------------------------结构体-----------------------------*/
ChassisSpeed_t chassis;
Pid_Typedef pidChassisPosition,pidChassisPosition_Speed;
Pid_Typedef SOLO_pidChassisPosition;
/*----------------------------------外部变量---------------------------*/
extern RC_Ctl_t RC_Ctl;
extern Status_t Status;
extern Gimbal_Typedef Gimbal;
extern RobotInit_Struct Infantry;
extern F105_Typedef F105;
extern F405_typedef F405;
extern short Turning_flag;
/**********************************************************************************************************
*函 数 名: Chassis_Powerdown_Cal
*功能说明: 锁车模式
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Chassis_Powerdown_Cal()
{
	if(ChassisAct_Init_Flag!=Chassis_Powerdown_Mode) 
		ChassisAct_Init_Flag=Chassis_Powerdown_Mode;
	
  chassis.carSpeedx=0;chassis.carSpeedy=0;chassis.carSpeedw=0;
}

/**********************************************************************************************************
*函 数 名: Chassis_Act_Cal
*功能说明: 正常模式
*形    参: rc  key
*返 回 值: 无
**********************************************************************************************************/
void Chassis_Act_Cal(Remote rc,Key key) 
{
	if(ChassisAct_Init_Flag!=Chassis_Act_Mode)
	{
    ChassisAct_Init_Flag=Chassis_Act_Mode;
	}
	Theta=ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;
	
	CosTheTa=cos(Theta);
	SinTheTa=sin(Theta);
	TanTheTa=tan(Theta);
	
  if(Status.ControlMode==Control_RC_Mode)
  {
		chassis.carSpeedx = 2*(-1024+rc.ch1); 
		chassis.carSpeedy = 2*(1024-rc.ch0);
  }
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
	  chassis.carSpeedx = -((key.a-key.d)*2000*SinTheTa+(key.s-key.w)*2000*CosTheTa);
		chassis.carSpeedy = -((key.s-key.w)*2000*SinTheTa-(key.a-key.d)*2000*CosTheTa);
	}

	/*底盘跟随*/
	ResetPos = (ChassisPostionAngle_TranSform(Infantry.Yaw_init))/360*8192;
	pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos;
	chassis.carSpeedw = PID_Calc(&pidChassisPosition, Gimbal.Yaw.Motor);	
	
	
//	pidChassisPosition_Speed.SetPoint = - chassis.carSpeedw;	
//	chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed, 0);

	if(Status.GimbalMode == Gimbal_DropShot_Mode || Status.GimbalMode == Gimbal_Buff_Mode)//吊射模式
		chassis.carSpeedw = 0;
}



/**********************************************************************************************************
*函 数 名: Chassis_SelfProtect_Cal
*功能说明: 保护模式
*形    参: rc  key
*返 回 值: 无
**********************************************************************************************************/
float SP_Theta,CosSP_Theta,SinSP_Theta,TanSP_Theta;
void Chassis_SelfProtect_Cal(Remote rc,Key key)
{
	if(ChassisAct_Init_Flag!=Chassis_SelfProtect_Mode)
	  ChassisAct_Init_Flag=Chassis_SelfProtect_Mode;
	
	SP_Theta=ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;
	CosSP_Theta=cos(SP_Theta);
	SinSP_Theta=sin(SP_Theta);
	TanSP_Theta=tan(SP_Theta);
	
	if(Status.ControlMode==Control_RC_Mode)
  {
		chassis.carSpeedx= (-1024+rc.ch0)*SinSP_Theta + (-1024+rc.ch1)*CosSP_Theta; 
		chassis.carSpeedy= (-1024+rc.ch1)*SinSP_Theta - (-1024+rc.ch0)*CosSP_Theta;
  }
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
	  chassis.carSpeedx = -((key.a-key.d)*1200*SinSP_Theta+(key.s-key.w)*1200*CosSP_Theta);
		chassis.carSpeedy = -((key.s-key.w)*1200*SinSP_Theta-(key.a-key.d)*1200*CosSP_Theta);	
	}
	
//	if(F105.HP < F105.Last_HP)				//被射了就转快一点
//	{
//		Be_shooted_flag = 200;
//		F105.Last_HP = F105.HP;
//	}
//	if(Be_shooted_flag > 0)
//	{
//		chassis.carSpeedw = 600;
//		Be_shooted_flag--;
//	}
//	else
	
//	ResetPos = ABS((ChassisPostionAngle_TranSform(Infantry.Yaw_init) - 100)/360*8192);			//5°余量
	
//	if(ResetPos > 0 && ResetPos <=1024)
//		chassis.carSpeedw = RotateMaxSpeedw - 4 * ResetPos;
//	else if(ResetPos > 1024 && ResetPos <= 2048)
//		chassis.carSpeedw = RotateMaxSpeedw - 4 * (2048 - ResetPos);
//	else if(ResetPos > 2048 && ResetPos <= 3072)
//		chassis.carSpeedw = RotateMaxSpeedw - 4 * (ResetPos - 2048);
//	else if(ResetPos > 3072 && ResetPos <= 4096)
//		chassis.carSpeedw = RotateMaxSpeedw - 4 * (4096 - ResetPos);
//	else	
//		chassis.carSpeedw = RotateMaxSpeedw;			//发送100W时的旋转速度，在底盘再限速

	chassis.carSpeedw = 11000;				//底盘直接设置
	
	if(Turning_flag%2 == 1)
		chassis.carSpeedw *= -1;
}

/**********************************************************************************************************
*函 数 名: Chassis_Solo_Cal
*功能说明: 单挑模式，撞角对着别人
*形    参: rc  key
*返 回 值: 无
**********************************************************************************************************/
short SOLO_bias;
short SOLO_bias_max = 550;
short H_pid = 100;
short L_pid = 20;
float H_P = 0.8f;

void Chassis_Solo_Cal(Remote rc,Key key)
{
	if(ChassisAct_Init_Flag!=Chassis_Solo_Mode)
	{
		SOLO_bias = SOLO_bias_max;
	  ChassisAct_Init_Flag=Chassis_Solo_Mode;
	}
	
	Theta=ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;
	CosTheTa=cos(Theta);
	SinTheTa=sin(Theta);
	TanTheTa=tan(Theta);
	
	if(Status.ControlMode==Control_RC_Mode)
  {
		chassis.carSpeedx= (-1024+rc.ch0)*SinTheTa + (-1024+rc.ch1)*CosTheTa; 
		chassis.carSpeedy= (-1024+rc.ch1)*SinTheTa - (-1024+rc.ch0)*CosTheTa;
  }
	
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
	  chassis.carSpeedx = -((key.a-key.d)*2000*SinTheTa+(key.s-key.w)*2000*CosTheTa);
		chassis.carSpeedy = -((key.s-key.w)*2000*SinTheTa-(key.a-key.d)*2000*CosTheTa);
	}
	
	/*底盘跟随*/
		if(ABS(chassis.carSpeedx) <=10 && ABS(chassis.carSpeedy) < 10)		//静止时扭腰
		{
			ResetPos = (ChassisPostionAngle_TranSform(Infantry.Solo_Yaw_init))/360*8192;		//与正对角差值计算
			SOLO_pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos+SOLO_bias;					//单挑模式用单独的pid
			chassis.carSpeedw = PID_Calc(&SOLO_pidChassisPosition, Gimbal.Yaw.Motor);
		
			if(SOLO_bias == SOLO_bias_max && ABS(Gimbal.Yaw.Motor - SOLO_pidChassisPosition.SetPoint) < 50)
				SOLO_bias = -SOLO_bias_max;
			else if(SOLO_bias == -SOLO_bias_max && ABS(Gimbal.Yaw.Motor - SOLO_pidChassisPosition.SetPoint) < 50)
				SOLO_bias = SOLO_bias_max;
		}
		else		//开始运动，停止扭腰
		{
			ResetPos = (ChassisPostionAngle_TranSform(Infantry.Solo_Yaw_init))/360*8192;		//与正对角差值计算
			SOLO_pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos;
			chassis.carSpeedw = PID_Calc(&SOLO_pidChassisPosition, Gimbal.Yaw.Motor);
		}
	
	//速度环
//	pidChassisPosition_Speed.SetPoint = - chassis.carSpeedw;	
//	chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed, 0);
	
//	if(ABS(chassis.carSpeedw) < 200)		//抑制静态零漂
//		chassis.carSpeedw = 0;	
//	pidChassisPosition_Speed.SetPoint = -PID_Calc(&pidChassisPosition, Gimbal.Yaw.Motor);
//	chassis.carSpeedw = LIMIT_MAX_MIN(PID_Calc(&SOLO_pidChassisPosition, -0.002f*F105.ChassisSpeedw), 400, -400); 
}

/**********************************************************************************************************
*函 数 名: Chassis_NoFollow_Cal
*功能说明: 无跟随模式
*形    参: rc  key
*返 回 值: 无
**********************************************************************************************************/
void Chassis_NoFollow_Cal(Remote rc,Key key) 
{
	if(ChassisAct_Init_Flag!=Chassis_NoFollow_Mode)
	{
    ChassisAct_Init_Flag=Chassis_NoFollow_Mode;
	}
	Theta=ChassisPostionAngle_TranSform(Infantry.Yaw_init)/360.0f*6.28318f;
	
	CosTheTa=cos(Theta);
	SinTheTa=sin(Theta);
	TanTheTa=tan(Theta);
	
  if(Status.ControlMode==Control_RC_Mode)
  {
		chassis.carSpeedx = 2*(-1024+rc.ch1); 
		chassis.carSpeedy = 2*(1024-rc.ch0);
  }
	if(Status.ControlMode==Control_MouseKey_Mode)  
	{
	  chassis.carSpeedx = -((key.a-key.d)*2000*SinTheTa+(key.s-key.w)*2000*CosTheTa);
		chassis.carSpeedy = -((key.s-key.w)*2000*SinTheTa-(key.a-key.d)*2000*CosTheTa);
	}
	//无跟随
		chassis.carSpeedw = 0;	
}

/**********************************************************************************************************
*函 数 名: ChassisPostionAngle_TranSform
*功能说明: 角度转换函数
           由初始角度和Zero_CheckYawPosition()来计算-180~180的角度
           用过零检测后的值和初始化的Yaw来做位置差获取角度差
           考虑过零检测的Yaw是否大于初始化的值，大于初始化的角度值时TheTa为正，否则为负；
           并且规定TheTa值的范围为与初始值的较小角度
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
float ChassisPostionAngle_TranSform(short InitPos)
{
	if( Gimbal.Yaw.Motor<InitPos)
	{
		if(((InitPos-Gimbal.Yaw.Motor)/8191.0f*360.0f)>360)
		{
			Theta0=(InitPos-Gimbal.Yaw.Motor)/8192.0f*360.0f;
			Theta=Theta0-(int)Theta0/360*360;		//先Theta/360，再转int(取整数)，再*360 这样就把多余的圈数去除
		}
		else
		{
			Theta=(InitPos-Gimbal.Yaw.Motor)/8192.0f*360.0f;
		}
		if(Theta<180)
		  Theta=-Theta;	
		else
			Theta=360-Theta;
	}
	else
	{
		if(((Gimbal.Yaw.Motor-InitPos))/8192.0f*360.0f>360)
		{
			Theta0=(Gimbal.Yaw.Motor-InitPos)/8192.0f*360.0f;
			Theta=Theta0-(int)Theta0/360*360;
		}
		else
		{
			Theta=(Gimbal.Yaw.Motor-InitPos)/8192.0f*360.0f;
		}
		if(Theta>180)
			Theta=Theta-360;
	}
	return Theta;
}

/**********************************************************************************************************
*函 数 名: Chassis_CurrentPid_Cal
*功能说明: 向底盘发送xyw向速度
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Chassis_CurrentPid_Cal(void)
{
	switch (Status.ChassisMode)
	{
		case Chassis_Act_Mode:
			Chassis_Act_Cal(RC_Ctl.rc,RC_Ctl.key);
			break;
		case Chassis_SelfProtect_Mode:
			 Chassis_SelfProtect_Cal(RC_Ctl.rc,RC_Ctl.key);
			break;
		case Chassis_Solo_Mode:
			Chassis_Solo_Cal(RC_Ctl.rc,RC_Ctl.key);
			break;
		case Chassis_NoFollow_Mode:
			Chassis_NoFollow_Cal(RC_Ctl.rc,RC_Ctl.key);
			break;
		case Chassis_Powerdown_Mode:
			Chassis_Powerdown_Cal();
			break;
		default:
			break;
	}
	F405.Chassis_Flag = Status.ChassisMode;
	if(ABS(ResetPos) > 500 && Status.ChassisMode == Chassis_Act_Mode)		//如果还没跟随到位
	{
		F405.Follow_state = 1;										//发标志位给底盘
	}else
	{
		F405.Follow_state = 0;
	}
	ChassisCan1Send(&chassis.carSpeedx,&chassis.carSpeedy,&chassis.carSpeedw);
}

/**********************************************************************************************************
*函 数 名: Chassis_Flag_Set
*功能说明: 模式标志设置
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
//void Chassis_Flag_Set(void)
//{
//	switch(Status.ChassisMode)
//	{
//		case Chassis_Act_Mode:
//			F405.Chassis_Flag = 0;
//			break;
//		case Chassis_SelfProtect_Mode:
//			F405.Chassis_Flag = 1;
//			break;
//		case Chassis_Solo_Mode:
//			F405.Chassis_Flag = 2;
//			break;
//		case Chassis_Powerdown_Mode:
//			Chassis_beSet_flag = 0;
//			break;
//		default:
//			F405.Chassis_Flag = 0;
//			break;
//	}
//}
/**********************************************************************************************************
*函 数 名: Pid_ChassisPosition
*功能说明: 底盘随云台旋转pid参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Pid_ChassisPosition_Init(void)                
{
	pidChassisPosition.P = 0.8f;				//2.0					//双环1.0f
	pidChassisPosition.I = 0.0f;					
	pidChassisPosition.D = 0.0f;				
	pidChassisPosition.IMax = 200.0f;
	pidChassisPosition.OutMax = 2000.0f;
	
	SOLO_pidChassisPosition.P = 1.2f;		
	SOLO_pidChassisPosition.I = 0.0f;					
	SOLO_pidChassisPosition.D = 0.0f;				
	SOLO_pidChassisPosition.IMax = 200.0f;
	SOLO_pidChassisPosition.OutMax = 2000.0f;
	
	pidChassisPosition_Speed.P = 2.3f;			//2.3
	pidChassisPosition_Speed.I = 0.0f;
	pidChassisPosition_Speed.D = 0.0f;
	pidChassisPosition_Speed.IMax = 200.0f;
	pidChassisPosition_Speed.OutMax = 8000.0f;
}

/**********************************************************************************************************
*函 数 名: Chassis_task
*功能说明: 底盘任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint32_t Chassis_high_water;
void Chassis_task(void *pvParameters)
{
   while (1) {
    Chassis_CurrentPid_Cal(); 
		 
    vTaskDelay(1); 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
