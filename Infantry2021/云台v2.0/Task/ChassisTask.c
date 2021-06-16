/**********************************************************************************************************
 * @�ļ�     ChassisTask.c
 * @˵��     ���̿���
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2019.10
**********************************************************************************************************/
#include "main.h"
/*----------------------------------�ڲ�����---------------------------*/
short ChassisAct_Init_Flag=0;
float Theta,SinTheTa,CosTheTa,TanTheTa,Theta0,Speed_Theta;
float ResetPos;
short Be_shooted_flag;
const short FollowMaxSpeedw = 2000;			//�������ת��
const short RotateMaxSpeedw = 6000;			//С�������ת��
/*----------------------------------�ṹ��-----------------------------*/
ChassisSpeed_t chassis;
Pid_Typedef pidChassisPosition,pidChassisPosition_Speed;
Pid_Typedef SOLO_pidChassisPosition;
/*----------------------------------�ⲿ����---------------------------*/
extern RC_Ctl_t RC_Ctl;
extern Status_t Status;
extern Gimbal_Typedef Gimbal;
extern RobotInit_Struct Infantry;
extern F105_Typedef F105;
extern F405_typedef F405;
extern short Turning_flag;
/**********************************************************************************************************
*�� �� ��: Chassis_Powerdown_Cal
*����˵��: ����ģʽ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_Powerdown_Cal()
{
	if(ChassisAct_Init_Flag!=Chassis_Powerdown_Mode) 
		ChassisAct_Init_Flag=Chassis_Powerdown_Mode;
	
  chassis.carSpeedx=0;chassis.carSpeedy=0;chassis.carSpeedw=0;
}

/**********************************************************************************************************
*�� �� ��: Chassis_Act_Cal
*����˵��: ����ģʽ
*��    ��: rc  key
*�� �� ֵ: ��
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

	/*���̸���*/
	ResetPos = (ChassisPostionAngle_TranSform(Infantry.Yaw_init))/360*8192;
	pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos;
	chassis.carSpeedw = PID_Calc(&pidChassisPosition, Gimbal.Yaw.Motor);	
	
	
//	pidChassisPosition_Speed.SetPoint = - chassis.carSpeedw;	
//	chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed, 0);

	if(Status.GimbalMode == Gimbal_DropShot_Mode || Status.GimbalMode == Gimbal_Buff_Mode)//����ģʽ
		chassis.carSpeedw = 0;
}



/**********************************************************************************************************
*�� �� ��: Chassis_SelfProtect_Cal
*����˵��: ����ģʽ
*��    ��: rc  key
*�� �� ֵ: ��
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
	
//	if(F105.HP < F105.Last_HP)				//�����˾�ת��һ��
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
	
//	ResetPos = ABS((ChassisPostionAngle_TranSform(Infantry.Yaw_init) - 100)/360*8192);			//5������
	
//	if(ResetPos > 0 && ResetPos <=1024)
//		chassis.carSpeedw = RotateMaxSpeedw - 4 * ResetPos;
//	else if(ResetPos > 1024 && ResetPos <= 2048)
//		chassis.carSpeedw = RotateMaxSpeedw - 4 * (2048 - ResetPos);
//	else if(ResetPos > 2048 && ResetPos <= 3072)
//		chassis.carSpeedw = RotateMaxSpeedw - 4 * (ResetPos - 2048);
//	else if(ResetPos > 3072 && ResetPos <= 4096)
//		chassis.carSpeedw = RotateMaxSpeedw - 4 * (4096 - ResetPos);
//	else	
//		chassis.carSpeedw = RotateMaxSpeedw;			//����100Wʱ����ת�ٶȣ��ڵ���������

	chassis.carSpeedw = 11000;				//����ֱ������
	
	if(Turning_flag%2 == 1)
		chassis.carSpeedw *= -1;
}

/**********************************************************************************************************
*�� �� ��: Chassis_Solo_Cal
*����˵��: ����ģʽ��ײ�Ƕ��ű���
*��    ��: rc  key
*�� �� ֵ: ��
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
	
	/*���̸���*/
		if(ABS(chassis.carSpeedx) <=10 && ABS(chassis.carSpeedy) < 10)		//��ֹʱŤ��
		{
			ResetPos = (ChassisPostionAngle_TranSform(Infantry.Solo_Yaw_init))/360*8192;		//�����Խǲ�ֵ����
			SOLO_pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos+SOLO_bias;					//����ģʽ�õ�����pid
			chassis.carSpeedw = PID_Calc(&SOLO_pidChassisPosition, Gimbal.Yaw.Motor);
		
			if(SOLO_bias == SOLO_bias_max && ABS(Gimbal.Yaw.Motor - SOLO_pidChassisPosition.SetPoint) < 50)
				SOLO_bias = -SOLO_bias_max;
			else if(SOLO_bias == -SOLO_bias_max && ABS(Gimbal.Yaw.Motor - SOLO_pidChassisPosition.SetPoint) < 50)
				SOLO_bias = SOLO_bias_max;
		}
		else		//��ʼ�˶���ֹͣŤ��
		{
			ResetPos = (ChassisPostionAngle_TranSform(Infantry.Solo_Yaw_init))/360*8192;		//�����Խǲ�ֵ����
			SOLO_pidChassisPosition.SetPoint = Gimbal.Yaw.Motor-ResetPos;
			chassis.carSpeedw = PID_Calc(&SOLO_pidChassisPosition, Gimbal.Yaw.Motor);
		}
	
	//�ٶȻ�
//	pidChassisPosition_Speed.SetPoint = - chassis.carSpeedw;	
//	chassis.carSpeedw = PID_Calc(&pidChassisPosition_Speed, 0);
	
//	if(ABS(chassis.carSpeedw) < 200)		//���ƾ�̬��Ư
//		chassis.carSpeedw = 0;	
//	pidChassisPosition_Speed.SetPoint = -PID_Calc(&pidChassisPosition, Gimbal.Yaw.Motor);
//	chassis.carSpeedw = LIMIT_MAX_MIN(PID_Calc(&SOLO_pidChassisPosition, -0.002f*F105.ChassisSpeedw), 400, -400); 
}

/**********************************************************************************************************
*�� �� ��: Chassis_NoFollow_Cal
*����˵��: �޸���ģʽ
*��    ��: rc  key
*�� �� ֵ: ��
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
	//�޸���
		chassis.carSpeedw = 0;	
}

/**********************************************************************************************************
*�� �� ��: ChassisPostionAngle_TranSform
*����˵��: �Ƕ�ת������
           �ɳ�ʼ�ǶȺ�Zero_CheckYawPosition()������-180~180�ĽǶ�
           �ù�������ֵ�ͳ�ʼ����Yaw����λ�ò��ȡ�ǶȲ�
           ���ǹ������Yaw�Ƿ���ڳ�ʼ����ֵ�����ڳ�ʼ���ĽǶ�ֵʱTheTaΪ��������Ϊ����
           ���ҹ涨TheTaֵ�ķ�ΧΪ���ʼֵ�Ľ�С�Ƕ�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
float ChassisPostionAngle_TranSform(short InitPos)
{
	if( Gimbal.Yaw.Motor<InitPos)
	{
		if(((InitPos-Gimbal.Yaw.Motor)/8191.0f*360.0f)>360)
		{
			Theta0=(InitPos-Gimbal.Yaw.Motor)/8192.0f*360.0f;
			Theta=Theta0-(int)Theta0/360*360;		//��Theta/360����תint(ȡ����)����*360 �����ͰѶ����Ȧ��ȥ��
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
*�� �� ��: Chassis_CurrentPid_Cal
*����˵��: ����̷���xyw���ٶ�
*��    ��: ��
*�� �� ֵ: ��
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
	if(ABS(ResetPos) > 500 && Status.ChassisMode == Chassis_Act_Mode)		//�����û���浽λ
	{
		F405.Follow_state = 1;										//����־λ������
	}else
	{
		F405.Follow_state = 0;
	}
	ChassisCan1Send(&chassis.carSpeedx,&chassis.carSpeedy,&chassis.carSpeedw);
}

/**********************************************************************************************************
*�� �� ��: Chassis_Flag_Set
*����˵��: ģʽ��־����
*��    ��: ��
*�� �� ֵ: ��
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
*�� �� ��: Pid_ChassisPosition
*����˵��: ��������̨��תpid������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Pid_ChassisPosition_Init(void)                
{
	pidChassisPosition.P = 0.8f;				//2.0					//˫��1.0f
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
*�� �� ��: Chassis_task
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
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
