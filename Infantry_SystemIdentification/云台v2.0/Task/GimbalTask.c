/**********************************************************************************************************
 * @�ļ�     GimbalTask.c
 * @˵��     ��̨����
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2020.1
**********************************************************************************************************/
#include "main.h"
/*----------------------------------�ڲ�����---------------------------*/
int gimbal_pitch_max = 25;		//��ʼ��PITCH��λ
int gimbal_pitch_min = -20;

int inttoshort[4];
short GimbalAct_Init_Flag=0;

short YawCurrent,PitchCurrent;
float GimbalYawPos,GimbalPitchPos;

float Recent_Angle_Buff;
float Recent_Yaw_Angle_Armor;
float Recent_Pitch_Angle_Armor;
/*-----------------------------------�ṹ��-----------------------------*/
Pid_Typedef PidPitchSpeed,PidPitchPos,PidYawSpeed,PidYawPos;
Pid_Typedef PidPitchAidPos,PidPitchAidSpeed,PidYawAidPos,PidYawAidSpeed;
/*----------------------------------�ⲿ����---------------------------*/
extern RC_Ctl_t RC_Ctl; 
extern Status_t Status;

extern Gimbal_Typedef Gimbal;
extern Gyro_Typedef GyroReceive;//����������
extern RobotInit_Struct Infantry;
extern short FrictionWheel_speed;
extern F405_typedef F405;
extern PC_Receive_t PC_Receive;


/**********************************************************************************************************
*�� �� ��: Gimbal_Powerdown_Cal
*����˵��: ��̨����ģʽ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Gimbal_Powerdown_Cal()
{
	if(GimbalAct_Init_Flag!=Gimbal_Powerdown_Mode)
	{
		Laser_Off();
	  GimbalAct_Init_Flag=Gimbal_Powerdown_Mode;
	}
	PidPitchSpeed.SetPoint=0;
	PidPitchPos.SetPoint = Gimbal.Pitch.MotorTransAngle;
	PidYawSpeed.SetPoint=0;
  PidYawPos.SetPoint = Gimbal.Yaw.Gyro;
	PitchCurrent=0;
	YawCurrent=0;
}

/**********************************************************************************************************
*�� �� ��: Gimbal_Act_Cal
*����˵��: ��̨����ģʽ
*��    ��: rc  mouse  Pc_RecvData
*�� �� ֵ: ��
**********************************************************************************************************/
float test_pitch = 0;
float test_yaw = 0;
int aaaaa;
void Gimbal_Act_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv)
{
	if( GimbalAct_Init_Flag!=Gimbal_Act_Mode)
	{
		Laser_On();
		GimbalPitchPos = Gimbal.Pitch.MotorTransAngle;				//�Ӵ��ģʽ�лأ�����pitch����ǣ�yaw�����ǽǣ����Ҷ�
		GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalAct_Init_Flag=Gimbal_Act_Mode;
	}

	if(Status.ControlMode==Control_RC_Mode)//Rc_Control
	{
	  GimbalYawPos   += (1024-rc.ch2)*0.0005f;
	  GimbalPitchPos -= (1024-rc.ch3)*0.0003f;//��������
	}
	if(Status.ControlMode==Control_MouseKey_Mode)//Mouse_Key
	{
    GimbalPitchPos -= mouse.y*0.005f;
    GimbalYawPos   -= mouse.x*0.005f;
		GimbalPitchPos += mouse.z*0.001f;
	}

	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,gimbal_pitch_max,gimbal_pitch_min);//��λ(�õ���Ƕ�)			//��ס��С����˿���MotorTransangle��pid
  /***********************************************************************************/
	PidPitchPos.SetPoint = GimbalPitchPos;
	PidYawPos.SetPoint = GimbalYawPos;
	
//	PidPitchPos.SetPoint = test_pitch;
//	PidYawPos.SetPoint = test_yaw;
	
  /**************************************�������ֵ**************************************/
//PITCH                
//  PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,F405.Chassis_Flag?Gimbal.Pitch.Gyro:Gimbal.Pitch.MotorTransAngle);   
  PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,/*GyroReceive.PITCH*/Gimbal.Pitch.MotorTransAngle);    	
	inttoshort[0]=-(PID_Calc(&PidPitchSpeed,GyroReceive.GY));//��������
	PitchCurrent=(short)inttoshort[0];

//YAW
	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.Gyro);					//��ҪYAW�������ǽǶ���ActualValue
	PidYawSpeed.SetPoint=LIMIT_MAX_MIN(PidYawSpeed.SetPoint,5.5f,-5.5f);    
	inttoshort[1]=PID_Calc(&PidYawSpeed,GyroReceive.GZ);
	YawCurrent=inttoshort[1];
	/* pid_test ����*/
//	test_gyro_pitch = -Gimbal.Pitch.Gyro;
//	test_gyro_yaw = -Gimbal.Yaw.Gyro;
}

/**********************************************************************************************************
*�� �� ��: Gimbal_Armor_Cal
*����˵��: ��̨����ģʽ
*��    ��: rc  mouse  Pc_RecvData
*�� �� ֵ: ��
**********************************************************************************************************/
float speed_limit = 30.0f;
void Gimbal_Armor_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv)
{
	if(GimbalAct_Init_Flag!=Gimbal_Armor_Mode)
	{
		GimbalAct_Init_Flag=Gimbal_Armor_Mode;
		GimbalYawPos = Gimbal.Yaw.Gyro;
	  GimbalPitchPos=Gimbal.Pitch.MotorTransAngle;
		Pc_Recv->RCPitch=Gimbal.Pitch.MotorTransAngle;
		Pc_Recv->RCYaw = Gimbal.Yaw.Gyro;
	}
	/**�����������***/
	Recent_Pitch_Angle_Armor = Pc_Recv->RCPitch;
	while(ABS(Recent_Pitch_Angle_Armor) >= 90 )
	{
		if(Recent_Pitch_Angle_Armor<0)
			 Recent_Pitch_Angle_Armor=Recent_Pitch_Angle_Armor+180;  
		else
			 Recent_Pitch_Angle_Armor=-(Recent_Pitch_Angle_Armor-180);
	}
	
	
	Recent_Yaw_Angle_Armor = Pc_Recv->RCYaw;
	while(ABS(Gimbal.Yaw.Gyro - Recent_Yaw_Angle_Armor) >= 180 )
	{
		if(Gimbal.Yaw.Gyro<0)
			 Recent_Yaw_Angle_Armor=Recent_Yaw_Angle_Armor-360;  
		else
			 Recent_Yaw_Angle_Armor=Recent_Yaw_Angle_Armor+360;
	}
	
	GimbalPitchPos= Recent_Pitch_Angle_Armor;
	GimbalYawPos = Recent_Yaw_Angle_Armor;
	
	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,gimbal_pitch_max,gimbal_pitch_min);//��λ
	/***************************************************************************************/
	PidPitchAidPos.SetPoint  =    GimbalPitchPos;
	PidYawAidPos.SetPoint       =    GimbalYawPos;
  /**************************************�������ֵ***************************************/
	PidPitchAidSpeed.SetPoint = PID_Calc(&PidPitchAidPos,Gimbal.Pitch.MotorTransAngle);
	inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed,GyroReceive.GY));		
	PitchCurrent = (short)inttoshort[0];
	
	PidYawAidSpeed.SetPoint = PID_Calc(&PidYawAidPos,Gimbal.Yaw.Gyro);	
	PidYawAidSpeed.SetPoint = LIMIT_MAX_MIN(PidYawAidSpeed.SetPoint,speed_limit,-speed_limit);
	inttoshort[1] = PID_Calc(&PidYawAidSpeed,GyroReceive.GZ);
	YawCurrent = inttoshort[1];
}


/**********************************************************************************************************
*�� �� ��: Gimbal_Buff_Cal
*����˵��: ��̨���ģʽ
*��    ��: rc  mouse  Pc_RecvData
*�� �� ֵ: ��
**********************************************************************************************************/
void Gimbal_Buff_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv)
{
	if(GimbalAct_Init_Flag!=Gimbal_Buff_Mode)
	{
		GimbalAct_Init_Flag=Gimbal_Buff_Mode;
		GimbalYawPos = Gimbal.Yaw.MotorTransAngle;
	  GimbalPitchPos=Gimbal.Pitch.MotorTransAngle;
		Pc_Recv->RCPitch=Gimbal.Pitch.MotorTransAngle;
		Pc_Recv->RCYaw = Gimbal.Yaw.MotorTransAngle;
	}
	/**�����������***/
	Recent_Pitch_Angle_Armor = Pc_Recv->RCPitch;
	while(ABS(Recent_Pitch_Angle_Armor) >= 90 )
	{
		if(Recent_Pitch_Angle_Armor<0)
			 Recent_Pitch_Angle_Armor=Recent_Pitch_Angle_Armor+180;  
		else
			 Recent_Pitch_Angle_Armor=-(Recent_Pitch_Angle_Armor-180);
	}
	
	
	Recent_Yaw_Angle_Armor = Pc_Recv->RCYaw;
	while(ABS(Gimbal.Yaw.MotorTransAngle - Recent_Yaw_Angle_Armor) >= 180 )
	{
		if(Gimbal.Yaw.MotorTransAngle<0)
			 Recent_Yaw_Angle_Armor=Recent_Yaw_Angle_Armor-360;  
		else
			 Recent_Yaw_Angle_Armor=Recent_Yaw_Angle_Armor+360;
	}
	
	GimbalPitchPos= Recent_Pitch_Angle_Armor;
	GimbalYawPos = Recent_Yaw_Angle_Armor;
	
	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,gimbal_pitch_max,gimbal_pitch_min);//��λ
	/***************************************************************************************/
	PidPitchAidPos.SetPoint  =    GimbalPitchPos;
	PidYawAidPos.SetPoint       =    GimbalYawPos;
  /**************************************�������ֵ***************************************/
	PidPitchAidSpeed.SetPoint = PID_Calc(&PidPitchAidPos,Gimbal.Pitch.MotorTransAngle);
	inttoshort[0] = -(PID_Calc(&PidPitchAidSpeed,GyroReceive.GY));		
	PitchCurrent = (short)inttoshort[0];
	
	PidYawAidSpeed.SetPoint = PID_Calc(&PidYawAidPos,Gimbal.Yaw.MotorTransAngle);	
	PidYawAidSpeed.SetPoint = LIMIT_MAX_MIN(PidYawAidSpeed.SetPoint,speed_limit,-speed_limit);
	inttoshort[1] = PID_Calc(&PidYawAidSpeed,GyroReceive.GZ);
	YawCurrent = inttoshort[1];
}

/**********************************************************************************************************
*�� �� ��: Gimbal_DropShot_Cal
*����˵��: ��̨����ģʽ
*��    ��: rc  mouse  Pc_RecvData
*�� �� ֵ: ��
**********************************************************************************************************/

void Gimbal_DropShot_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv)
{
	if( GimbalAct_Init_Flag!=Gimbal_DropShot_Mode)
	{
		Laser_On();
		GimbalAct_Init_Flag=Gimbal_DropShot_Mode;
		GimbalYawPos = Gimbal.Yaw.MotorTransAngle;//����ģʽ
		GimbalPitchPos = Gimbal.Pitch.MotorTransAngle;
	}
	
	if(Status.ControlMode==Control_RC_Mode)//Rc_Control
	{
	  GimbalYawPos   += (1024-rc.ch2)*0.0005f;
		GimbalPitchPos -= (1024-rc.ch3)*0.0005f;//��������
	}
	if(Status.ControlMode==Control_MouseKey_Mode)//Mouse_Key
	{
    GimbalPitchPos -= mouse.y*0.005f;
    GimbalYawPos   -= mouse.x*0.005f;
	}
	
	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,gimbal_pitch_max,gimbal_pitch_min);//��λ(�õ���Ƕ�)
  /***********************************************************************************/
	PidPitchPos.SetPoint = GimbalPitchPos;
	PidYawPos.SetPoint = GimbalYawPos;
  /**************************************�������ֵ**************************************/
	PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,Gimbal.Pitch.MotorTransAngle);  
	inttoshort[0]=-(PID_Calc(&PidPitchSpeed,GyroReceive.GY));//��������
	PitchCurrent=(short)inttoshort[0];
	
	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.MotorTransAngle);	
	PidYawSpeed.SetPoint=LIMIT_MAX_MIN(PidYawSpeed.SetPoint,5.5f,-5.5f);    
	inttoshort[1]=PID_Calc(&PidYawSpeed,GyroReceive.GZ);
	YawCurrent=inttoshort[1];	
}

/**********************************************************************************************************
*�� �� ��: Gimbal_SI_Cal
*����˵��: ��̨ϵͳ��ʶģʽ
*��    ��: rc  mouse  Pc_RecvData
*�� �� ֵ: ��
**********************************************************************************************************/
short F_Change_flag = 0;		//�л�Ƶ�ʱ�־
float	Gimbal_direct;

int T;//����
int T_cnt = 0;//����
int T_Time_cnt = 0;//���ڴ�������
int F_cnt = 0,F_cnt_last;//ָ��F��ָ��
float F = 1;

void Gimbal_SI_Cal(float Gimbal_pitch,float Gimbal_yaw)
{
	if(GimbalAct_Init_Flag!=Gimbal_SI_Mode)
	{
		GimbalPitchPos = 0;				
		GimbalYawPos = Gimbal.Yaw.Gyro;
		F = 1;		
		GimbalAct_Init_Flag = Gimbal_SI_Mode;
	}
		
	T_change();		
	
	if(Gimbal_yaw != 0&&Gimbal_pitch == 0)
	{
		GimbalYawPos = Gimbal_direct*Gimbal_yaw;
	}
	else if(Gimbal_yaw == 0&&Gimbal_pitch != 0)
	{
		GimbalPitchPos = LIMIT_MAX_MIN((Gimbal_direct*Gimbal_pitch),gimbal_pitch_max,gimbal_pitch_min);
	}
	
//	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,gimbal_pitch_max,gimbal_pitch_min);//��λ(�õ���Ƕ�)
  /***********************************************************************************/
	PidPitchPos.SetPoint = GimbalPitchPos;
	PidYawPos.SetPoint = GimbalYawPos;
  /**************************************�������ֵ**************************************/	
//	PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,Gimbal.Pitch.MotorTransAngle);  		//���ֶ�����������һ��
	PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,Gimbal.Pitch.MotorTransAngle);  		//���ֶ�����������һ��
	inttoshort[0]=-(PID_Calc(&PidPitchSpeed,GyroReceive.GY));//��������
	PitchCurrent=(short)inttoshort[0];
	
//	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.MotorTransAngle);	
	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.Gyro);	
	PidYawSpeed.SetPoint=LIMIT_MAX_MIN(PidYawSpeed.SetPoint,5.5f,-5.5f);    
	inttoshort[1]=PID_Calc(&PidYawSpeed,GyroReceive.GZ);
	YawCurrent=inttoshort[1];	
}

void get_F(void)
{
	static int j = 0;
	if(F < 22)
	{
		j++;
		F += 0.5f;
	}
	else if(F == 22)
	{
		F = 24;
	}
	else if(F >= 24&&F < 40)
	{
		F = F + 2;
	}
	else if(F == 40)
	{
		F = 50;
	}
	else if(F >= 50&&F < 120)
	{
		F = F + 10;
	}
	else if(F == 120)
	{
		F = 200;
	}
	else if(F == 250)
	{
		F = 333;
	}
	else if(F == 333)
	{
		F = 500;
	}
}

void T_change(void)
{
	T = round(1000/F);
	if(GimbalAct_Init_Flag == Gimbal_SI_Mode)
	{
		if(F_Change_flag == 0)
		{
			Gimbal_direct = sin(2*3.14*T_cnt/T);
			if(T_cnt >= T)
			{
				T_cnt = 0;
				T_Time_cnt++;
			}
			T_cnt++;
			F_cnt_last = F_cnt;

			if(T_Time_cnt >= 10 && F <22)
			{
				T_Time_cnt = 0;
				F_cnt++; 
			}
			else if(T_Time_cnt >= 10 && F >= 22 && F <50)
			{
				T_Time_cnt = 0;
				F_cnt++; 
			}
			else if(T_Time_cnt >= 20 && F >= 50)
			{
				T_Time_cnt = 0;
				F_cnt++; 
			}
			if(F_cnt != F_cnt_last)		//Ƶ�ʸı�
			{
				F_Change_flag  = 1;
			}
		}
		else if(F_Change_flag == 1)
		{
			Gimbal_direct = 0;
			if((ABS(PidPitchPos.SetPoint - PidPitchPos.ActualValue) < 0.2f) || (ABS(PidYawPos.SetPoint - PidYawPos.ActualValue) < 0.2f))		//�ص���ʼλ��
			{
				get_F();
				T_cnt = 0;
				F_Change_flag = 0;
			}
		}
	}
	else if(GimbalAct_Init_Flag != Gimbal_SI_Mode)
	{
		F = 1;
	}
}


/**********************************************************************************************************
*�� �� ��: Gimbal_CurrentPid_Cal
*����˵��: ���͵���ֵ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Gimbal_CurrentPid_Cal(void)
{
	switch (Status.GimbalMode)
	{
		case Gimbal_Powerdown_Mode:
			Gimbal_Powerdown_Cal();
			break;
		case Gimbal_Act_Mode:
			Gimbal_Act_Cal(RC_Ctl.rc,RC_Ctl.mouse,&PC_Receive);
			break;  
		case Gimbal_Armor_Mode:
			Gimbal_Armor_Cal(RC_Ctl.rc,RC_Ctl.mouse,&PC_Receive);
			break;
		case Gimbal_Buff_Mode:
			Gimbal_Buff_Cal(RC_Ctl.rc,RC_Ctl.mouse,&PC_Receive);
			break;
		case Gimbal_DropShot_Mode:
			Gimbal_DropShot_Cal(RC_Ctl.rc,RC_Ctl.mouse,&PC_Receive);
			break; 
		case Gimbal_SI_Mode:
			Gimbal_SI_Cal(10.0, 0.0);			//pitch
//			Gimbal_SI_Cal(0.0, 20.0);		//yaw
			break;
		default:
			break;
	}
	F405.Gimbal_Flag = Status.GimbalMode;
	GimbalCan2Send(PitchCurrent,YawCurrent);
}

/**********************************************************************************************************
*�� �� ��: Pid_PitchGyroPosSpeed
*����˵��: Pitch��˫��PID
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Pid_PitchGyroPosSpeed_Init(void)
{
	//����ϵͳ
	PidPitchPos.P=0.3f;
	PidPitchPos.I=0.0f;
	PidPitchPos.D=0.5f;
	PidPitchPos.IMax=0.0f;
	PidPitchPos.SetPoint=0.0f;
	PidPitchPos.OutMax=20.0f;
	PidPitchSpeed.P=30000.0f;
	PidPitchSpeed.I=20.0f; 
	PidPitchSpeed.D=0.0f;
	PidPitchSpeed.IMax=450.0f;
	PidPitchSpeed.SetPoint=0.0f;
	PidPitchSpeed.OutMax=30000.0f;
}

/**********************************************************************************************************
*�� �� ��: Pid_YawGyroPosSpeed
*����˵��: Yaw��˫��PID
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Pid_YawGyroPosSpeed_Init(void)
{
	//����ϵͳ(��������)
	PidYawPos.P=0.255f;
	PidYawPos.I=0.0;
	PidYawPos.D=0.5f;
	PidYawPos.IMax=0.0f;
	PidYawPos.SetPoint=0.0f;
	PidYawPos.OutMax=5.5f;
	PidYawSpeed.P=30000.0f;
	PidYawSpeed.I=10.0f;
	PidYawSpeed.D=0.0f;
	PidYawSpeed.IMax=2000.0f;
	PidYawSpeed.SetPoint=0.0f;
	PidYawSpeed.OutMax=30000.0f;
}

/**********************************************************************************************************
*�� �� ��: Pid_Pitch_MotorPos_GyroSpeed
*����˵��: Pitch�Ḩ��˫��PID
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Pid_PitchMotorPos_GyroSpeed_Init(void) 
{
	PidPitchAidPos.P=0.25f;				//0.35
	PidPitchAidPos.I=0.0015f;
	PidPitchAidPos.D=0.0f;
	PidPitchAidPos.IMax=50.0f;
	PidPitchAidPos.SetPoint=0.0f;
	PidPitchAidPos.OutMax=5.5f;
	PidPitchAidSpeed.P=22000.0f;	//18000
	PidPitchAidSpeed.I=0.0f; 
	PidPitchAidSpeed.D=0.0f;
	PidPitchAidSpeed.IMax=1000.0f;
	PidPitchAidSpeed.SetPoint=0.0f;
	PidPitchAidSpeed.OutMax=13000.0f;
}
/**********************************************************************************************************
*�� �� ��: Pid_Yaw_MotorPos_GyroSpeed
*����˵��: Yaw�Ḩ��˫��PID
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Pid_YawMotorPos_GyroSpeed_Init(void) 
{
	PidYawAidPos.P=0.32f;						//0.25
	PidYawAidPos.I=0.002f;			
	PidYawAidPos.D=0.0f;				
	PidYawAidPos.IMax=100.0f;
	PidYawAidPos.SetPoint=0.0f;
	PidYawAidPos.OutMax=5.5f;
	PidYawAidSpeed.P=25000.0f;			//12000
	PidYawAidSpeed.I=0.0f; 
	PidYawAidSpeed.D=0.0f;
	PidYawAidSpeed.IMax=100.0f;
	PidYawAidSpeed.SetPoint=0.0f;
	PidYawAidSpeed.OutMax=30000.0f;
}

/**********************************************************************************************************
*�� �� ��: Gimbal_task
*����˵��: ��̨������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t Gimbal_high_water;
UBaseType_t GGGG;
void Gimbal_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 1;
   while (1) {
    IWDG_Feed();//ι��
		ZeroCheck_cal();
		 
		Gimbal_CurrentPid_Cal();
		 
		GGGG=uxTaskGetStackHighWaterMark(NULL);
		
		xLastWakeTime = xTaskGetTickCount();
		
    vTaskDelayUntil(&xLastWakeTime,xFrequency); 

#if INCLUDE_uxTaskGetStackHighWaterMark
        Gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

