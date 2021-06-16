/**********************************************************************************************************
 * @�ļ�     ShootTask.c
 * @˵��     ��������
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2020.1
**********************************************************************************************************/
# include "main.h"
/*----------------------------------�ڲ�����---------------------------*/
short MirocPosition;
short FrictionWheel_speed=0,BulletSpeed,BodanMotorCurrent,ShootAct_Init_Flag;
short BodanMotorCurrent;
int SendToTx2BullectCnt;
int Shoot_Init_flag = 0;
/*----------------------------------�ṹ��------------------------------*/
Pid_Typedef PidBodanMotorPos,PidBodanMotorSpeed;
/*----------------------------------�ⲿ����---------------------------*/
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
*�� �� ��: FrictionSpeedChoose
*����˵��: Ħ����ת��ѡ��
*��    ��: flag
*�� �� ֵ: ��
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
*�� �� ��: Shoot_Fire_Cal
*����˵��: ����ģʽ
*��    ��: ��
*�� �� ֵ: ��
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
*�� �� ��: Shoot_Check_Cal
*����˵��: ��¼ģʽ[�򿪼��⣬Ħ����/��Ƶ3~4��]
*��    ��: rc
*�� �� ֵ: ��
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
*�� �� ��: Shoot_Test_Cal
*����˵��: ����ģʽ�����ڸ�����λ������
*��    ��: 
*�� �� ֵ: ��
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
*�� �� ��: Shoot_Powerdown_Cal
*����˵��: ����ģʽ[΢������/Ħ����/����ת��]
*��    ��: rc
*�� �� ֵ: ��
**********************************************************************************************************/
void Shoot_Powerdown_Cal(void)
{
	if(ShootAct_Init_Flag!=4)
	  ShootAct_Init_Flag=4;
	PidBodanMotorPos.SetPoint=0;
	PidBodanMotorPos.SetPoint=Bodan_Pos;
}

/**********************************************************************************************************
*�� �� ��: Shoot_Tx2_Cal
*����˵��: ����ģʽ
*��    ��: rc
*�� �� ֵ: ��
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
			if(ABS(PC_Receive.RCPitch - Gimbal.Pitch.MotorTransAngle)<1.5f && ABS(PC_Receive.RCYaw - Gimbal.Yaw.Gyro)<1.5f)	//�Ѿ����鵽λ���Զ�����
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
			if(ABS(PC_Receive.RCPitch - Gimbal.Pitch.MotorTransAngle)<1.5f && ABS(PC_Receive.RCYaw - Gimbal.Yaw.MotorTransAngle)<1.5f)	//�Ѿ����鵽λ���Զ�����
			{
				if(ABS(PidBodanMotorPos.SetPoint-Bodan_Pos)<8000)
				{
					MirocPosition = k_onegrid * Onegrid;
					SendToTx2BullectCnt++;
				}
			}
    }
	}	
//����
//		if(armor_state == ARMOR_AIMED)
//  	{
//			if(ABS(PC_Receive.RCPitch - Gimbal.Pitch.MotorTransAngle)<1.5f && ABS(PC_Receive.RCYaw - Gimbal.Yaw.MotorTransAngle)<1.5f)	//�Ѿ����鵽λ���Զ�����
//			{
//				if(ABS(PidBodanMotorPos.SetPoint-Bodan_Pos)<8000)
//				{
//					MirocPosition = k_onegrid * Onegrid;
//					SendToTx2BullectCnt++;
//				}
//			}
//    }
//	}
	/*�����Ʒ���ʱ����*/
//	if(Status.GimbalMode == Gimbal_Buff_Mode)
//	{
//		if(F105.IsShootAble==1)
//  	{
//			if(ABS(PC_Receive.RCPitch - Gimbal.Pitch.MotorTransAngle)<0.8f && ABS(PC_Receive.RCYaw - Gimbal.Yaw.Gyro)<0.8f)	//���ҵ�Ŀ�겢����һ����Χ��
//			{
//				if(ABS(PidBodanMotorPos.SetPoint-Bodan_Pos)<8000)
//				{
//					MirocPosition = k_onegrid * Onegrid;
//					SendToTx2BullectCnt++;
//				}
//			}
//    }
//	}
	
//	/*���Ʒ���ʱ����*/
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
*�� �� ��: BodanMotor_CurrentPid_Cal
*����˵��: �������λ�û�˫���������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void BodanMotor_CurrentPid_Cal(void)
{
	switch(Status.ShootMode)//���ģʽѡ��
	{
		case Shoot_Check_Mode:
			Shoot_Check_Cal();
			break;
		case Shoot_Fire_Mode:
			Shoot_Fire_Cal();
			break;
		case Shoot_Tx2_Mode:
			Shoot_Test_Cal();			//210424����
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
*�� �� ��: Pid_BodanMotor
*����˵��: �������λ���ٶȻ�pid������ʼ��
*��    ��: ��
*�� �� ֵ: ��
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
*�� �� ��: Shoot_task
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
#define POWER_ON 1
#define POWER_OFF 0
uint32_t Shoot_high_water;
int Shoot_Power = 0;		//��ȡ���������ѹ
int test_shoot_power;
int init_time = 0;
short SHOOT_HADBE_INIT = 0;	//�Ѿ��ϵ���ʼ�����ı�־
void Shoot_task(void *pvParameters)
{
   static long init_time;
	 static short SHOOT_HADBE_INIT;	//�Ѿ��ϵ���ʼ�����ı�־
   while (1) {
		 Shoot_Power = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
		 if(Shoot_Power == POWER_ON)
		 {
			if(init_time>5000)		
			{
				FrictionSpeedChoose();
				Shoot_Init_flag = 1;
				SHOOT_HADBE_INIT = 1;		//ֻҪ��ʼ����һ�ξͻᱻ��ǣ������
			}
			else		//��ʼ��
			{
				Shoot_Init_flag = 0;
				FrictionWheel_speed = Init_FritionSpeed;
			}
			if(Status.ChassisMode == Chassis_Powerdown_Mode)
			{
				init_time = 0;
				Shoot_Init_flag = 0;				//��ֹ����ת
			}else if(Status.ChassisMode != Chassis_Powerdown_Mode && SHOOT_HADBE_INIT == 1)		//�Ѿ���ʼ���ɹ�һ���ˣ������л��˷ǵ���ģʽ
			{
				init_time = 5000;
			}
			init_time++;
			FrictionWheel_Set(FrictionWheel_speed);
		 }
		 else if(Shoot_Power == POWER_OFF)
		 {
				Shoot_Init_flag = 0;				//��ֹ����ת����
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
