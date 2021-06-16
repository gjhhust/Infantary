/**********************************************************************************************************
 * @�ļ�     ChassisTask.c
 * @˵��     ���̿���+��������
 * @�汾  	 V3.0
 * @����     ��ҵȨ
 * @����     2021.4.26
**********************************************************************************************************/
#include "main.h"
/*----------------------------------�ڲ�����---------------------------*/
short WheelCurrentSend[4];
short Set_Jump[4] = {0};
float Current_Change[4] = {0};//����������
float Current_f[4] = {0};//�������f
float Flow[4] = {0};//ʵ�ʵ���f
float speed[4] = {0};//ʵ���ٶ�f

//��������ϵ�� PowerLimit
short Actual_P_max;						//ʵ�������
short Self_Protect_Limit;			//С����ת������
float k_BAT;
short Excess_P_max;
short CurrentMax;

const float k_chassis_t = 0.0168f;			
//24*0.0007 24��chassis�������ѹ �������24 һ���21��
//0.0007��������Ƶ���ֵ��Ӧ�ĵ�λchassis���������

/*----------------------------------�ṹ��-----------------------------*/
Pid_Typedef Pid_Current[4];
Pid_Typedef pidChassisWheelSpeed[4];
Pid_Typedef pidChassisPosition_Speed;
F105_Typedef F105;
Power_Typedef Power_method[METHOD_NUM];

/*----------------------------------�ⲿ����---------------------------*/
extern JudgeReceive_t JudgeReceive;
extern ChassisSpeed_t chassis;
extern RM820RReceive_Typedef ChassisMotorCanReceive[4];
extern F405_typedef F405;
extern enum POWERSTATE_Typedef PowerState;

extern float Input[4];
extern float Output[4];

/**********************************************************************************************************
*�� �� ��: Method_Check
*����˵��: ���ݲ�ͬ�Ĺ���ѡ��ͬ�Ŀ��Ʋ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Method_Check(void)
{
	short i;
	short choose_PN = 0;
	
	static short PN;
	static short last_PN;
	
	for(choose_PN = 1;choose_PN < METHOD_NUM; choose_PN++)		//0���ڴ���Ĭ�ϲ���
	{
		if(Power_method[choose_PN].Actual_P_max == JudgeReceive.MaxPower)
		{
			PN = choose_PN;
			break;
		}	
	}
	if(choose_PN >= METHOD_NUM)	//û�ҵ�ƥ�������ʲ���
		PN = PN_Default;	//Ĭ�ϲ���

	if(last_PN != choose_PN)			//˵�������б仯
	{
		Actual_P_max = Power_method[PN].Actual_P_max;
		Self_Protect_Limit = Power_method[PN].Self_Protect_Limit;
		k_BAT = Power_method[PN].k_BAT;
		Excess_P_max = Power_method[PN].Excess_P_max;
		CurrentMax = Power_method[PN].CurrentMax;
		for(i=0; i<4; i++)
		{
			pidChassisWheelSpeed[i].OutMax = CurrentMax;
		}
	}
	last_PN = PN;
}

/**********************************************************************************************************
*�� �� ��: Chassis_Speed_Cal
*����˵��: ����xyw���ٶȼ���Ŀ���ٶ�ֵ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
short test_Self_Protect_Limit = 3600;
float test_k_BAT = 1.0f;

void Chassis_Speed_Cal(void)
{
	static float k_xy = 3;
	static float k_CAP = 2.0f;
	static short carSpeedw = 0;
	static short Angular_Velocity;
	
	
//xy���ٶ���w���ٶ����
	switch(F405.Chassis_Flag)
	{
		case Chassis_Powerdown_Mode:
			k_xy = 0;
			carSpeedw = 0;
		break;
		
		case Chassis_Act_Mode:
			Angular_Velocity = 0.026f*(ChassisMotorCanReceive[0].RealSpeed+ChassisMotorCanReceive[1].RealSpeed+ChassisMotorCanReceive[2].RealSpeed+ChassisMotorCanReceive[3].RealSpeed);
			pidChassisPosition_Speed.SetPoint = -chassis.carSpeedw;
			carSpeedw = PID_Calc(&pidChassisPosition_Speed, Angular_Velocity); 

			if(chassis.carSpeedx !=0 && chassis.carSpeedy != 0)
				k_xy = 2;
			else
				k_xy = 3;
		break;
		
		case Chassis_SelfProtect_Mode:
			{
				if(chassis.carSpeedx !=0 || chassis.carSpeedy != 0)
				{
					if(chassis.carSpeedx !=0 && chassis.carSpeedy != 0)
						k_xy = 1.8f;
					else
						k_xy = 2.5f;
				}
				else
				{
					k_xy = 2.5f;
				}		
				if(PowerState == BAT)
				{
//				carSpeedw = LIMIT_MAX_MIN(chassis.carSpeedw, test_Self_Protect_Limit, -test_Self_Protect_Limit);
					carSpeedw = LIMIT_MAX_MIN(chassis.carSpeedw, Self_Protect_Limit, -Self_Protect_Limit);
				}
				else if(PowerState == CAP)
				{
					carSpeedw = chassis.carSpeedw;
				}
			}
		break;
			
		case Chassis_Solo_Mode:
		{
			if(chassis.carSpeedx !=0 && chassis.carSpeedy != 0)
				k_xy = 1.4f;
			else
				k_xy = 2;
			
			Angular_Velocity = 0.026f*(ChassisMotorCanReceive[0].RealSpeed+ChassisMotorCanReceive[1].RealSpeed+ChassisMotorCanReceive[2].RealSpeed+ChassisMotorCanReceive[3].RealSpeed);
			pidChassisPosition_Speed.SetPoint = -chassis.carSpeedw;
			carSpeedw = PID_Calc(&pidChassisPosition_Speed, Angular_Velocity); 
		}
		break;
		
		case Chassis_NoFollow_Mode:
			k_xy = 2.5f;
			carSpeedw = 0;
		break;
		
		default: 
			k_xy = 0;
			carSpeedw = 0;
			break;
	}
	
	
//���ݲ�ͬ���� ��Ӧ��ͬxy���ٶ�ϵ��
	if(PowerState == BAT)
	{
//		k_xy *= test_k_BAT;
		k_xy *= k_BAT;
	}
	else if(PowerState == CAP)
	{
		k_xy *= k_CAP;
	}
		
	pidChassisWheelSpeed[0].SetPoint= (k_xy*(+chassis.carSpeedx+chassis.carSpeedy)-carSpeedw);
	pidChassisWheelSpeed[1].SetPoint= (k_xy*(-chassis.carSpeedx+chassis.carSpeedy)-carSpeedw);
	pidChassisWheelSpeed[2].SetPoint= (k_xy*(+chassis.carSpeedx-chassis.carSpeedy)-carSpeedw);
	pidChassisWheelSpeed[3].SetPoint= (k_xy*(-chassis.carSpeedx-chassis.carSpeedy)-carSpeedw);	
}

/**********************************************************************************************************
*�� �� ��: PowerLimit
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
float test_W_Chassis_t = 0;	//���㹦������
//short test_Excess_P_max = 320;

//short test_speed_out[4];
//short test_current_out[4];
//short test_wheel_send[4];

void PowerLimit(void)
{
	short i = 0;
	short DescendFlag = 0;
	
	static float W_Chassis_t = 0;//���̹���(������)
	static float W_k = 0;
	
	static short EnergyMargin = 10;		//���еĻ�����������
	static float My_P_max;				//����ĵ�ǰ�����
	static float k_ExcessPower;
		
//���������
	if(JudgeReceive.remainEnergy <= EnergyMargin)
	{
		My_P_max = JudgeReceive.MaxPower*0.9f;
	}
	else
	{
		My_P_max = LIMIT_MAX_MIN(Excess_P_max*(JudgeReceive.remainEnergy-EnergyMargin)/(60-EnergyMargin), Excess_P_max, JudgeReceive.MaxPower);
//		My_P_max = LIMIT_MAX_MIN(Excess_P_max*(JudgeReceive.remainEnergy-EnergyMargin)/(60-EnergyMargin), test_Excess_P_max, JudgeReceive.MaxPower);
	}
	
//���յ����˲�
	Current_Filter_Excu();
	
	for(i = 0;i < 4;i ++)
	{
		Pid_Current[i].SetPoint = PID_Calc(&pidChassisWheelSpeed[i],ChassisMotorCanReceive[i].RealSpeed);
		Current_Set_Jump();
		Current_Change[i] = PID_Calc(&Pid_Current[i],Flow[i]);
		
//		test_speed_out[i] = Pid_Current[i].SetPoint;
//		test_current_out[i] = Current_Change[i];
		
		if(Set_Jump[i] == 0)
		{
			Current_f[i] += Current_Change[i];
		}
		else if(Set_Jump[i] == 1)
		{
			Current_f[i] = Pid_Current[i].SetPoint;
		}
		WheelCurrentSend[i] = Current_f[i];	
		
//���㵱ǰ����
		W_Chassis_t += ABS(Pid_Current[i].SetPoint)*k_chassis_t;//���ʼ���20*24/16384 = 0.029
	}
	test_W_Chassis_t = W_Chassis_t;	

//����ǰ���ʳ�������� ��7�������ٶ�
	while(W_Chassis_t > My_P_max && DescendFlag < 7)//flag < 7
	{
		W_k = 0.95f;
		W_Chassis_t = 0;
		
		for(i=0;i<4;i++)//ͨ�������ٶȼ�С����ֵ
		{
			pidChassisWheelSpeed[i].SetPoint *= W_k;
			
			//�ٶȻ�+������
			Pid_Current[i].SetPoint = PID_Calc(&pidChassisWheelSpeed[i],ChassisMotorCanReceive[i].RealSpeed);
			Current_Filter_Excu();
			Current_Set_Jump();
			Current_Change[i] = PID_Calc(&Pid_Current[i],Flow[i]);
			if(Set_Jump[i] == 0)
			{
				Current_f[i] += Current_Change[i];
			}
			else if(Set_Jump[i] == 1)
			{
				Current_f[i] = Pid_Current[i].SetPoint;
			}
			WheelCurrentSend[i] = Current_f[i];
			
			W_Chassis_t += ABS(Pid_Current[i].SetPoint)*k_chassis_t;
		}	
		test_W_Chassis_t = W_Chassis_t;
		DescendFlag++;
	}
	
//������������������Ĺ����Գ�����ֱ����������	
	if(W_Chassis_t > My_P_max)//��������������ֱ����������
	{
			k_ExcessPower = My_P_max/W_Chassis_t;//������ϵ��
			for(i=0;i<4;i++)
			{
				WheelCurrentSend[i] *= k_ExcessPower;
				W_Chassis_t += ABS(WheelCurrentSend[i])*k_chassis_t;
			}
			test_W_Chassis_t = W_Chassis_t;
		}
}

/**********************************************************************************************************
*�� �� ��: Chassis_CurrentPid_Cal
*����˵��: ���̲���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_CurrentPid_Cal(void)
{
	int i=0;
	Method_Check();			//���õ�ǰ���ʲ���
	Chassis_Speed_Cal();//����xyw���ٶȼ���Ŀ���ٶ�ֵ
	if(PowerState == BAT)
	{
		PowerLimit();
	}
	else if(PowerState == CAP)
	{
		for(i=0;i<4;i++)
		{
			WheelCurrentSend[i] = PID_Calc(&pidChassisWheelSpeed[i],ChassisMotorCanReceive[i].RealSpeed);
		}
	}
	else
	{
		for(i=0;i<4;i++)
		{
			WheelCurrentSend[i] = 0;
		}
	}
	//���͵���ֵ����������
}

/**********************************************************************************************************
*�� �� ��: Current_Filter_Excu
*����˵��: ���ĸ����ӵĵ�������ֵ�ֱ��˲�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Current_Filter_Excu(void)
{
	for(int i = 0;i < 4;i++)
	{
		Input[i] = (float)ChassisMotorCanReceive[i].Current; 
	}
	Fir(Input,Output);
}

/**********************************************************************************************************
*�� �� ��: Current_Set_Jump
*����˵��: �ж��Ƿ��õ�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
//short Current_Set_Flag;			//�ĸ����Ӷ����õ�����ʱ�����õ�����
//210513 �ֱ���Կ������� Ч������
void Current_Set_Jump(void)
{
	int i;
	for(i = 0;i < 4;i ++)
	{
		if(F405.Chassis_Flag == Chassis_Act_Mode)		//����ģʽ
		{
			if(ABS(Pid_Current[i].SetPoint - Pid_Current[i].SetPointLast) > 3000)
				Set_Jump[i] = 0;
			else
				Set_Jump[i] = 1;
		}
		else
			Set_Jump[i] = 1;
	}

}
/**********************************************************************************************************
*�� �� ��: Chassis_Power_Control_Init
*����˵��: ���̹������Ʋ�����ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_Power_Control_Init(void)
{
	int num = 0;
	/****************Ĭ�ϲ���********************/
	Power_method[num].Actual_P_max = 40;
	Power_method[num].Self_Protect_Limit = 3000;
	Power_method[num].k_BAT = 0.8f;
	Power_method[num].Excess_P_max = 300;
	Power_method[num].CurrentMax = 8000;
	/****************40W********************/
	num++;
	Power_method[num].Actual_P_max = 40;
	Power_method[num].Self_Protect_Limit = 3000;
	Power_method[num].k_BAT = 0.8f;
	Power_method[num].Excess_P_max = 300;
	Power_method[num].CurrentMax = 8000;
	/****************45W********************/
	num++;
	Power_method[num].Actual_P_max = 45;
	Power_method[num].Self_Protect_Limit = 3000;
	Power_method[num].k_BAT = 0.9f;
	Power_method[num].Excess_P_max = 300;
	Power_method[num].CurrentMax = 8000;
	/****************50W********************/
	num++;
	Power_method[num].Actual_P_max = 50;
	Power_method[num].Self_Protect_Limit = 4000;
	Power_method[num].k_BAT = 1.0f;
	Power_method[num].Excess_P_max = 350;
	Power_method[num].CurrentMax = 8000;
	/****************60W********************/
	num++;
	Power_method[num].Actual_P_max = 60;
	Power_method[num].Self_Protect_Limit = 5400;
	Power_method[num].k_BAT = 1.3f;
	Power_method[num].Excess_P_max = 400;
	Power_method[num].CurrentMax = 8000;
	/****************80W********************/
	num++;
	Power_method[num].Actual_P_max = 80;
	Power_method[num].Self_Protect_Limit = 7000;
	Power_method[num].k_BAT = 1.7f;
	Power_method[num].Excess_P_max = 450;
	Power_method[num].CurrentMax = 9000;
	/****************100W********************/
	num++;
	Power_method[num].Actual_P_max = 100;
	Power_method[num].Self_Protect_Limit = 8400;
	Power_method[num].k_BAT = 2.0f;
	Power_method[num].Excess_P_max = 500;
	Power_method[num].CurrentMax = 10000;
	/****************120W********************/
	num++;
	Power_method[num].Actual_P_max = 120;
	Power_method[num].Self_Protect_Limit = 11000;
	Power_method[num].k_BAT = 2.3f;
	Power_method[num].Excess_P_max = 600;
	Power_method[num].CurrentMax = 10000;
}

/**********************************************************************************************************
*�� �� ��: Pid_ChassisWheelInit
*����˵��: ����XY���˶�PID������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Pid_ChassisWheelInit(void)
{
	short i=0;
	
	for(i = 0;i < 4;i ++)
	{
		//������
		Pid_Current[i].P = 0.16f;
		Pid_Current[i].I = 0.0f;
		Pid_Current[i].D = 0.0f;
		Pid_Current[i].IMax = 2500;//2500
		Pid_Current[i].SetPoint = 0.0f;
		Pid_Current[i].OutMax = 8000.0f;	//8000.0f
		
		//�ٶȻ�
		pidChassisWheelSpeed[i].P = 5.0f;
		pidChassisWheelSpeed[i].I = 0;
		pidChassisWheelSpeed[i].D = 0;
		pidChassisWheelSpeed[i].ErrorMax = 1000.0f;
		pidChassisWheelSpeed[i].IMax = 4000.0f;
		pidChassisWheelSpeed[i].SetPoint = 0.0f;	
		pidChassisWheelSpeed[i].OutMax = 8000.0f;	

		//�����ٶȻ�
		pidChassisPosition_Speed.P = 3.0f;//w���ٶȻ�
		pidChassisPosition_Speed.I = 0.0f;
		pidChassisPosition_Speed.D = 0.0f;
		pidChassisPosition_Speed.IMax = 200.0f;
		pidChassisPosition_Speed.OutMax = 9000.0f;//8000.0f
	}
}

/**********************************************************************************************************
*�� �� ��: HeatControl
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
//��������
extern JudgeReceive_t JudgeReceive;

uint16_t HeatMax17, HeatCool17;
const short BulletHeat17 = 10;

short CurHeat17, LastHeat17, AvailableHeat17; //��ǰ������ ��һ������, ���м�������

uint16_t Shooted17Cnt;	//һ�������Ѵ���ӵ���
uint16_t AvailableBullet17;	//��һ�����������

char ShootAbleFlag;

void HeatControl(void)
{
	if(JudgeReceive.HeatUpdateFlag == 1)	//��������
	{
		Shooted17Cnt = 0;
		AvailableHeat17 = LIMIT_MAX_MIN(HeatMax17 - CurHeat17 + HeatCool17,HeatMax17,0);
//		AvailableHeat17 = HeatMax17 - CurHeat17;
		if(JudgeReceive.ShootCpltFlag == 1)	//��⵽������Ϊ�������º������ӵ�
		{
			AvailableHeat17 = LIMIT_MAX_MIN(AvailableHeat17 - BulletHeat17,HeatMax17,0);
			JudgeReceive.ShootCpltFlag = 0;	//�Ѵ����걾�η���
		}
		AvailableBullet17 = AvailableHeat17 / BulletHeat17;
		ShootAbleFlag = (AvailableBullet17 < 1)?0:1;		
	}	
	
	else if((JudgeReceive.ShootCpltFlag == 1) && (JudgeReceive.HeatUpdateFlag == 0))	//����û�и��£�����⵽����
	{
		JudgeReceive.ShootCpltFlag = 0;		//�Ѵ����걾�η���
		Shooted17Cnt++;		//������һ���ӵ�
		ShootAbleFlag = (Shooted17Cnt >= AvailableBullet17)?0:1;		
	}
}

/**********************************************************************************************************
*�� �� ��: HeatUpdate
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
const float HeatControlThreshold = 0.8f;   	//�����������Ƶ���ֵ

void HeatUpdate(void)
{
//	HeatMax17 = JudgeReceive.HeatMax17 + (short)(1250/JudgeReceive.maxHP) - BulletHeat17;		//ե������.jpg
	HeatMax17 = JudgeReceive.HeatMax17 - BulletHeat17;		//ե������.jpg
	HeatCool17 = JudgeReceive.HeatCool17/10;
	CurHeat17 = JudgeReceive.shooterHeat17;
	
	if(CurHeat17 != LastHeat17)
	{
		JudgeReceive.HeatUpdateFlag = 1;
		JudgeReceive.ShootCpltFlag = 0;			//���������򽫷����־λ����(û�д�����Ĵ�)
	}
	
	if(CurHeat17 < HeatControlThreshold*HeatMax17)
	{
		ShootAbleFlag = 1;
		JudgeReceive.ShootCpltFlag = 0;
	}
	else
	{
		if((JudgeReceive.ShootCpltFlag == 1) || (JudgeReceive.HeatUpdateFlag == 1))
		HeatControl();
	}
	
	JudgeReceive.HeatUpdateFlag = 0;		//�Ѵ����걾����������
	LastHeat17 = CurHeat17;
	F105.IsShootAble = ShootAbleFlag;
}

/**********************************************************************************************************
*�� �� ��: BuildF105
*����˵��: ����Ҫ�����ϲ���F105�ṹ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void BuildF105(void)
{
	if(JudgeReceive.robot_id < 10)
		F105.RobotRed = 1;
	else
		F105.RobotRed = 0;			//0Ϊ��ɫ��1Ϊ��ɫ
	switch(JudgeReceive.BulletSpeedMax17)
	{
		case 15:
		{
			F105.BulletSpeedLevel = 0;
			break;
		}
		case 18:
		{
			F105.BulletSpeedLevel = 1;
			break;
		}
		case 30:
		{
			F105.BulletSpeedLevel = 2;
			break;
		}
		default:
		{
			F105.BulletSpeedLevel = 0;
			break;
		}
	}
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
  portTickType xLastWakeTime;
	const portTickType xFrequency = 1;
	
  while (1) {
    
		//���ݳ�ŵ����
		if(JudgeReceive.remainEnergy < 30)
			Charge_Off;
		else
			Charge_On;
		
		//��������
    Chassis_CurrentPid_Cal();
		ChassisCan1Send(WheelCurrentSend[0],WheelCurrentSend[1],WheelCurrentSend[2],WheelCurrentSend[3]); 
		
		//��������
		HeatUpdate();
		BuildF105();
		Can2Send0(&F105);
		
    vTaskDelay(1); 
		 
		xLastWakeTime = xTaskGetTickCount();
				
		vTaskDelayUntil(&xLastWakeTime,xFrequency); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
