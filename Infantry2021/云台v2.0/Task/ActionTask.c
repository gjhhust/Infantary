/**********************************************************************************************************
 * @�ļ�     ActionTask.c
 * @˵��     ��������
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2020.1
**********************************************************************************************************/
#include "main.h"
/*--------------�ڲ�����-------------------*/
short Mouse_Key_Flag;
short waitFlag[10]={10,0,0,0,0,0,0,0,0,0};
short SpeedMode = 1;//Ħ�������ٵ�λ
short DanCang_Flag;//���տ��ر�־λ
short ctrl_rising_flag,pre_key_ctrl,v_rising_flag,pre_key_v,c_rising_flag,pre_key_c,pre_key_e,e_rising_flag,x_rising_flag,pre_key_x,Press_Key_x_Flag,v_rising_flag,pre_key_b;
short q_rising_flag,b_rising_flag,f_rising_flag,g_rising_flag,pre_key_f,mouse_Press_r_rising_flag,pre_mouse_l,r_rising_flag,pre_key_r,z_rising_flag,pre_key_z,Press_Key_z_Flag;
short pre_key_q,pre_key_g,pre_v_rising_flag;	//��һv��������״̬
short v_high_flag;

short Turning_flag;	//С���ݱ�־
short MicroSw_flag;	//�����־
short Graphic_Init_flag;	//ͼ�γ�ʼ����־
short NoFollow_flag, Solo_flag, Buff_flag; 
/*������tx2�ػ���־�ͼ�ʱ*/
short PC_Sendflag = 0;
short Tx2_Off_times = 0;

//���ָǶ�� ����ʱ��
short SteeringEngineDelay = 0;
/*---------------�ṹ��--------------------*/
Status_t Status;
/*--------------�ⲿ����-------------------*/
extern RC_Ctl_t RC_Ctl; 
extern unsigned char magazineState;
extern Pid_Typedef PidBodanMotorPos,PidBodanMotorSpeed; 
extern RobotInit_Struct Infantry;
extern F105_Typedef F105;
extern F405_typedef F405;
extern PC_Receive_t PC_Receive;
extern int Bodan_Pos;
extern short MirocPosition;//���Ʋ�����ת
extern short FrictionWheel_speed;
extern RobotInit_Struct Infantry;
/**********************************************************************************************************
*�� �� ��: Status_Act
*����˵��: �����������������ֵ��̣���̨�����������
					 ���̣� 1)  Chassis_Act_Cal(Remote rc,Key key)                                 ��������ģʽ
									2)  Chassis_SelfProtect_Cal(Remote rc,Key key)                         ���ұ���ģʽ��С���ݣ�
									3)	Chassis_Solo_Cal(Remote rc,Key key)																 ����ģʽ
									4)  Chassis_Powerdown_Cal()                                            ��������ģʽ   
									
					 ��̨�� 1)  Gimbal_Act_Cal(Remote rc,Mouse mouse,PC_Recv_t *Pc_RecvData)       ��̨����ģʽ
									2)  Gimbal_Armor_Cal(PC_Recv_t *Pc_Recv, RC_Ctl_t *RC_clt)             �������ģʽ
									3)  Gimbal_ShenFu_Cal(PC_Recv_t *Pc_Recv, RC_Ctl_t *RC_clt)            �������ģʽ
									4)  Gimbal_Powerdown_Cal()                                             ��̨����ģʽ 
									
					 ���   1)  Shoot_Check_Cal();                                                 ��¼ģʽʹ��
					        2)  Shoot_Fire_Cal();                                                  �������ģʽ
									3)  Shoot_Armor_Cal();                                                 �������ģʽ
									4)  Shoot_ShenFu_Cal();                                                ��������ģʽ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Status_Act(void)
{
	SetInputMode(RC_Ctl.rc);
	switch(Status.ControlMode)
	{
		case Control_RC_Mode:
			   Remote_Process(RC_Ctl.rc);
			break;
		case Control_MouseKey_Mode:
		     Mouse_Key_Process(RC_Ctl);
			break;
		case Control_Powerdown_Mode:
			   Powerdown_Process();
			break;
		default:
			break;
	}
	
	if(Mouse_Key_Flag == 3)
	{
    MouseKey_Act_Cal(RC_Ctl);
	}
	
	F405Can1Send(&F405);//����405��Ϣ
}

/**********************************************************************************************************
*�� �� ��: SetInputMode
*����˵��: ����ģʽѡ��
*��    ��: rc
*�� �� ֵ: ��
**********************************************************************************************************/
void SetInputMode(Remote rc)
{
	switch(rc.s1)
	{
		case 1:
			Status.ControlMode=Control_RC_Mode;
		break;
		case 3:
			Status.ControlMode=Control_MouseKey_Mode;
		break;
		case 2:
			Status.ControlMode=Control_Powerdown_Mode;
		break;
	}
	Tx2_Off_Test(rc);	//ң��������tx2�ػ��ж�
}

/**********************************************************************************************************
*�� �� ��: Powerdown_Process
*����˵��: �ϵ�ģʽ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Powerdown_Process()
{
	if(Mouse_Key_Flag!=1)
	  Mouse_Key_Flag=1;
	Status.ChassisMode = Chassis_Powerdown_Mode;
	Status.GimbalMode  = Gimbal_Powerdown_Mode;
	Status.ShootMode = Shoot_Powerdown_Mode;
}

/**********************************************************************************************************
*�� �� ��: Tx2_Off_Test
*����˵��: Tx2�ػ�ָ���жϣ����Ҳ��˲������£�������ģʽ����ҡ�����ڿ������͹ػ�ָ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Tx2_Off_Test(Remote rc)
{
	if(rc.s1 == 2 && rc.ch0 < 500 && rc.ch2 > 1500)
		Tx2_Off_times++;		//������
	else 
		{
			Tx2_Off_times = 0;
			PC_Sendflag = 0;
		}
	if(Tx2_Off_times > 500)
		PC_Sendflag = Tx2_Off;
}

/**********************************************************************************************************
*�� �� ��: Remote_Process
*����˵��: ң����ģʽѡ��
*��    ��: rc
*�� �� ֵ: ��
**********************************************************************************************************/
void Remote_Process(Remote rc)
{
	if(Mouse_Key_Flag!=2)
	{
	  Mouse_Key_Flag=2;
	}
	
	if(rc.s2==3) //����ģʽ
	{
		Status.GimbalMode=Gimbal_Act_Mode;
		Status.ChassisMode=Chassis_Act_Mode;
		Status.ShootMode=Shoot_Powerdown_Mode;
	}
	if(rc.s2==2) //��¼ģʽ
	{
		Status.GimbalMode=Gimbal_Act_Mode; 
		Status.ChassisMode=Chassis_Act_Mode;
		Status.ShootMode=Shoot_Check_Mode;
//		SteeringEngine_Set(Infantry.MagClose);
		SteeringEngine_Set(Infantry.MagOpen);	
	}
	
//	if(rc.s2==1) //С����ģʽ
//	{
//		Status.GimbalMode=Gimbal_Act_Mode;
//		Status.ChassisMode=Chassis_SelfProtect_Mode;
//		Status.ShootMode=Shoot_Powerdown_Mode;
//	}
//	if(rc.s2 == 1)//���ģʽ
//	{
//		Status.GimbalMode = Gimbal_Buff_Mode;
////		Status.GimbalMode = Gimbal_Armor_Mode;
//		Status.ChassisMode = Chassis_NoFollow_Mode;
//		Status.ShootMode = Shoot_Tx2_Mode;
//	}
	if(rc.s2==1)  //����ģʽ
	{
//		Status.GimbalMode=Gimbal_Act_Mode;
//		Status.ChassisMode=Chassis_Powerdown_Mode;
//		Status.ShootMode=Shoot_Powerdown_Mode;
//		SteeringEngine_Set(Infantry.MagOpen);
		
		Status.GimbalMode=Gimbal_Armor_Mode;
		Status.ChassisMode=Chassis_Act_Mode;
		Status.ShootMode=Shoot_Tx2_Mode;
		SteeringEngine_Set(Infantry.MagClose);		
	}
}

/**********************************************************************************************************
*�� �� ��: MouseKey_Act_Cal
*����˵��: ����ģʽ(��ʣһ��e,q,z)
           w,s,a,d          ǰ�����Һ���
           q								�޸���ģʽ
					 e             		���ҵ���
           r                װ�ӵ�/���Լ��رյ���/����Pitch�����ֵ/��������
           f                ���ؼ���  
					 g								
					 z								���
           x                С��           
           c                ��λ��������������̨�������������������SpeedMode=0������رգ�
           v                ����ģʽ(С���ݣ����ҿ����ƶ�)
					 b								ͼ�γ�ʼ��
           shift            ʹ�ó�������
           ctrl             
           mouse.press_r    �Ҽ����븨��ģʽ
           mouse.press_l    ��������(����)/��������
*��    ��: RC_Ctl
*�� �� ֵ: ��
**********************************************************************************************************/
extern float GimbalPitchPos;
extern int   Shoot_Init_flag;
float k_onegrid = 0.1f;
float k_single = 0.9f;
short aaaa = 500;
void MouseKey_Act_Cal(RC_Ctl_t RC_Ctl)
{		
//	static int shoot_ticked;
/******************************���ؼ��� f��*****************************************/
		f_rising_flag=RC_Ctl.key.f-pre_key_f;
    pre_key_f=RC_Ctl.key.f;
			if(f_rising_flag==1)
				MicroSw_flag++;
			if(MicroSw_flag % 2 == 0)
			{
				MicroSw_Off;		
				F405.Laser_Flag = 0;
			}
			else 
			{
				MicroSw_On;		//Ĭ�ϼ��⿪��
				F405.Laser_Flag = 1;
			}	

/******************************����Ħ���� g��*****************************************/
		g_rising_flag=RC_Ctl.key.g-pre_key_g;
    pre_key_g=RC_Ctl.key.g;
			if(g_rising_flag==1)
			{
				Tim4_Configuration();
				delay_ms(20);
				FrictionSpeedChoose();
				FrictionWheel_Set(FrictionWheel_speed);
			}

/******************************ͼ�ν����ʼ�� b��*****************************************/
		b_rising_flag=RC_Ctl.key.b-pre_key_b;
    pre_key_b=RC_Ctl.key.b;
			if(b_rising_flag==1)
				Graphic_Init_flag++;
			if(Graphic_Init_flag % 2 == 0)
			{	
				F405.Graphic_Init_Flag = 0;
			}
			else 
			{
				F405.Graphic_Init_Flag = 1;
			}			

/******************************���ұ���ģʽ(С����) ctrl��*****************************************/
		ctrl_rising_flag = RC_Ctl.key.ctrl - pre_key_ctrl;
		pre_key_ctrl=RC_Ctl.key.ctrl;
		if(ctrl_rising_flag==1)
			Turning_flag++;
//		if((RC_Ctl.key.ctrl==1 && Status.ChassisMode == Chassis_Act_Mode) || (Status.ChassisMode == Chassis_SelfProtect_Mode && RC_Ctl.key.ctrl == 1))//��û�б�����ģʽ����һֱ���ŵ����
//		{
////			if(Status.GimbalMode == Gimbal_Buff_Mode)
////				Status.GimbalMode = Gimbal_Act_Mode;		
//			if(Status.ChassisMode == Chassis_Act_Mode)
//				Status.ChassisMode=Chassis_SelfProtect_Mode;
//			else if(Status.ChassisMode == Chassis_SelfProtect_Mode)
//				Status.ChassisMode = Chassis_Act_Mode;
//		}
		if(RC_Ctl.key.ctrl==1)//��û�б�����ģʽ����һֱ���ŵ����
		{
			if(Status.ChassisMode == Chassis_Act_Mode || Status.ChassisMode == Chassis_SelfProtect_Mode)
			Status.ChassisMode=Chassis_SelfProtect_Mode;
		}
		else if(Status.ChassisMode == Chassis_SelfProtect_Mode)
		{
			Status.ChassisMode = Chassis_Act_Mode;
		}
		
/******************************����ģʽ e��*****************************************/	
			e_rising_flag=RC_Ctl.key.e-pre_key_e;
			pre_key_e = RC_Ctl.key.e;
		
		  /*��ס�ŵ���*/
			if(RC_Ctl.key.e==1)//��û�б�����ģʽ����һֱ���ŵ����
			{
				if(Status.ChassisMode == Chassis_Act_Mode || Status.ChassisMode == Chassis_Solo_Mode)
				Status.ChassisMode=Chassis_Solo_Mode;
			}
			else if(Status.ChassisMode == Chassis_Solo_Mode)
			{
				Status.ChassisMode = Chassis_Act_Mode;
			}
			
/*����һ���л�ģʽ*/
//			if(e_rising_flag == 1)
//			{
//				if(Status.ChassisMode == Chassis_Act_Mode || Status.ChassisMode == Chassis_Solo_Mode)
//					Solo_flag++;
//			}		
////Solo_flagΪż ��������Ϊ�棬����������λ��+1�ָ�
//			if(Solo_flag % 2 == 0 && Status.ChassisMode == Chassis_Solo_Mode)				//��ԭ
//			{
//				Status.ChassisMode = Chassis_Act_Mode;
//			}
//			else if(Solo_flag % 2 == 1 && Status.ChassisMode == Chassis_Act_Mode)		//����Ϊ����ģʽ
//			{
//				Status.ChassisMode = Chassis_Solo_Mode;
//			}
			

/*******************************�سǲ������� r��**********************************************/			
	 r_rising_flag=RC_Ctl.key.r-pre_key_r;
   pre_key_r=RC_Ctl.key.r;
	 if(r_rising_flag==1)			//��������ʱ
	 {
//	   DanCang_Flag++;
//	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
//	 TIM_Cmd(TIM3,ENABLE);
		 SteeringEngine_Configuration();
		 if(magazineState == 0x00) 
			 SteeringEngine_Set(Infantry.MagOpen);
		 else
			 SteeringEngine_Set(Infantry.MagClose);
//		 SteeringEngineDelay = aaaa;
	 }
//	 if(DanCang_Flag % 2 == 1)
//	 {
//		SteeringEngineDelay = aaaa;
//		SteeringEngine_Set(Infantry.MagOpen);
//	 }
//	 else
//	 {
//		SteeringEngineDelay = aaaa;
//		SteeringEngine_Set(Infantry.MagClose);
//	 }
//	 
//	 if(SteeringEngineDelay > 0)
//			SteeringEngineDelay--;
//	 else
//	 {
//			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,DISABLE);
//			TIM_Cmd(TIM3,DISABLE);
//	 }
	 
/********************************�������ݿ���**********************************************/	
		 if(RC_Ctl.key.shift == 1)
			F405.SuperPowerLimit = 2;		//ʹ�ó�������
		 else
		 {
	//		 if(F405.SuperPowerLimit != 1)		//û�����𲽼���
				F405.SuperPowerLimit = 0;				//��ʹ�ó�������
		 }
		 
/******************************�޸���ģʽ q��*****************************************/
//			q_rising_flag=RC_Ctl.key.q-pre_key_q;
//			pre_key_q = RC_Ctl.key.q;
//			if(q_rising_flag == 1)
//			{
//				if(Status.ChassisMode == Chassis_Act_Mode || Status.ChassisMode == Chassis_NoFollow_Mode)
//					NoFollow_flag++;
//			}		
//			if(NoFollow_flag % 2 == 0 && Status.ChassisMode == Chassis_NoFollow_Mode)
//			{
//				Status.ChassisMode = Chassis_Act_Mode;
//			}
//			else if(NoFollow_flag % 2 == 1 && Status.ChassisMode == Chassis_Act_Mode)
//			{
//				Status.ChassisMode = Chassis_NoFollow_Mode;
//			}

///******************************���ģʽ z��*****************************************/
//			z_rising_flag=RC_Ctl.key.z-pre_key_z;
//			pre_key_z = RC_Ctl.key.z;
//			if(z_rising_flag == 1)
//			{
//				if(Status.ChassisMode == Chassis_Act_Mode || Status.ChassisMode == Chassis_NoFollow_Mode)
//					Buff_flag++;
//			}		
//			if(Buff_flag % 2 == 0 && Status.ChassisMode == Chassis_NoFollow_Mode && Status.GimbalMode == Gimbal_Buff_Mode)			//
//			{
//				Status.ChassisMode = Chassis_Act_Mode;
//				Status.GimbalMode = Gimbal_Act_Mode;
//				Status.ShootMode = Shoot_Fire_Mode;
//			}
//			else if(Buff_flag % 2 == 1 && Status.ChassisMode == Chassis_NoFollow_Mode && Status.GimbalMode == Gimbal_Act_Mode)
//			{
//				Status.ChassisMode = Chassis_NoFollow_Mode;
//				Status.GimbalMode = Gimbal_Buff_Mode;
//				Status.ShootMode = Shoot_Fire_Mode;				//����ֶ���
//				//Status.ShootMode = Shoot_Tx2_Mode;				//����Զ���
//			}

		/******************************Big  Buff  ���� z��*****************************************/
//		z_rising_flag=RC_Ctl.key.z-pre_key_z;
//    pre_key_z=RC_Ctl.key.z;
//		if(z_rising_flag==1)
//		{
//			Press_Key_z_Flag++;
//			Press_Key_x_Flag = 0;
//			if(Press_Key_z_Flag%2==1)
//			{ 
//				MirocPosition = 0;
//			  Status.GimbalMode=Gimbal_Buff_Mode;
//		    Status.ChassisMode=Chassis_Powerdown_Mode;
//		    Status.ShootMode=Shoot_Tx2_Mode;
//			}
//			else
//			{
//				Status.ShootMode=Shoot_Fire_Mode;
//				Status.GimbalMode=Gimbal_Act_Mode;
//  		  Status.ChassisMode=Chassis_Act_Mode;
//			}
//		}
//		
//		/******************************Small  Buff ��С�� x��*****************************************/
//		x_rising_flag=RC_Ctl.key.x-pre_key_x;
//    pre_key_x=RC_Ctl.key.x;
//		if(x_rising_flag==1)
//		{
//			Press_Key_x_Flag++;
//			Press_Key_z_Flag = 0;
//			if(Press_Key_x_Flag%2==1)
//			{
//				MirocPosition =0;
//			  Status.GimbalMode=Gimbal_Buff_Mode;
//		    Status.ChassisMode=Chassis_Powerdown_Mode;
//		    Status.ShootMode=Shoot_Tx2_Mode;
//			}
//			else
//			{
//				Status.ShootMode=Shoot_Fire_Mode;
//				Status.GimbalMode=Gimbal_Act_Mode;
//  		  Status.ChassisMode=Chassis_Act_Mode;
//			}
//		}

		/******************************��갴�����*******************************************/
		mouse_Press_r_rising_flag=RC_Ctl.mouse.press_l-pre_mouse_l;
		pre_mouse_l=RC_Ctl.mouse.press_l;
		if(RC_Ctl.mouse.press_l==1)
		{
			 waitFlag[5]++;  
			 if((waitFlag[5]<20)&&(waitFlag[5]>0))//�̰�     //��ʱ���Ե�һ��
			 {	
				 if	(mouse_Press_r_rising_flag==1)	
				 {
						waitFlag[5]=0;
						if(F105.IsShootAble==1)//Heat_Limit
						{
							if(ABS(Bodan_Pos-PidBodanMotorPos.SetPoint)<3000 && Shoot_Init_flag == 1)
							{
								MirocPosition += Onegrid;		
							}						
						}
				 }				 
			 }
			 if(waitFlag[5]>60)			//����
			 {
					if(ABS(PidBodanMotorPos.SetPoint-Bodan_Pos)<20000)
					{ 
						if(F105.IsShootAble==1)
						{
							if(ABS(Bodan_Pos-PidBodanMotorPos.SetPoint)<8000  && Shoot_Init_flag == 1)
							{
//								if(shoot_ticked%10==0)
//								{
//									MirocPosition +=  Onegrid;
//								}
								MirocPosition += k_onegrid * Onegrid;
//								shoot_ticked++;
							}
								
						}
					}	
			 }		 
		}
		else if(RC_Ctl.mouse.press_l==0)			//����
		{
			waitFlag[5]=0;
//			shoot_ticked = 0;
		}
		
		/******************************����������ƣ����飩 �Ҽ�*****************************************/
		if(RC_Ctl.mouse.press_r==1) 
		{
		  Status.GimbalMode=Gimbal_Armor_Mode;
		  Status.ShootMode=Shoot_Fire_Mode;
			Laser_Off();
		}
		else if(Status.GimbalMode==Gimbal_Armor_Mode) 
		{
			Status.GimbalMode=Gimbal_Act_Mode;
			Status.ShootMode= Shoot_Fire_Mode;
			if(MicroSw_flag % 2 == 0)
				Laser_Off();
			else
				Laser_On();
		}
	 
	/********************************ң�ش���************************************************/	
//	 if(RC_Ctl.rc.s2==2)   //Big 	Buff
//	{
//	 	if(Mouse_Key_Flag!=4)
//   	  Mouse_Key_Flag=4;
//		Status.ShootMode=Shoot_Tx2_Mode;
//		Status.GimbalMode=Gimbal_Buff_Mode;
//		Status.ChassisMode=Chassis_Powerdown_Mode;
//	}
//	if(RC_Ctl.rc.s2==1)   //Small Buff
//	{
//		if(Mouse_Key_Flag!=5)
//    	Mouse_Key_Flag=5;
//		Status.ShootMode=Shoot_Tx2_Mode;
//		Status.GimbalMode=Gimbal_Buff_Mode;
//		Status.ChassisMode=Chassis_Powerdown_Mode;
//	}
}


/**********************************************************************************************************
*�� �� ��: Mouse_Key_Process
*����˵��: ����ģʽ��ʼ��
*��    ��: RC_Ctl
*�� �� ֵ: ��
**********************************************************************************************************/
void Mouse_Key_Process(RC_Ctl_t RC_Ctl)
{
	/******************************���β�����ʼ*****************************************/
	if(Mouse_Key_Flag!=3)
	{
		SpeedMode=1;
		Mouse_Key_Flag=3;
		magazineState=0x00; //Close
		SteeringEngine_Set(Infantry.MagClose);
	}
	Status.GimbalMode=Gimbal_Act_Mode;
  Status.ChassisMode=Chassis_Act_Mode;
	Status.ShootMode=Shoot_Fire_Mode;
}

/**********************************************************************************************************
*�� �� ��: ModeChoose_task
*����˵��: ģʽѡ������
*��    ��: pvParameters
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t ModeChoose_high_water;
void ModeChoose_task(void *pvParameters)
{
	
   while (1) {
		Status_Act();
		
    vTaskDelay(3); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        ModeChoose_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
