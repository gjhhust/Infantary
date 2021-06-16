/**********************************************************************************************************
 * @文件     DataReceive.c
 * @说明     接收函数
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2020.1
**********************************************************************************************************/
#include "main.h"

RC_Ctl_t RC_Ctl;//遥控器数据
Gyro_Typedef GyroReceive;//陀螺仪数据
F105_Typedef F105;
PC_Receive_t PC_Receive;
BodanMotorReceive_Typedef BodanReceive;

float PitchMotorReceive,YawMotorReceive;//Pitch,Yaw电机角度

KalmanFilter_t pitch_Kalman, yaw_Kalman;

short RC_DisConnect;//遥控器掉线检测
short F105_DisConect;//功率板掉线检测

extern Gimbal_Typedef Gimbal;
extern volatile long run_time_check;
extern Status_t Status;
extern short KalMan_doneflag;
/**********************************************************************************************************
*函 数 名: Can1Receive1
*功能说明: 功率板和拨弹电机通信
*形    参: rx_message1
*返 回 值: 无
**********************************************************************************************************/
void Can1Receive1(CanRxMsg rx_message1)
{
	switch(rx_message1.StdId)
	{ 
		case 0x100:
		     memcpy(&F105.ChassisSpeedw, &rx_message1.Data[0], 2);  
		     memcpy(&F105.Remain_power, &rx_message1.Data[2], 2); 		
	    	 memcpy(&F105.IsShootAble, &rx_message1.Data[4], 1);
				 memcpy(&F105.RobotRed, &rx_message1.Data[5], 1);  
		     memcpy(&F105.BulletSpeedLevel, &rx_message1.Data[6], 1);
		     F105_DisConect=0;
		 break;		
		case 0x205:
				 BodanReceive.Angle=rx_message1.Data[0]<<8 | rx_message1.Data[1];
				 BodanReceive.RealSpeed=rx_message1.Data[2]<<8 | rx_message1.Data[3];
 		break;
	}
}

/**********************************************************************************************************
*函 数 名: Can2Receive0
*功能说明: Pitch,Yaw电机角度接收
*形    参: rx_message0
*返 回 值: 无
**********************************************************************************************************/
void Can2Receive0(CanRxMsg rx_message0)
{
	switch(rx_message0.StdId)
	{ 
		 case 0x205:                                                             
				 PitchMotorReceive=rx_message0.Data[0]<<8 | rx_message0.Data[1];
		 break;
		 case 0x207:         
				 YawMotorReceive=rx_message0.Data[0]<<8 | rx_message0.Data[1];
		 break; 
	 }
}
/**********************************************************************************************************
*函 数 名: Can2Receive1
*功能说明: 陀螺仪数据接收
*形    参: rx_message1
*返 回 值: 无
**********************************************************************************************************/
void Can2Receive1(CanRxMsg rx_message1)
{
	switch(rx_message1.StdId)
	{ 
		 case 0x100:
		 {
			 memcpy(&GyroReceive.PITCH, rx_message1.Data, 4);
			 memcpy(&GyroReceive.GY, &rx_message1.Data[4], 4);
		 }
		 break;
		 case 0x101:
		 {
			 memcpy(&GyroReceive.YAW, &rx_message1.Data, 4);
			 memcpy(&GyroReceive.GZ, &rx_message1.Data[4], 4);
			 while (GyroReceive.YAW>180)
	     {
		     GyroReceive.YAW=GyroReceive.YAW-360;
	     }
	     while (GyroReceive.YAW<-180)
	     {
		     GyroReceive.YAW=GyroReceive.YAW+360;
	     }
		 }
	 }
}

/**********************************************************************************************************
*函 数 名: PCReceive
*功能说明: USART6与PC通信接收数据处理   
*形    参: rx_buffer[]
*返 回 值: 无
**********************************************************************************************************/
int pc_yaw;
short pc_pitch;
short distance;
short armor_state = 0;		//表示辅瞄是不是有找到目标
float aim_yaw, aim_pitch;
short tx2_last_receive_flag;	//表示有没有数据更新
short tx2_receive_flag;	
short aaaaaaaaaa;

void PCReceive(unsigned char PCReceivebuffer[])
{				
	run_time_check++;
	pc_pitch = (short)(PCReceivebuffer[2]<<8|PCReceivebuffer[3]);//这里不能转为float，不然负数传过来会变为正数
	pc_yaw = (int)(PCReceivebuffer[4]<<24|PCReceivebuffer[5]<<16|PCReceivebuffer[6]<<8|PCReceivebuffer[7]<<0);
	distance = (short)PCReceivebuffer[8];
	aim_yaw =  (float)pc_yaw/100.0f;
	aim_pitch = (float)pc_pitch/100.0f;
	tx2_receive_flag = PCReceivebuffer[1];	//作为更新标志位
	if(tx2_receive_flag == 0x01 && tx2_receive_flag != tx2_last_receive_flag)		
	{
		armor_state = ARMOR_AIMED;	
		aaaaaaaaaa++;		
	}
	else
	{
		armor_state = ARMOR_NO_AIM;	
	}
		tx2_last_receive_flag = tx2_receive_flag;

		if(Status.GimbalMode == Gimbal_Armor_Mode)
		{	
			if(ABS(aim_yaw - Gimbal.Yaw.Gyro) < 90 && ABS(aim_pitch - Gimbal.Pitch.MotorTransAngle)< 60)		//程序安全
			{
				PC_Receive.RCPitch = (float)aim_pitch;	//更新值
				PC_Receive.RCYaw = (float)(aim_yaw);
			}
			else
			{
				PC_Receive.RCPitch = Gimbal.Pitch.MotorTransAngle;	//更新值
				PC_Receive.RCYaw = Gimbal.Yaw.Gyro;
			}
		}
		else if(Status.GimbalMode == Gimbal_Buff_Mode)
		{
			if(ABS(aim_yaw - Gimbal.Yaw.MotorTransAngle) < 90 && ABS(aim_pitch - Gimbal.Pitch.MotorTransAngle)< 60)		//程序安全
			{
				PC_Receive.RCPitch = (float)aim_pitch;	//更新值
				PC_Receive.RCYaw = (float)(aim_yaw);
			}
			else
			{
				PC_Receive.RCPitch = Gimbal.Pitch.MotorTransAngle;	//更新值
				PC_Receive.RCYaw = Gimbal.Yaw.MotorTransAngle;
			}
		}

	PC_Receive.RCdistance = (float)distance/10.0f;
	
	PC_Receive.DisConnect=0;
	tx2_last_receive_flag = tx2_receive_flag;
}

/**********************************************************************************************************
*函 数 名: RemoteReceive
*功能说明: 遥控器数据接收
*形    参: rx_buffer[]
*返 回 值: 无
**********************************************************************************************************/
void RemoteReceive(volatile unsigned char rx_buffer[])
{
	RC_Ctl.rc.ch0 = (rx_buffer[0]| (rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
	RC_Ctl.rc.ch1 = ((rx_buffer[1] >> 3) | (rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
	RC_Ctl.rc.ch2 = ((rx_buffer[2] >> 6) | (rx_buffer[3] << 2) | (rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2
	RC_Ctl.rc.ch3 = ((rx_buffer[4] >> 1) | (rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
	RC_Ctl.rc.s1 = ((rx_buffer[5] >> 4)& 0x0003); //!< Switch left
	RC_Ctl.rc.s2 = ((rx_buffer[5] >> 6)& 0x0003);
	RC_Ctl.mouse.x = rx_buffer[6] | (rx_buffer[7] << 8); //!< Mouse X axis
	RC_Ctl.mouse.y = rx_buffer[8] | (rx_buffer[9] << 8); //!< Mouse Y axis
	RC_Ctl.mouse.z = rx_buffer[10] | (rx_buffer[11] << 8); //!< Mouse Z axis
	RC_Ctl.mouse.press_l = rx_buffer[12]; //!< Mouse Left Is Press ?
	RC_Ctl.mouse.press_r = rx_buffer[13]; //!< Mouse Right Is Press ?
	RC_Ctl.key.w = rx_buffer[14]&0x01; // KeyBoard value
	RC_Ctl.key.s = (rx_buffer[14]>>1)&0x01;
	RC_Ctl.key.a = (rx_buffer[14]>>2)&0x01;
	RC_Ctl.key.d = (rx_buffer[14]>>3)&0x01;
	RC_Ctl.key.shift =(rx_buffer[14]>>4)&0x01;
	RC_Ctl.key.ctrl = (rx_buffer[14]>>5)&0x01;
	RC_Ctl.key.q = (rx_buffer[14]>>6)&0x01;
	RC_Ctl.key.e = (rx_buffer[14]>>7)&0x01;	
	RC_Ctl.key.r = (rx_buffer[15])&0x01;
	RC_Ctl.key.f = (rx_buffer[15]>>1)&0x01;
	RC_Ctl.key.g = (rx_buffer[15]>>2)&0x01; 
	RC_Ctl.key.z = (rx_buffer[15]>>3)&0x01;
	RC_Ctl.key.x = (rx_buffer[15]>>4)&0x01;
	RC_Ctl.key.c = (rx_buffer[15]>>5)&0x01;
	RC_Ctl.key.v = (rx_buffer[15]>>6)&0x01;
	RC_Ctl.key.b = (rx_buffer[15]>>7)&0x01;
	if((RC_Ctl.rc.ch0-1024<15)&&(RC_Ctl.rc.ch0-1024>-15)) RC_Ctl.rc.ch0=1024;
	if((RC_Ctl.rc.ch1-1024<15)&&(RC_Ctl.rc.ch1-1024>-15)) RC_Ctl.rc.ch1=1024;
	if((RC_Ctl.rc.ch2-1024<10)&&(RC_Ctl.rc.ch2-1024>-10)) RC_Ctl.rc.ch2=1024;
	if((RC_Ctl.rc.ch3-1024<10)&&(RC_Ctl.rc.ch3-1024>-10)) RC_Ctl.rc.ch3=1024;	

	RC_DisConnect=0;
}


/**********************************************************************************************************
*函 数 名: RC_Rst
*功能说明: 遥控器数据复位
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void RC_Rst(void)
{
		RC_Ctl.rc.ch0 = 1024;
		RC_Ctl.rc.ch1 = 1024;
		RC_Ctl.rc.ch2 = 1024;
		RC_Ctl.rc.ch3 = 1024;
		RC_Ctl.mouse.x = 0;
		RC_Ctl.mouse.y = 0;
		RC_Ctl.mouse.z = 0;
		RC_Ctl.mouse.press_l = 0;                                                
		RC_Ctl.mouse.press_r = 0;
	
		RC_Ctl.key.w = 0;
		RC_Ctl.key.s = 0;                            
		RC_Ctl.key.a = 0;
		RC_Ctl.key.d = 0;
		RC_Ctl.key.q = 0;
		RC_Ctl.key.e = 0;
		RC_Ctl.key.r = 0;
		RC_Ctl.key.f = 0;
		RC_Ctl.key.shift = 0;
		RC_Ctl.key.ctrl = 0;
	
	  RC_Ctl.rc.s1 = 2;
		RC_Ctl.rc.s2 = 2;
} 


/**********************************************************************************************************
*函 数 名: F105_Rst
*功能说明: 功率板掉线复位
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
unsigned int Heat_LimitTick;
void F105_Rst()
{
	  Heat_LimitTick++;
	  if(Heat_LimitTick%50 ==1 )
	    F105.IsShootAble = 1;
    else
			F105.IsShootAble =0;
	  F105.Limit_Power_k = 0.25;
}

