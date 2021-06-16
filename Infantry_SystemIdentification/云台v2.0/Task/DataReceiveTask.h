#ifndef __DATARECEIVETASK_H__
#define __DATARECEIVETASK_H__
#include "usart1.h"
#include "stm32f4xx.h"

/*遥控器结构体*/
typedef __packed struct
{
		unsigned short ch0;
		unsigned short ch1;
		unsigned short ch2;
		unsigned short ch3;
		unsigned short s1;
		unsigned short s2;
}Remote;
/*鼠标结构体*/
typedef __packed 	struct
{
		short x;
		short y;
		short z;
		unsigned char press_l;
		unsigned char press_r;
}Mouse;
/*键盘结构体*/
typedef __packed struct
{
		unsigned short w,s,a,d,q,e,r,f,g,z,x,c,v,b,shift,ctrl;
}Key;
/*遥键鼠结构体综合*/
typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
  char RCrecvd,RCDisconnectCnt;//RCrecvd为数据接收标志位
}RC_Ctl_t;
/*陀螺仪接收结构体*/
typedef struct GYRO{
		float GY;				//陀螺仪速度
		float GZ;
		float PITCH;		//陀螺仪角度
		float YAW;
}
Gyro_Typedef;
/*功率板接收结构体*/
typedef struct F105{
	short ChassisSpeedw;
	short Remain_power;
  char IsShootAble;
	char RobotRed;
  char BulletSpeedLevel;
	
  float Limit_Power_k;
	
	short HP;
	short Last_HP;
}F105_Typedef;

typedef struct
{
	float RCPitch;
	float RCYaw;
	float RCdistance;
	short ReceiveFromTx2BullectCnt;
	short FrictionWheel_speed;
	short DisConnect;
}PC_Receive_t;

typedef struct{
	short Angle;
	short RealSpeed;  
  short Current;	
}BodanMotorReceive_Typedef;

void Can1Receive1(CanRxMsg rx_message1);
void Can1Receive0(CanRxMsg rx_message0);
void Can2Receive0(CanRxMsg rx_message0);
void Can2Receive1(CanRxMsg rx_message1);

void PCReceive(unsigned char PCReceivebuffer[]);
void RemoteReceive(volatile unsigned char rx_buffer[]);

void RC_Rst(void);
void F105_Rst(void);

#define ARMOR_NO_AIM	0xff	//没有找到目标
#define ARMOR_AIMED		0x30	//有目标

#endif
