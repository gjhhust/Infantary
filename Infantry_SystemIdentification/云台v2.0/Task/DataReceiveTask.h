#ifndef __DATARECEIVETASK_H__
#define __DATARECEIVETASK_H__
#include "usart1.h"
#include "stm32f4xx.h"

/*ң�����ṹ��*/
typedef __packed struct
{
		unsigned short ch0;
		unsigned short ch1;
		unsigned short ch2;
		unsigned short ch3;
		unsigned short s1;
		unsigned short s2;
}Remote;
/*���ṹ��*/
typedef __packed 	struct
{
		short x;
		short y;
		short z;
		unsigned char press_l;
		unsigned char press_r;
}Mouse;
/*���̽ṹ��*/
typedef __packed struct
{
		unsigned short w,s,a,d,q,e,r,f,g,z,x,c,v,b,shift,ctrl;
}Key;
/*ң����ṹ���ۺ�*/
typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
  char RCrecvd,RCDisconnectCnt;//RCrecvdΪ���ݽ��ձ�־λ
}RC_Ctl_t;
/*�����ǽ��սṹ��*/
typedef struct GYRO{
		float GY;				//�������ٶ�
		float GZ;
		float PITCH;		//�����ǽǶ�
		float YAW;
}
Gyro_Typedef;
/*���ʰ���սṹ��*/
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

#define ARMOR_NO_AIM	0xff	//û���ҵ�Ŀ��
#define ARMOR_AIMED		0x30	//��Ŀ��

#endif
