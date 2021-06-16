#ifndef __ACTIONTASK_H__
#define __ACTIONTASK_H__

#include "main.h"
/*步兵模式选择结构体*/
typedef struct
{
  short ControlMode;
	short ChassisMode;
	short ShootMode;
	short GimbalMode;
	short RstMode;
	short BulletSpeedMode;
	float BulletSpeed;
	short UseSw_NoSw;
}Status_t;

#define Control_RC_Mode 300
#define Control_MouseKey_Mode 301
#define Control_Powerdown_Mode 302

//#define Chassis_Act_Mode 303
//#define Chassis_SelfProtect_Mode 304
//#define Chassis_Solo_Mode 	305
//#define Chassis_Crazy_Mode	306
//#define Chassis_Powerdown_Mode 307

//#define Gimbal_Act_Mode 311
//#define Gimbal_Armor_Mode  312
//#define Gimbal_Buff_Mode 313
//#define Gimbal_DropShot_Mode 314
//#define Gimbal_Powerdown_Mode 315

#define Chassis_Powerdown_Mode 					0
#define Chassis_Act_Mode  							1
#define Chassis_SelfProtect_Mode 				2
#define Chassis_Solo_Mode  							3
#define Chassis_NoFollow_Mode  					4

#define Gimbal_Powerdown_Mode 					0
#define Gimbal_Act_Mode 								1
#define Gimbal_Armor_Mode  							2
#define Gimbal_Buff_Mode 								3
#define Gimbal_DropShot_Mode 						4
#define Gimbal_SI_Mode 									5

#define Shoot_Fire_Mode  344
#define Shoot_Powerdown_Mode 345
#define Shoot_Check_Mode 346
#define Shoot_Tx2_Mode 347

void SetInputMode(Remote );
void Status_Act(void);

void MouseKey_Act_Cal(RC_Ctl_t RC_Ctl);

void Remote_Process(Remote rc);
void Mouse_Key_Process(RC_Ctl_t RC_Ctl);
void Powerdown_Process(void);
void Tx2_Off_Test(Remote rc);


void ModeChoose_task(void *pvParameters);

#endif
