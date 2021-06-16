#ifndef __SHOOTTASK_H
#define __SHOOTTASK_H

#include "main.h"
#include "DataReceivetask.h"
//700 «◊ÓµÕ…‰ÀŸ  12m/s
#define Init_FritionSpeed     2000
//#define Low_FrictionSpeed     5705				//15m/s		
//#define Medium_FrictionSpeed  5920			  //18m/s
//#define High_FrictionSpeed    6960				//30m/s

#define Onegrid 26807   //26807


void FrictionSpeedChoose(void);



void Shoot_Check_Cal(void);
void Shoot_Fire_Cal(void);
void Shoot_Powerdown_Cal(void);
void Shoot_Tx2_Cal(void);
void Shoot_Test_Cal(void);

void BodanMotor_CurrentPid_Cal(void);
void Pid_BodanMotor_Init(void);

void Shoot_task(void *pvParameters);

#endif
