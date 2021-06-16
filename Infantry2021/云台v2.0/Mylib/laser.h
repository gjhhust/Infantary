#ifndef __LASER_H__
#define __LASER_H__

#define Laser_On() GPIO_SetBits(GPIOC,GPIO_Pin_14) 
#define Laser_Off() GPIO_ResetBits(GPIOC,GPIO_Pin_14) 

void LASER_Configration(void);

#endif 
