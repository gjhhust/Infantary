/**********************************************************************************************************
 * @�ļ�     DataSendTask.c
 * @˵��     ���ݷ���
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2019.10
**********************************************************************************************************/
#include "main.h"
/*----------------------------------�ڲ�����---------------------------*/

/*----------------------------------�ṹ��-----------------------------*/

/*----------------------------------�ⲿ����---------------------------*/
extern INA260 INA260_1;//����
extern INA260 INA260_2;//���
extern float AD_actual_value;//����ʵ�ʵ�ѹ
extern F405_typedef F405;
extern JudgeReceive_t JudgeReceive;
/**********************************************************************************************************
*�� �� ��: ChassisCan1Send
*����˵��: ���̵������ֵ����
*��    ��: �ĸ��������ֵ
*�� �� ֵ: ��
**********************************************************************************************************/
void ChassisCan1Send(short a,short b,short c,short d)
{
    CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = 0x200;
	  a=LIMIT_MAX_MIN(a,10000,-10000);
	  b=LIMIT_MAX_MIN(b,10000,-10000);
	  c=LIMIT_MAX_MIN(c,10000,-10000);
	  d=LIMIT_MAX_MIN(d,10000,-10000);
    tx_message.Data[0] = (unsigned char)((a>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(a&0xff);  
    tx_message.Data[2] = (unsigned char)((b>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(b&0xff);
    tx_message.Data[4] = (unsigned char)((c>>8)&0xff);
    tx_message.Data[5] = (unsigned char)(c&0xff);
    tx_message.Data[6] = (unsigned char)((d>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(d&0xff);
    CAN_Transmit(CAN1,&tx_message);
}
/**********************************************************************************************************
*�� �� ��: Can2Send0
*����˵��: can2���ͺ���
*��    ��: ChassisSpeedw, Remain_power, IsShootAble, RobotRed, BulletSpeedLevel
*�� �� ֵ: ��
**********************************************************************************************************/
void Can2Send0(F105_Typedef *F105_Send)
{
	  CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = 0x100;
	  
	  memcpy(&tx_message.Data[0],&F105_Send->ChassisSpeedw,2);
		memcpy(&tx_message.Data[2],&F105_Send->Remain_power,2);
		memcpy(&tx_message.Data[4],&F105_Send->IsShootAble,1);
		memcpy(&tx_message.Data[5],&F105_Send->RobotRed,1);
		memcpy(&tx_message.Data[6],&F105_Send->BulletSpeedLevel,1);	

	  CAN_Transmit(CAN2,&tx_message);
}
/**********************************************************************************************************
*�� �� ��: Can2Send1
*����˵��: can2���ͺ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Can2Send1(short *k,short *p,short *m, short *n)
{
	  CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = 0x100;
	  
	  memcpy(tx_message.Data,k,2);	
	  memcpy(&tx_message.Data[2],p,2);	
	  memcpy(&tx_message.Data[4],m,2);	
	  memcpy(&tx_message.Data[6],n,2);	
	   
	  CAN_Transmit(CAN2,&tx_message);
}
/**********************************************************************************************************
*�� �� ��: USART2SEND
*����˵��: DataScope���ͺ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
extern float test_W_Chassis_t;
void USART2SEND(void)
{
		DataScope_Get_Channel_Data(test_W_Chassis_t, 1 );  
		DataScope_Get_Channel_Data(JudgeReceive.realChassispower, 2 );  
		DataScope_Get_Channel_Data(JudgeReceive.remainEnergy, 3 );  
		DataScope_Get_Channel_Data(JudgeReceive.MaxPower, 4 );  
		DataScope_Get_Channel_Data(0.0, 5 );  
		DataScope_Get_Channel_Data(0.0, 6 );  
		DataScope_Get_Channel_Data(0.0, 7 );  
		DataScope_Get_Channel_Data(0.0, 8 );  
		DataScope_Get_Channel_Data(0.0, 9 );  
		DataScope_Get_Channel_Data(0.0, 10 );  

		u8 Send_Count;
		Send_Count = DataScope_Data_Generate(10);
		for( int i = 0 ; i < Send_Count; i++)
		{
			while((USART2->SR&0X40)==0);  	
			USART2->DR = DataScope_OutPut_Buffer[i];    
		}
}
