/**********************************************************************************************************
 * @�ļ�     pid.c
 * @˵��     pid�㷨
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2020.1
**********************************************************************************************************/
#include "main.h"

/**********************************************************************************************************
*�� �� ��: PID_Calc
*����˵��: PID�����㷨
*��    ��: PID_Struct *P  PID�����ṹ��
  *        ActualValue    PID���㷴��������ǰ��ʵ���ֵ��
*�� �� ֵ: PID�����������ֵ
**********************************************************************************************************/
float PID_Calc(Pid_Typedef *P, float ActualValue)
{
		P->ActualValue = ActualValue;
	
		P->PreError = P->SetPoint - ActualValue;
		P->dError = P->PreError - P->LastError;
	
	  P->SetPointLast = P->SetPoint;
	
		P->SumError += P->PreError;
		P->LastError = P->PreError;
		
		if(P->SumError >= P->IMax)
			P->SumError = P->IMax;
		else if(P->SumError <= -P->IMax)
			P->SumError = -P->IMax;
		
		P->POut = P->P * P->PreError;
		P->IOut = P->I * P->SumError;
		P->DOut = P->D * P->dError;
		
		return LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax); 
}
