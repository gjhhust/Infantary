#include "Start_Task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim7.h"
unsigned portBASE_TYPE Mark[8];		//�۲������ջʹ�����
uint32_t CPU_high_water;

/*�������ȼ���ֵԽ�ͣ����ȼ�Խ��*/

#define START_TASK_PRIO 1  //�������ȼ�
#define START_STK_SIZE 128 //�����ջ
static TaskHandle_t StartTask_Handler; //������

#define CPU_TASK_PRIO 2  //�������ȼ�
#define CPU_STK_SIZE 128 //�����ջw
static TaskHandle_t CPUTask_Handler; //������

#define GIMBAL_TASK_PRIO 20  //�������ȼ�
#define GIMBAL_STK_SIZE 128 //�����ջ
static TaskHandle_t GimbalTask_Handler; //������

#define CHASSIS_TASK_PRIO 18  //�������ȼ�
#define CHASSIS_STK_SIZE 128 //�����ջ
static TaskHandle_t ChassisTask_Handler; //������

#define OFFLINE_CHECK_TASK_PRIO 3  //�������ȼ�
#define OFFLINE_CHECK_STK_SIZE 128 //�����ջ
static TaskHandle_t OfflineCheckTask_Handler; //������

#define MODE_CHOOSE_TASK_PRIO 16  //�������ȼ�
#define MODE_CHOOSE_STK_SIZE 128 //�����ջ
static TaskHandle_t ModeChooseTask_Handler; //������

#define SHOOT_TASK_PRIO 17  //�������ȼ�
#define SHOOT_STK_SIZE 128 //�����ջ
static TaskHandle_t ShootTask_Handler; //������

#define TX2_TASK_PRIO 19  //�������ȼ�
#define TX2_STK_SIZE 128 //�����ջ
static TaskHandle_t TX2Task_Handler; //������


void start_task(void *pvParameters)
{
  taskENTER_CRITICAL();
								
	xTaskCreate((TaskFunction_t)Chassis_task,          //������
                (const char *)"Chassis_task",          //��������
                (uint16_t)CHASSIS_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)CHASSIS_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&ChassisTask_Handler); //������

	xTaskCreate((TaskFunction_t)Gimbal_task,          //������
                (const char *)"Gimbal_task",          //��������
                (uint16_t)GIMBAL_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)GIMBAL_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&GimbalTask_Handler); //������
								
	xTaskCreate((TaskFunction_t)Shoot_task,          //������
                (const char *)"Shoot_task",          //��������
                (uint16_t)SHOOT_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)SHOOT_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&ShootTask_Handler); //������
								
	xTaskCreate((TaskFunction_t)ModeChoose_task,          //������
                (const char *)"ModeChoose_task",          //��������
                (uint16_t)MODE_CHOOSE_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)MODE_CHOOSE_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&ModeChooseTask_Handler); //������
								
	xTaskCreate((TaskFunction_t)TX2_task,          //������
                (const char *)"TX2_task",          //��������
                (uint16_t)TX2_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)TX2_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&TX2Task_Handler); //������
								
	xTaskCreate((TaskFunction_t)Offline_Check_task,          //������
                (const char *)"Offline_Check_task",          //��������
                (uint16_t)OFFLINE_CHECK_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)OFFLINE_CHECK_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&OfflineCheckTask_Handler); //������
								
	xTaskCreate((TaskFunction_t)CPU_task,          //������
                (const char *)"CPU_task",          //��������
                (uint16_t)CPU_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)CPU_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&CPUTask_Handler); //������
								
								
	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
  taskEXIT_CRITICAL();            //�˳��ٽ���
}


uint8_t CPU_RunInfo1[200]; //������������ʱ����Ϣ �ֱ��ǣ������� ����״̬ ���ȼ� ʣ��ջ �������
uint8_t CPU_RunInfo2[200]; //������������ʱ����Ϣ �ֱ��ǣ������� ���м���  ʹ����

void CPU_task(void *pvParameters)
{
	
   while (1) {
    memset(CPU_RunInfo1,0,200); //��Ϣ����������
 
    vTaskList((char *)&CPU_RunInfo1); //��ȡ��������ʱ����Ϣ
    
    memset(CPU_RunInfo2,0,200); //��Ϣ����������
 		Mark[0] = uxTaskGetStackHighWaterMark(StartTask_Handler );
		Mark[1] = uxTaskGetStackHighWaterMark(CPUTask_Handler);
		Mark[2] = uxTaskGetStackHighWaterMark(GimbalTask_Handler);
		Mark[3] = uxTaskGetStackHighWaterMark(ChassisTask_Handler);
		Mark[4] = uxTaskGetStackHighWaterMark(OfflineCheckTask_Handler);
		Mark[5] = uxTaskGetStackHighWaterMark(ModeChooseTask_Handler);
		Mark[6] = uxTaskGetStackHighWaterMark(ShootTask_Handler);
		Mark[7] = uxTaskGetStackHighWaterMark(TX2Task_Handler);
		 
    vTaskGetRunTimeStats((char *)&CPU_RunInfo2);
    vTaskDelay(1000); /* ��ʱ 500 �� tick */

#if INCLUDE_uxTaskGetStackHighWaterMark
        CPU_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //������
                (const char *)"start_task",          //��������
                (uint16_t)START_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&StartTask_Handler); //������
}


