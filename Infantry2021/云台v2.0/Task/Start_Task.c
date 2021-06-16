#include "Start_Task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim7.h"
unsigned portBASE_TYPE Mark[8];		//观察任务堆栈使用情况
uint32_t CPU_high_water;

/*任务优先级数值越低，优先级越低*/

#define START_TASK_PRIO 1  //任务优先级
#define START_STK_SIZE 128 //任务堆栈
static TaskHandle_t StartTask_Handler; //任务句柄

#define CPU_TASK_PRIO 2  //任务优先级
#define CPU_STK_SIZE 128 //任务堆栈w
static TaskHandle_t CPUTask_Handler; //任务句柄

#define GIMBAL_TASK_PRIO 20  //任务优先级
#define GIMBAL_STK_SIZE 128 //任务堆栈
static TaskHandle_t GimbalTask_Handler; //任务句柄

#define CHASSIS_TASK_PRIO 18  //任务优先级
#define CHASSIS_STK_SIZE 128 //任务堆栈
static TaskHandle_t ChassisTask_Handler; //任务句柄

#define OFFLINE_CHECK_TASK_PRIO 3  //任务优先级
#define OFFLINE_CHECK_STK_SIZE 128 //任务堆栈
static TaskHandle_t OfflineCheckTask_Handler; //任务句柄

#define MODE_CHOOSE_TASK_PRIO 16  //任务优先级
#define MODE_CHOOSE_STK_SIZE 128 //任务堆栈
static TaskHandle_t ModeChooseTask_Handler; //任务句柄

#define SHOOT_TASK_PRIO 17  //任务优先级
#define SHOOT_STK_SIZE 128 //任务堆栈
static TaskHandle_t ShootTask_Handler; //任务句柄

#define TX2_TASK_PRIO 19  //任务优先级
#define TX2_STK_SIZE 128 //任务堆栈
static TaskHandle_t TX2Task_Handler; //任务句柄


void start_task(void *pvParameters)
{
  taskENTER_CRITICAL();
								
	xTaskCreate((TaskFunction_t)Chassis_task,          //任务函数
                (const char *)"Chassis_task",          //任务名称
                (uint16_t)CHASSIS_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)CHASSIS_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&ChassisTask_Handler); //任务句柄

	xTaskCreate((TaskFunction_t)Gimbal_task,          //任务函数
                (const char *)"Gimbal_task",          //任务名称
                (uint16_t)GIMBAL_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)GIMBAL_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&GimbalTask_Handler); //任务句柄
								
	xTaskCreate((TaskFunction_t)Shoot_task,          //任务函数
                (const char *)"Shoot_task",          //任务名称
                (uint16_t)SHOOT_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)SHOOT_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&ShootTask_Handler); //任务句柄
								
	xTaskCreate((TaskFunction_t)ModeChoose_task,          //任务函数
                (const char *)"ModeChoose_task",          //任务名称
                (uint16_t)MODE_CHOOSE_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)MODE_CHOOSE_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&ModeChooseTask_Handler); //任务句柄
								
	xTaskCreate((TaskFunction_t)TX2_task,          //任务函数
                (const char *)"TX2_task",          //任务名称
                (uint16_t)TX2_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)TX2_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&TX2Task_Handler); //任务句柄
								
	xTaskCreate((TaskFunction_t)Offline_Check_task,          //任务函数
                (const char *)"Offline_Check_task",          //任务名称
                (uint16_t)OFFLINE_CHECK_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)OFFLINE_CHECK_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&OfflineCheckTask_Handler); //任务句柄
								
	xTaskCreate((TaskFunction_t)CPU_task,          //任务函数
                (const char *)"CPU_task",          //任务名称
                (uint16_t)CPU_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)CPU_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&CPUTask_Handler); //任务句柄
								
								
	vTaskDelete(StartTask_Handler); //删除开始任务
  taskEXIT_CRITICAL();            //退出临界区
}


uint8_t CPU_RunInfo1[200]; //保存任务运行时间信息 分别是：任务名 任务状态 优先级 剩余栈 任务序号
uint8_t CPU_RunInfo2[200]; //保存任务运行时间信息 分别是：任务名 运行计数  使用率

void CPU_task(void *pvParameters)
{
	
   while (1) {
    memset(CPU_RunInfo1,0,200); //信息缓冲区清零
 
    vTaskList((char *)&CPU_RunInfo1); //获取任务运行时间信息
    
    memset(CPU_RunInfo2,0,200); //信息缓冲区清零
 		Mark[0] = uxTaskGetStackHighWaterMark(StartTask_Handler );
		Mark[1] = uxTaskGetStackHighWaterMark(CPUTask_Handler);
		Mark[2] = uxTaskGetStackHighWaterMark(GimbalTask_Handler);
		Mark[3] = uxTaskGetStackHighWaterMark(ChassisTask_Handler);
		Mark[4] = uxTaskGetStackHighWaterMark(OfflineCheckTask_Handler);
		Mark[5] = uxTaskGetStackHighWaterMark(ModeChooseTask_Handler);
		Mark[6] = uxTaskGetStackHighWaterMark(ShootTask_Handler);
		Mark[7] = uxTaskGetStackHighWaterMark(TX2Task_Handler);
		 
    vTaskGetRunTimeStats((char *)&CPU_RunInfo2);
    vTaskDelay(1000); /* 延时 500 个 tick */

#if INCLUDE_uxTaskGetStackHighWaterMark
        CPU_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
}


