/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control_task.h"   /* controlTask */
#include "remote_task.h"    /* remoteTask */
#include "ui_task.h"        /* uiTask */
#include "test_task.h"      /* testTask */
#include "i2c_mutex.h"      /* I2C1 互斥锁（创建在这里）*/

/* ============ 任务开关（一键切换）============
 * 1 = 运行，0 = 禁用（编译期直接不创建该任务）
 *
 *           测试模式        正常模式
 *  TASK_CONTROL_ON    0        1
 *  TASK_REMOTE_ON     0        1
 *  TASK_UI_ON         0        1
 *  TASK_TEST_ON       1        0
 * ⚠️ TASK_CONTROL_ON 和 TASK_TEST_ON 不要同时开（两者都会写电机）。 */
#define TASK_CONTROL_ON   1
#define TASK_REMOTE_ON    1
#define TASK_UI_ON        1

#define TASK_TEST_ON      0
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//在这里定义任务句�?,出身设置
osThreadId_t remoteTaskHandle;
//遥控任务
const osThreadAttr_t remoteTask_attributes = {
  .name = "remoteTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,  
};

osThreadId_t controlTaskHandle;
//控制任务
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,  //设置为最高优先级
};
SemaphoreHandle_t i2c_mutex = NULL; /* I2C1 互斥锁（OLED+MPU 共用）*/
osThreadId_t uiTaskHandle;
//显示任务
const osThreadAttr_t uiTask_attributes = {
  .name = "uiTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,   //最低优先级，纯显示不抢时间
};

osThreadId_t testTaskHandle;
//测试任务（仅测试模式创建，见顶部任务开关）
const osThreadAttr_t testTask_attributes = {
  .name = "testTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,   //测试任务最高（测试模式才存在）
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* I2C1 互斥锁：OLED 和 MPU6050 共用总线 */
  i2c_mutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
    /* 遥控邮箱：提前在调度器启动前创建，避免 control_task 先于 remote_task 运行时 remoteQueueHandle 还是 NULL */
    remoteQueueHandle = xQueueCreate(1, sizeof(RemoteData_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* 任务创建统一放在下面 USER CODE 区：带开关控制 + 防 CubeMX 重新生成时冲掉 */

  /* USER CODE BEGIN RTOS_THREADS */
  /* 按任务开关创建（开关宏定义在文件顶部 Includes 区）*/
#if TASK_CONTROL_ON
  controlTaskHandle = osThreadNew(controlTask, NULL, &controlTask_attributes);
#endif
#if TASK_REMOTE_ON
  remoteTaskHandle  = osThreadNew(remoteTask,  NULL, &remoteTask_attributes);
#endif
#if TASK_UI_ON
  uiTaskHandle      = osThreadNew(uiTask,      NULL, &uiTask_attributes);
#endif
#if TASK_TEST_ON
  testTaskHandle    = osThreadNew(testTask,    NULL, &testTask_attributes);
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  (void)argument;              /* 未使用参数，避免编译警告 */
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ============ 栈溢出钩子（需在 FreeRTOSConfig.h 开 configCHECK_FOR_STACK_OVERFLOW）====
 * 任务栈被写穿（局部变量太多/恢复溢出）时，FreeRTOS 会调用这里。
 * 此时栈已坏，绝不能打印/访问外设（会二次溢出）。
 * 安全做法：关中断 + 死循环锁死，并把爆栈任务名存到全局，方便调试器查看。 */
char g_stackOverflowTask[configMAX_TASK_NAME_LEN] = "none";
volatile uint8_t g_stackOverflowFlag = 0;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;               /* 未使用参数，避免编译警告 */
    __disable_irq();                       /* 锁死调度，防止再切换/再写坏内存 */
    g_stackOverflowFlag = 1;               /* 调试标志：=1 表示曾经溢栈 */
    if (pcTaskName != NULL) {
        /* 安全拷任务名（栈已坏，只做简单拷贝，不打印）*/
        uint8_t i = 0;
        while (i < configMAX_TASK_NAME_LEN - 1 && pcTaskName[i]) {
            g_stackOverflowTask[i] = pcTaskName[i];
            i++;
        }
        g_stackOverflowTask[i] = '\0';
    }
    for (;;) { }                           /* 死循环：停在爆栈点，方便调试器定位 */
}

/* USER CODE END Application */

