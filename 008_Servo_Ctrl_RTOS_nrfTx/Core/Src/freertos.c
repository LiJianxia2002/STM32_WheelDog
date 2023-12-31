/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "servo.h"
#include "nrf24l01.h"
#include "string.h"
#include "mpu6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void CLAMP(int16_t* const num,int16_t min,int16_t max)
{
	if(*num>max) *num=max;
	if(*num<min) *num=min;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t tmp_buf[33];

typedef struct{

	uint16_t rocker[3];
	uint8_t button;
	int16_t pitch;
	int16_t yaw;
	uint8_t servos[14];
	uint8_t reserve[8];
	
}remote_Rx_t ;

remote_Rx_t remote;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t servos[14];
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId UART_TaskHandle;
osThreadId Servo_TaskHandle;
osMessageQId UART2PWMHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Start_UART_Task(void const * argument);
void Start_Servo_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of UART2PWM */
  osMessageQDef(UART2PWM, 16, uint16_t);
  UART2PWMHandle = osMessageCreate(osMessageQ(UART2PWM), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of UART_Task */
  osThreadDef(UART_Task, Start_UART_Task, osPriorityNormal, 0, 128);
  UART_TaskHandle = osThreadCreate(osThread(UART_Task), NULL);

  /* definition and creation of Servo_Task */
  osThreadDef(Servo_Task, Start_Servo_Task, osPriorityHigh, 0, 128);
  Servo_TaskHandle = osThreadCreate(osThread(Servo_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_UART_Task */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_UART_Task */
void Start_UART_Task(void const * argument)
{
  /* USER CODE BEGIN Start_UART_Task */
  /* Infinite loop */
  for(;;)
  {
		GetMpuData();
		AngleCalculate();
		
    osDelay(5);
  }
  /* USER CODE END Start_UART_Task */
}

/* USER CODE BEGIN Header_Start_Servo_Task */
/**
* @brief Function implementing the Servo_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Servo_Task */
void Start_Servo_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Servo_Task */
	uint8_t origin[14]={0x30,0x90,0x90,0x90,0x90,0x90,0x90,0x90,0x90,0x90,0x90,0x90,0x90,0x31};
	memcpy(remote.servos,origin,14);
  /* Infinite loop */
  for(;;)
  {
		static uint8_t i;
		static int16_t last_yaw;
		int16_t tmp_yaw=0;
		int16_t tmp_pitch=0;
		static float delta_pitch,last_pitch;
		//
		remote.yaw=(int16_t)g_fCarAngle_yaw; //陀螺仪控制串口舵机数据
//		remote.pitch=(int16_t)g_fCarAngle_pitch;
//		
//		delta_pitch=(g_fCarAngle_pitch-last_pitch)*3;
		
//		if(delta_pitch<2&&delta_pitch>-2)
//			delta_pitch=0.01;
		
		tmp_yaw=(uint8_t)remote.servos[1]+(remote.yaw-last_yaw)*4;
		tmp_pitch=90+(int16_t)(g_fCarAngle_pitch)*3;
		
		CLAMP(&tmp_yaw,0,180);
		CLAMP(&tmp_pitch,0,180);
		
		remote.servos[1]=tmp_yaw;
		remote.servos[2]=tmp_pitch;
		
		last_yaw=remote.yaw;
		//
		memcpy(tmp_buf,&remote,sizeof(tmp_buf));
		
		if(NRF24L01_TxPacket(tmp_buf)==TX_OK)
    {
      printf("NRF24L01无线模块数据发送成功：\r\n");
			for(i=0;i<33;i++)
				printf("%u ",tmp_buf[i]);
			printf("\r\n"); //串口打印的数据
			
    }
    else
    {
      printf("NRF24L01无线模块数据发送失败\r\n");
    } 
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		
    osDelay(50);
  }
  /* USER CODE END Start_Servo_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//串口接收完成回调函数
{
	if(huart->Instance == USART1)
	{
	  memcpy(remote.servos,servos,14);
		HAL_UART_Receive_IT(&huart1, servos,14);
	}
}

/* USER CODE END Application */

