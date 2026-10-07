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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END Variables */
/* Definitions for MOTOR */
osThreadId_t MOTORHandle;
const osThreadAttr_t MOTOR_attributes = {
  .name = "MOTOR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for VOFATx */
osThreadId_t VOFATxHandle;
const osThreadAttr_t VOFATx_attributes = {
  .name = "VOFATx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for VOFARx */
osThreadId_t VOFARxHandle;
const osThreadAttr_t VOFARx_attributes = {
  .name = "VOFARx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uart_rx */
osMessageQueueId_t uart_rxHandle;
const osMessageQueueAttr_t uart_rx_attributes = {
  .name = "uart_rx"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void DJ_MOTOR(void *argument);
void VOFA(void *argument);
void UART(void *argument);

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
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uart_rx */
  uart_rxHandle = osMessageQueueNew (128, sizeof(uint8_t), &uart_rx_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MOTOR */
  MOTORHandle = osThreadNew(DJ_MOTOR, NULL, &MOTOR_attributes);

  /* creation of VOFATx */
  VOFATxHandle = osThreadNew(VOFA, NULL, &VOFATx_attributes);

  /* creation of VOFARx */
  VOFARxHandle = osThreadNew(UART, NULL, &VOFARx_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_DJ_MOTOR */
/**
  * @brief  Function implementing the MOTOR thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DJ_MOTOR */
__weak void DJ_MOTOR(void *argument)
{
  /* USER CODE BEGIN DJ_MOTOR */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DJ_MOTOR */
}

/* USER CODE BEGIN Header_VOFA */
/**
* @brief Function implementing the VOFATx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VOFA */
__weak void VOFA(void *argument)
{
  /* USER CODE BEGIN VOFA */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END VOFA */
}

/* USER CODE BEGIN Header_UART */
/**
* @brief Function implementing the VOFARx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART */
__weak void UART(void *argument)
{
  /* USER CODE BEGIN UART */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UART */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

