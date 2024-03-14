/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"

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
/* Definitions for Task01 */
osThreadId_t Task01Handle;
const osThreadAttr_t Task01_attributes = {
  .name = "Task01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task02 */
osThreadId_t Task02Handle;
const osThreadAttr_t Task02_attributes = {
  .name = "Task02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Button */
osThreadId_t ButtonHandle;
const osThreadAttr_t Button_attributes = {
  .name = "Button",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Event1 */
osEventFlagsId_t Event1Handle;
const osEventFlagsAttr_t Event1_attributes = {
  .name = "Event1"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Task01Function(void *argument);
void Task02Function(void *argument);
void ButtonFunction(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void freertos_delay(uint32_t milliseconds) {
    const TickType_t delay_ticks = pdMS_TO_TICKS(milliseconds);
    TickType_t startTime = xTaskGetTickCount(); ///START TIME
    while ((xTaskGetTickCount() - startTime) < delay_ticks) {
        if ((GPIOC->IDR & (1 << 13)) == 0) {
        	osEventFlagsSet(Event1Handle,2);
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);
  SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOCEN);

    GPIOA->MODER &= 0xfffff3ff;//LED
    GPIOA->MODER |= 0x00000400;//LED
    GPIOA->OTYPER &= 0xffdf;
    GPIOA->PUPDR &= 0xfffff3ff;
    GPIOA->PUPDR |= 0x00000800;
    GPIOA->ODR &= 0xffcf;

    GPIOB->MODER &= 0xfffff3ff;//LED
    GPIOB->MODER |= 0x00000400;//LED
    GPIOB->OTYPER &= 0xffdf;
    GPIOB->PUPDR &= 0xfffff3ff;
    GPIOB->PUPDR |= 0x00000800;
    GPIOB->ODR &= 0xffcf;

    GPIOC->MODER &= 0xf4ffffff;///BUTTON
    GPIOC->OTYPER &= 0xbfff;//BUTTON
    GPIOC->PUPDR &= 0xf3ffffff;
    GPIOC->PUPDR |= 0xfbffffff;

    GPIOC->MODER &= 0xfffff3ff;//LED
    GPIOC->MODER |= 0x00000400;//LED
    GPIOC->OTYPER &= 0xffdf;
    GPIOC->PUPDR &= 0xfffff3ff;
    GPIOC->PUPDR |= 0x00000800;
    GPIOC->ODR &= 0xffcf;
    GPIOC ->ODR |= (1<<5); /// verde on
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task01 */
  Task01Handle = osThreadNew(Task01Function, NULL, &Task01_attributes);

  /* creation of Task02 */
  Task02Handle = osThreadNew(Task02Function, NULL, &Task02_attributes);

  /* creation of Button */
  ButtonHandle = osThreadNew(ButtonFunction, NULL, &Button_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of Event1 */
  Event1Handle = osEventFlagsNew(&Event1_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task01Function */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task01Function */
void Task01Function(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task02Function */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task02Function */
void Task02Function(void *argument)
{
  /* USER CODE BEGIN Task02Function */
  /* Infinite loop */
  for(;;)
  {
	  if(osEventFlagsGet(Event1Handle)==1 || osEventFlagsGet(Event1Handle)==2 || osEventFlagsGet(Event1Handle)==3)
	  {
		GPIOA->ODR &= 0xffcf;///RED OFF
		GPIOB->ODR &= 0xffcf;
		GPIOB->ODR |= (1 << 5);//YELLOW ON
		GPIOC->ODR &= 0xffcf;///GREEN OFF
		freertos_delay(3000);

		GPIOA->ODR &= 0xffcf;///RED ON
		GPIOA->ODR |= (1 << 5);
		GPIOB->ODR &= 0xffcf;///YELLOW OFF
		GPIOC->ODR &= 0xffcf;///GREEN OFF
		freertos_delay(20000);
		osEventFlagsClear(Event1Handle,2);

	    GPIOA->ODR &= 0xffcf;///RED OFF
	    GPIOB->ODR &= 0xffcf;
	    GPIOC->ODR &= 0xffcf;
	    GPIOC->ODR |= (1 << 5);///GREEN ON
	    freertos_delay(15000);
	  osEventFlagsClear(Event1Handle,1);
      }
	  osDelay(1);
  }
  /* USER CODE END Task02Function */
}

/* USER CODE BEGIN Header_ButtonFunction */
/**
* @brief Function implementing the Button thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ButtonFunction */
void ButtonFunction(void *argument)
{
  /* USER CODE BEGIN ButtonFunction */
  /* Infinite loop */
  for(;;)
  {
	  int currentState = (GPIOC->IDR & (1 << 13));
	  if(currentState==0)
	  {
		  osEventFlagsSet (Event1Handle, 1);
	  }
    osDelay(1);
  }
  /* USER CODE END ButtonFunction */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
