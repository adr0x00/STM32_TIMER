/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

 /* Логика программы для CD-ROM
  * - Опрос кнопок через прерывание , а не постоянный в программме.
  * - Нажали кнопку и если сигнал open то открываем , ну и анологично закрытию.
  * - во время открытия мигаем светодиодом
  * - M1-M2 - 10 -закрыть , 01 - открыть.
  * - Добавить I2C дислей и набор кнопок.
  * */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY 20   // мс, время антидребезга
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

typedef enum {
    STOPPED,
    RUNNING
} TimerState;

TimerState state = STOPPED; // СОСТОЯНИЕ ПРОГРАММЫ


/* USER CODE END PV */
uint8_t minutes = 0;    // минуты
uint8_t seconds = 0;    // секунды

// СОСТОЯНИЕ КНОПКИ
uint8_t current_state = 0;
uint8_t last_state    = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

void display_time_update(void);// Обновление дисплея
void display_user_enter(void); // Хелло для пользователя
void Button_Minutes(void);
void Button_Seconds(void);
void Button_StartStop(void);
uint8_t Button_Tick( uint16_t GPIO_Pin );

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HD44780_Init(2);

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  /* ЛОГИКА :
	   * ОПРОС КНОПОК , ЕСЛИ НАЖАЛИ ТО ПРОГРАММА МЕНЯЕТСЯ
	   * ОТСЧЕТ ЗАДАННОГО ВРЕМЕНИ
	   * ПО ОКОНЧАНИЮ ВЫВОДИМ СИГНАЛ НА ПИН КОТОРЫЙ УПРАВЛЯЕТ ДРАЙВЕРОМ
	   * */


	     if (   Button_Tick(BUTTON_MINUTES_PIN) ){
			    Button_Minutes();
		  }

		  if (  Button_Tick (BUTTON_SECONDS_PIN) ){
		  	    Button_Seconds();
		  }

		  if (  Button_Tick(BUTTON_START_STOP_PIN ) ){
			    Button_StartStop();
		  }

		  if (state == RUNNING) {
			     // ЗДЕСЬ ПРОИСХОДИТ ВСЕ ВОВРЕМЯ ОТСЧЕТА
			   display_time_update();
			   HAL_GPIO_WritePin(GPIOA , DRIVER_PIN , GPIO_PIN_SET);  // ВКЛЮЧАЕМ ЛАМПУ
			        if (seconds == 0) {
			             if ( minutes > 0) {
			                  minutes--;
			                  seconds = 59;
			              } else {
			                  state = STOPPED;
			                  // здесь можно включить пищалку или мигание светодиода

			                 HAL_GPIO_WritePin(GPIOA , DRIVER_PIN , GPIO_PIN_RESET);  // ВЫКЛЮЧАЕМ ЛАМПУ
			              }
			          } else {
			              seconds--;

			          }
			      } else {
			    	   // ЗДЕСЬ ПРОИСХОДИТ ВСЕ ЧТО ДО ОТСЧЕТА - ПРИВЕТСВИЕ И ТД
			    	      display_user_enter();
			      }



		  HAL_Delay(1000);        // обновление каждую секунду

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DRIVER_PIN_GPIO_Port, DRIVER_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : DRIVER_PIN_Pin */
  GPIO_InitStruct.Pin = DRIVER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DRIVER_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_START_STOP_Pin BUTTON_SECONDS_Pin BUTTON_MINUTES_Pin */
  GPIO_InitStruct.Pin = BUTTON_START_STOP_PIN|BUTTON_SECONDS_PIN|BUTTON_MINUTES_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void display_time_update(void) {
	// для конвертации из int в char
	 char time_buffer[16];
	 sprintf(time_buffer, "%02d:%02d", minutes, seconds);

	 HD44780_Clear();
	 HD44780_SetCursor(0,1);
	 HD44780_PrintStr("  TIMER:");
	 HD44780_SetCursor(9,1);
	 HD44780_PrintStr(time_buffer);  // ПЕЧАТАЕМ ВРЕМЯ

	 HD44780_Display();

}// обновление дисплея отсчета

void display_user_enter(void) {
	// для конвертации из int в char
	char min_buffer[8];
	char sec_buffer[8];

	// Форматируем минуты и секунды отдельно
	sprintf(min_buffer, "%02d", minutes);
	sprintf(sec_buffer, "%02d", seconds);

	HD44780_Clear();
	HD44780_SetCursor(0,0);
	HD44780_PrintStr(" PLS ENTER TIME ");

	HD44780_SetCursor(0,1);
	HD44780_PrintStr("SEC:");
	HD44780_SetCursor(5,1);
	HD44780_PrintStr(sec_buffer);  // Только секунды

	HD44780_SetCursor(8,1);
	HD44780_PrintStr("MIN:");
	HD44780_SetCursor(13,1);
	HD44780_PrintStr(min_buffer);  // Только минуты
} // обновление дисплея приветсвия

void Button_Minutes(void){

	 if( state == STOPPED){
		 // добавляем минуты
		 minutes = (minutes + 1) % 100; // максимум 99 минут
	 }
}

void Button_Seconds(void){

	 if( state == STOPPED){
	     // добавляем секунды
		 seconds = (seconds + 15) % 60; // шаг 15 секунд
     }

}

void Button_StartStop(void){

	 if( state == STOPPED && (minutes > 0 || seconds > 0)){

		 state = RUNNING; // Запускаем таймер.
	 }  else {

		 state = STOPPED;  // останавливаем
	 }

}

uint8_t Button_Tick( uint16_t GPIO_Pin ){

	    static uint32_t lastTick = 0;
	    static uint8_t lastState = 1;  // 1 = отпущена (pull-up)

	    uint8_t currentState;

	    currentState = HAL_GPIO_ReadPin(GPIOA, GPIO_Pin);

	    if (currentState != lastState) {
	        // если состояние изменилось → проверим по таймеру
	        if (HAL_GetTick() - lastTick > DEBOUNCE_DELAY) {
	            lastTick = HAL_GetTick();
	            lastState = currentState;

	            if (currentState == 0) {
	                return 1; // кнопка нажата
	            }
	        }
	    }

	    return 0; // кнопка не нажата
}
/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
