/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BufferLen 4096

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



unsigned char RecBuffer[BufferLen] ;
unsigned int Buffer_EOF_Flag = 0;

unsigned int BufferIndex = 0 ;		// (BufferIndex)max <= BufferLen

unsigned char key1Down;						// <- as Bool
unsigned char key2Down;						// <- as Bool

unsigned char Rec_Status = 0 ; 		// <- as Bool
unsigned char Play_Status = 0 ;   // <- as Bool

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

inline void RewindBufferIndex_GetEOFPoint() { 
	Buffer_EOF_Flag = BufferIndex;
	BufferIndex = 0;
}

inline void LED1_ON() {
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_RESET); 
} 

inline void LED1_OFF() {
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, GPIO_PIN_SET); 
} 


inline void LED2_ON() {
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_RESET); 
} 

inline void LED2_OFF() {
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, GPIO_PIN_SET); 
} 


// !!!!!!!!!!!!!!!!!! @ Timer CallBack !!!!!!!!!!!!!!!!!!!!
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	
	
	if(Rec_Status) {
		
		BufferIndex = BufferIndex + 1;

		if (key1Down){ 
			RecBuffer[BufferIndex] = 2;
		}else 
		if (key2Down){
			RecBuffer[BufferIndex] = 1;
		}else{
			RecBuffer[BufferIndex] = 0;
		}
		
	}

	if(Play_Status) {
		
		BufferIndex = BufferIndex + 1;
		
		if(BufferIndex == Buffer_EOF_Flag) {
			BufferIndex = 0;
		}
		if (RecBuffer[BufferIndex] == 0){
			LED1_OFF();
			LED2_OFF();
		}
		if (RecBuffer[BufferIndex] == 2){
			LED1_ON();
		}
		if (RecBuffer[BufferIndex] == 1){
			LED2_ON();
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);																// --> Tim3->On
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	
  while (1)																											// @ mainloop
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		if (HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin) == 0) {
			HAL_Delay(2);
			if (HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin) == 0) {
				Rec_Status = 1;
				key1Down = 1;
				LED1_ON();
			}else{
				key1Down = 0;
				LED1_OFF();
			}
		}
		
		
		if (HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin) == 0) {
			HAL_Delay(2);
			if (HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin) == 0) {
				key2Down = 1;
				LED2_ON();
			}else{
				key2Down = 0;
				LED2_OFF();
			}
		}


		if (HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin) == 1) {
			HAL_Delay(5);
			if (HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin) == 1) {
				RewindBufferIndex_GetEOFPoint();
				Rec_Status = 0;
				Play_Status = 1;
				HAL_Delay(1000);
			}
		}

		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
