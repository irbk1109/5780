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

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Transmit_Character(char value)
{
	while (1)
  {
		unsigned int status = USART3->ISR;
		status &= USART_ISR_TXE; //check transmission end flag 
		if(status != 0) 	
		{
			USART3->TDR = value;
			break;
		}
  }
}

void Transmit_String(char* string)
{
	int i = 0;
	while(1)
	{
		if(string[i] == NULL) 
		{
			break;
		}
		else 
		{
			Transmit_Character(string[i]);
			i++;
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
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

	
	//Setup RCC clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	GPIOC->MODER |= (1 << 23);
	GPIOC->MODER &= ~(1 << 22);
	GPIOC->MODER |= (1 << 21);
	GPIOC->MODER &= ~(1 << 20);
	
	//Select alternate functions for USART3
	GPIOC->AFR[1] &= ~(1 << 15);
	GPIOC->AFR[1] &= ~(1 << 14);
	GPIOC->AFR[1] &= ~(1 << 13);
	GPIOC->AFR[1] |= (1 << 12);
	GPIOC->AFR[1] &= ~(1 << 11);	
	GPIOC->AFR[1] &= ~(1 << 10);
	GPIOC->AFR[1] &= ~(1 << 9);
	GPIOC->AFR[1] |= (1 << 8);
	
	//Enable USART 3 
	USART3->CR1 |= (1 << 3);
	USART3->CR1 |= (1 << 2);
	USART3->CR1 |= (1 << 0);
	USART3->BRR |= HAL_RCC_GetHCLKFreq() / 9600; // get baud rate of 115200
	//Setup LEDS
	
	//General Purpose Output Mode 
	GPIOC->MODER &= ~(1 << 19);	
	GPIOC->MODER |= (1 << 18);
	GPIOC->MODER &= ~(1 << 17);
	GPIOC->MODER |= (1 << 16);
	GPIOC->MODER &= ~(1 << 15);
	GPIOC->MODER |= (1 << 14);	
	GPIOC->MODER &= ~(1 << 13);
	GPIOC->MODER |= (1 << 12);
	//Set LEDS to Push-Pull
	GPIOC->OTYPER &= ~(1 << 9);
	GPIOC->OTYPER &= ~(1 << 8);
	GPIOC->OTYPER &= ~(1 << 7);
	GPIOC->OTYPER &= ~(1 << 6);
	
	//Set LEDS to LowSpeed 
	GPIOC->OSPEEDR &= ~(1 << 18);
	GPIOC->OSPEEDR &= ~(1 << 16);
	GPIOC->OSPEEDR &= ~(1 << 14);
	GPIOC->OSPEEDR &= ~(1 << 12);
	
	//Set LEDS to No pull up/down Resistor
	GPIOC->PUPDR &= ~(1 << 19);
	GPIOC->PUPDR &= ~(1 << 18);
	GPIOC->PUPDR &= ~(1 << 17);
	GPIOC->PUPDR &= ~(1 << 16);
	GPIOC->PUPDR &= ~(1 << 15);
	GPIOC->PUPDR &= ~(1 << 14);
	GPIOC->PUPDR &= ~(1 << 13);
	GPIOC->PUPDR &= ~(1 << 12);
	GPIOC->ODR &= ~(1 << 6);
	GPIOC->ODR &= ~(1 << 7);
	GPIOC->ODR &= ~(1 << 8);
	GPIOC->ODR &= ~(1 << 9);
	//Test Transmit Strings 
//	int x = 0;
//	char  string[] = {'a', 'b', 'c', 'd', 'e'};
//  while (1)
//  {
//		x++;
//		if(x == 150000)
//			Transmit_String(string);
//  } 

	while(1)
	{
		char errorMsg[] = {'e', 'r', 'r', 'o', 'r', NULL};
		//if we data is read to be read 
		if((USART3->ISR & USART_ISR_RXNE) != 0)
		{
			char value = USART3->RDR & 0xFF;
			switch(value){
				case 'r':
					GPIOC->ODR ^= (1 << 6);
					break;
				case 'b':
					GPIOC->ODR ^= (1 << 7);
					break;
				case 'g':
					GPIOC->ODR ^= (1 << 9);
					break;
				case 'o':
					GPIOC->ODR ^= (1 << 8);
					break;
				default:
					Transmit_String(errorMsg);
				}		
		}
	}
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
