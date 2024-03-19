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


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();

	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
		//-----------------Setup LED----------------
	//Set PC6-9 to General Purpose Output Mode
	GPIOC->MODER &= ~(1 << 19);	
	GPIOC->MODER |= (1 << 18);
	GPIOC->MODER &= ~(1 << 17);
	GPIOC->MODER |= (1 << 16);
	GPIOC->MODER &= ~(1 << 15);	
	GPIOC->MODER |= (1 << 14);
	GPIOC->MODER &= ~(1 << 13);
	GPIOC->MODER |= (1 << 12);
	
	//Set PC6-9 to Push-Pull
	GPIOC->OTYPER &= ~(1 << 9);
	GPIOC->OTYPER &= ~(1 << 8);
	GPIOC->OTYPER &= ~(1 << 7);
	GPIOC->OTYPER &= ~(1 << 6);
	//Set PC6-9 to LowSpeed 
	GPIOC->OSPEEDR &= ~(1 << 18);
	GPIOC->OSPEEDR &= ~(1 << 16);
	GPIOC->OSPEEDR &= ~(1 << 14);
	GPIOC->OSPEEDR &= ~(1 << 12);
	//Set PC6-9 to No pull up/down Resistor
	GPIOC->PUPDR &= ~(1 << 19);
	GPIOC->PUPDR &= ~(1 << 18);
	GPIOC->PUPDR &= ~(1 << 17);
	GPIOC->PUPDR &= ~(1 << 16);
	GPIOC->PUPDR &= ~(1 << 15);
	GPIOC->PUPDR &= ~(1 << 14);
	GPIOC->PUPDR &= ~(1 << 13);
	GPIOC->PUPDR &= ~(1 << 12);
	//Set LEDS to low
	GPIOC->ODR &= ~(1 << 6); // red
	GPIOC->ODR &= ~(1 << 7); // blue
	GPIOC->ODR &= ~(1 << 8); // orange
	GPIOC->ODR &= ~(1 << 9); // green
	 
	 
	//Set PC4 to Analog Function, ADC_IN10
	GPIOC->MODER |= GPIO_MODER_MODER4; 
	
	//PC4 no pull up or down 
	GPIOC->OTYPER |= GPIO_OTYPER_OT_4;
	
	
	//PA4 to analog mode
	GPIOA->MODER |= GPIO_MODER_MODER4; 
	
	//PA4 no pull up or down 
	GPIOA->OTYPER |= GPIO_OTYPER_OT_4;
	
	//set bit 13 for conitionus conversion
	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	
	//set b'10 in bits [4:3] to get 8 bit resoulution 
	ADC1->CFGR1 |= ADC_CFGR1_RES_1;
	ADC1->CFGR1 &= ~ADC_CFGR1_RES_0;
	
	//set b'00 for bits[11:10] to disable hardware triggers 
	ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;
	
	//adc_Chslher set bit 10 to enable channel 10 
	ADC1->CHSELR |= ADC_CHSELR_CHSEL14;
	
	//FROM A.7.1
	//--------------------------ADC calibration-----------------
	/* (1) Ensure that ADEN = 0 */
	/* (2) Clear ADEN by setting ADDIS*/
	/* (3) Clear DMAEN */
	/* (4) Launch the calibration by setting ADCAL */
	/* (5) Wait until ADCAL=0 */
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
	ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
	/* For robust implementation, add here time-out management */
	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
	ADC1->CR |= ADC_CR_ADCAL; /* (4) */
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */
	{}
		
	//---------------start the adc -------------

	
	//Set ADRDY
	ADC1->ISR |= ADC_ISR_ADRDY;
	//SET ADEN
	ADC1->CR |= ADC_CR_ADEN;
	//Wait for ADRDY 
	while((ADC1->ISR & ADC_ISR_ADRDY)!= 0)
	{
		//KEEP setting DEN 
		ADC1->CR |= ADC_CR_ADEN;
	}
	//start the adc 
	ADC1->CR |= ADC_CR_ADSTART;
	
	//--------------Enable DAC---------------
	
	//Set to software trigger 
	DAC1->CR |= DAC_CR_TSEL1;
	
	//DAC channel 1 Enable 
	DAC1->CR |= DAC_CR_EN1;
	
	// Sine Wave: 8-bit, 32 samples/cycle
	const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
	232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
	// Triangle Wave: 8-bit, 32 samples/cycle
	const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
	190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};
	int index = 0;
  while (1)
  {
		//Output wave form
		DAC1->DHR8R1 = triangle_table[index];
		index++;
		if(index >= 32)
		{
			index = 0;
		}
		HAL_Delay(1);
		//-------Check voltage thresholds and set LEDS------------
		int voltage = ADC1->DR;
		
		if(voltage > 15)
		{
			GPIOC->ODR |= (1 << 6); //red on
		}
		else 
		{	
			GPIOC->ODR &= ~(1 << 6); //red off
		}
		if(voltage > 50)
		{
			GPIOC->ODR |= (1 << 7); // blue on
		}
		else 
		{
			GPIOC->ODR &= ~(1 << 7); // blue off
		}
		if(voltage > 75)
		{
			GPIOC->ODR |= (1 << 8); // orange
		}
		else 
		{
			GPIOC->ODR &= ~(1 << 8); // orange
		}	
		if(voltage > 100)
		{
			GPIOC->ODR |= (1 << 9); // green
		}
		else 
		{
			GPIOC->ODR &= ~(1 << 9); // green
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
