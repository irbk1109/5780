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
  /* USER CODE BEGIN 2 */
	
	//initalize the RCC clocks for gpio c and b
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	//Initalize I2C clock
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
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
	
	
	//-----------------Setup PB11, PB13, PB14, PC0----------------
	//Choose alternate function for PB11(I2C2_SDA AF1), PB13(I2C2_SCL AF5)
	GPIOB->AFR[1] &= ~(1 << 15);
	GPIOB->AFR[1] &= ~(1 << 14);
	GPIOB->AFR[1] &= ~(1 << 13);
	GPIOB->AFR[1] |=  (1 << 12);
	
	GPIOB->AFR[1] &= ~(1 << 23);
	GPIOB->AFR[1] |=  (1 << 22);
	GPIOB->AFR[1] &= ~(1 << 21);
	GPIOB->AFR[1] |=  (1 << 20);
	
	//Set alternate function mode PB11 and PB13
	GPIOB->MODER |= (1 << 27);
	GPIOB->MODER &= ~(1 << 26);
	GPIOB->MODER |= (1 << 23);
	GPIOB->MODER &= ~(1 << 22);
	
	//PC0 and PB14 output mode
	GPIOC->MODER |= (1 << 0);
	GPIOC->MODER &= ~(1 << 1);
	GPIOB->MODER &= ~(1 << 29);
	GPIOB->MODER |= (1 << 28);
	
	
	//Open drain PB11 PB13 
	GPIOB->OTYPER |= (1 << 11);
	GPIOB->OTYPER |= (1 << 13);
	
	//Push pull output PC0 PB14
	GPIOB->OTYPER &= ~(1 << 14);
	GPIOC->OTYPER &= ~(1 << 0);
	
	//Set Pb14 and PC0 high 
	GPIOC->ODR |= (1 << 0);
	GPIOB->ODR |= (1 << 14);
	
	//-----------------Set I2C2 TimingRgst to 100k----------------
	
	I2C2->TIMINGR |= (0x1  << 28);  //PRESC = 1
	I2C2->TIMINGR |= (0x13 << 0);  //SCLL   = 0x13
	I2C2->TIMINGR |= (0xF << 8);  //SCLH   = 0xF
	I2C2->TIMINGR |= (0x2  << 16); //SDADEL = 0x2
	I2C2->TIMINGR |= (0x4  << 20); //SCLDEL = 0x4
	
		I2C2->CR1 |= (1 << 0); 
	//-----------------I2C code start------------------------------ 
	//Enable I2C2 periphial using CR1 PE bit 

	
//	//	1. Set the slave address in the SADD[7:1] bit field.
//	I2C2->CR2 |= (0x69 << 1);
//	//	2. Set the number of data byte to be transmitted in the NBYTES[7:0] bit field.
//	I2C2->CR2 |= (1 << 16);	
//	//	4. Do not set the AUTOEND bit,x this lab requires software start/stop operation.
//	I2C2->CR2 &=~(1<<10); 
//	//	5. Setting the START bit to begin the address frame
//	I2C2->CR2 |= (1 << 13);
//	
//	//Wait for TXIS 
//	while(1)
//	{
//		if((I2C2->ISR & I2C_ISR_TXIS))
//		{
//			break;
//		}
//	}
//	
//	I2C2->TXDR = 0x0F;
//	
//	//Wait for transfer Complete
//	while(1)
//	{
//		if((I2C2->ISR & I2C_ISR_TC)){ 
//				
//			break;
//		}
//	}

//	//	1. Set the slave address in the SADD[7:1] bit field.
//	I2C2->CR2 |= (0x69 << 1);
//	//	2. Set the number of data byteto be transmitted in the NBYTES[7:0] bit field.
//	I2C2->CR2 |= (0x01 << 16);	
//	//	3. Configure the RD_WRN to indicate a read/write operation.
//	I2C2->CR2 |= (1 << 10);	
//	//	4. Do not set the AUTOEND bit, this lab requires software start/stop operation.
//	
//	//	5. Setting the START bit to begin the address frame
//	I2C2->CR2 |= (1 << 13);
//	
//	//Wait for Read
//		while(1)
//	{
//		if((I2C2->ISR & I2C_ISR_RXNE))
//		{
//			break;
//		}
//	}
//	//Wait for Transfer Complete
//		while(1)
//	{
//		if((I2C2->ISR & I2C_ISR_TC))
//		{
//			break;
//		}
//	}
//	//Set blue Led if Read Correctly 
//	if(I2C2->RXDR == 0xD3) GPIOC->ODR |= (1 << 7); // blue
//	
//	//Stop The I2C
//	I2C2->CR2 |= (1 << 14);
//	

	//---------------------Write 0xB to control Register of GyroScope -----------------
	//	1. Set the slave address in the SADD[7:1] bit field.
	I2C2->CR2 |= (0x69 << 1);
	//	2. Set the number of data byte to be transmitted in the NBYTES[7:0] bit field.
	I2C2->CR2 |= (1 << 17); 
	I2C2->CR2 &= ~(1 << 16); 
	
	//	3. Configure the RD_WRN to indicate a read/write operation.
	I2C2->CR2 &=~(1<<10); //Write 
	
	//	5. Setting the START bit to begin the address frame
	I2C2->CR2 |= (1 << 13);
	
	//Wait for TXIS 
	while(1)
	{
		if((I2C2->ISR & I2C_ISR_TXIS))
		{
			break;
		}
	}
	//Write Register Address 
	I2C2->TXDR = 0x20;

	while(1)
	{
		if((I2C2->ISR & I2C_ISR_TXIS))
		{
			break;
		}
	}
	//Write Value into register 
	I2C2->TXDR = 0xB;
	//Wait for transfer Complete
	while(1)
	{
		if((I2C2->ISR & I2C_ISR_TC)){ 
				
			break;
		}
	}

	//Stop
	I2C2->CR2 |= (1 << 14);
	
	
	//While Loop 
	int8_t xLow;
	int8_t	xHigh;
	int8_t yLow;
	int8_t yHigh; 
	int16_t X;
	int16_t Y;
	while(1){
		//Create 100 ms delay
	HAL_Delay(100);
	//----------------------Read X---------------------------------------------------------
	
	//1 Bytes 
	I2C2->CR2 |= (1 << 16);	
	I2C2->CR2 &= ~(1 << 17);
		
	I2C2->CR2 &= ~(1 << 10);	//Write

	//	5. Setting the START bit to begin the address frame
	I2C2->CR2 |= (1 << 13);
	
	//Wait for TXIS 
	while(1)
	{
		if((I2C2->ISR & I2C_ISR_TXIS))
		{
			break;
		}
	}
	//Write Register to read both x low and x high at same time
	I2C2->TXDR = 0xA8;
	
	//Wait for transfer Complete
	while(1)
	{
		if((I2C2->ISR & I2C_ISR_TC)){ 
				
			break;
		}
	}

	//2 Data Bits
	I2C2->CR2 |= (1 << 17);	
	I2C2->CR2 &= ~(1 << 16);
	//Read Data
	I2C2->CR2 |= (1 << 10);	 
	//Start
	I2C2->CR2 |= (1 << 13);
	
	//Wait for Read
		while(1)
	{
		if((I2C2->ISR & I2C_ISR_RXNE))
		{
			break;
		}
	}
	//store xLow
	xLow = I2C2->RXDR;
	while(1)
	{
		if((I2C2->ISR & I2C_ISR_RXNE))
		{
			break;
		}
	}
	//Store xHigh 
	xHigh = I2C2->RXDR;
	
	//Wait for Transfer Complete
		while(1)
	{
		if((I2C2->ISR & I2C_ISR_TC))
		{
			break;
		}
	}
	
	//Stop
	I2C2->CR2 |= (1 << 14);
	//----------------------Read Y---------------------------------------------------------

	//	2. Set the number of data byte to be transmitted in the NBYTES[7:0] bit field.
	I2C2->CR2 |= (1 << 16);	
	I2C2->CR2 &= ~(1<<17);
	//	3. Configure the RD_WRN to indicate a read/write operation.
	I2C2->CR2 &= ~(1 << 10);	//Write
	//	4. Do not set the AUTOEND bit,x this lab requires software start/stop operation.
	//	5. Setting the START bit to begin the address frame
	I2C2->CR2 |= (1 << 13);
	
	//Wait for TXIS 
	while(1)
	{
		if((I2C2->ISR & I2C_ISR_TXIS))
		{
			break;
		}
	}
	//Write Register to read both y low and y high at same time
	I2C2->TXDR = 0xAA;
	
	//Wait for transfer Complete
	while(1)
	{
		if((I2C2->ISR & I2C_ISR_TC)){ 
				
			break;
		}
	}

	//2 Data Bits
	I2C2->CR2 |= (1 << 17);	
	I2C2->CR2 &= ~(1 << 16);
	//Read Data
	I2C2->CR2 |= (1 << 10);	 
	//Start
	I2C2->CR2 |= (1 << 13);
	
	//Wait for Read
		while(1)
	{
		if((I2C2->ISR & I2C_ISR_RXNE))
		{
			break;
		}
	}
	//store yLow
	yLow = I2C2->RXDR;
	while(1)
	{
		if((I2C2->ISR & I2C_ISR_RXNE))
		{
			break;
		}
	}
	//Store yHigh 
	yHigh = I2C2->RXDR;
	
	//Wait for Transfer Complete
		while(1)
	{
		if((I2C2->ISR & I2C_ISR_TC))
		{
			break;
		}
	}
	
	//Stop
	I2C2->CR2 |= (1 << 14);
	
	
	//----------------------Output LED based on X and Y ----------------------------------
	
	//Combine the low and high values
	X = (int16_t )(xLow | (xHigh << 8));
	Y =(int16_t )( yLow | (yHigh << 8));

	
	//Check if positive or negative
	if( X > 100)
	{
		GPIOC->ODR &= ~(1 << 8); // orange
		GPIOC->ODR |= (1 << 9); // green
	}
	else if ( X < -100)
	{
		GPIOC->ODR |= (1 << 8); // orange
		GPIOC->ODR &= ~(1 << 9); // green
	}
	
		if( Y > 100)
	{
		GPIOC->ODR |= (1 << 6); // red
		GPIOC->ODR &= ~(1 << 7); // blue
	}
	 else if ( Y < -100)
	{
		GPIOC->ODR &= ~(1 << 6); // red
		GPIOC->ODR |= (1 << 7); // blue
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
