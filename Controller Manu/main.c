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

#define D1_LOW 		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,0);
#define D2_LOW 		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,0);
#define D3_LOW 		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,0);
#define D4_LOW 		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0);
#define D1_HIGH 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,1);
#define D2_HIGH 	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,1);
#define D3_HIGH		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,1);
#define D4_HIGH 	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart2;

int count =	1324;

int manu = 0;

//char num[10] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90};

int latter[100] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,85,59,60,61,62,63,64,0x88,0x83,0xC6,0xA1,0x86,0x8E,0xC2,0x89,0xF9,0xE1,0x8D,0xC7,0xAA,0xAB,0xA3,0x8C,0x98,0xAF,0x92,0x87,0xC1,0xE3,87,0xFF,0x91};

int temp1=0xFF, temp2=0xFF, temp3=0xFF, temp4=0xC0;

int *ptrtemp1 = &temp1;
int *ptrtemp2 = &temp2;
int *ptrtemp3 = &temp3;
int *ptrtemp4 = &temp4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void update_manu(int n1, int n2, int n3, int n4);

void update_numbers(int p);

void Printdata(unsigned char data)  // data = 8bit hexadecimal value
{
	// zero bit A = PA10
	if((data & 0x01) == 0x01){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	}

	// first bit B = PB3
	if((data & 0x02) == 0x02){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	}

	// Second bit C = PB5
	if((data & 0x04) == 0x04){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	}

	// third bit D = PB4
	if((data & 0x08) == 0x08){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	}

	// fourth bit E = PB10
	if((data & 0x10) == 0x10){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	}

	// fifth bit F = PA8
	if((data & 0x20) == 0x20){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	}

	// zero bit G = PA9
	if((data & 0x40) == 0x40){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	}

	// zero bit DP = PC7
	if((data & 0x80) == 0x80){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//******* display update manu *********************

void update_manu(int n1, int n2, int n3, int n4)
{
  *ptrtemp1 = latter[n1];
  *ptrtemp2 = latter[n2];
  *ptrtemp3 = latter[n3];
  *ptrtemp4 = latter[n4];
}

void update_numbers(int p){
  *ptrtemp1 = latter[p/1000];
  *ptrtemp2 = latter[((p/100)%10)];
  *ptrtemp3 = latter[((p/10)%10)];
  *ptrtemp4 = latter[(p%10)];
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  	  // digit:~1
  	  if(temp1 == 0){
  		D1_LOW;
  	  }else
  	  {
      Printdata(temp1);
  	  D1_HIGH;
  	  HAL_Delay(2);
  	  D1_LOW;
  	  }

  	  // digit:~2
  	  if((temp2 == 0) & (temp1 == 0)){
  		D2_LOW;
  	  }else
  	  {
  	  Printdata(temp2);
  	  D2_HIGH;
  	  HAL_Delay(2);
  	  D2_LOW;
  	  }

  	  // digit:~3
  	  if((temp1 == 0) & (temp2 == 0) & (temp3 == 0)){
  		D3_LOW;
  	  }else
  	  {
  	  Printdata(temp3);
  	  D3_HIGH;
  	  HAL_Delay(2);
  	  D3_LOW;
  	  }

  	  // digit:~4
  	  Printdata(temp4);
  	  D4_HIGH;
  	  HAL_Delay(2);
  	  D4_LOW;

  	  HAL_Delay(1);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D4_Pin|D3_Pin|D2_Pin|F_Pin
                          |G_Pin|A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, E_Pin|B_Pin|D_Pin|C_Pin
                          |D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D3_Pin D2_Pin F_Pin
                           G_Pin A_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D3_Pin|D2_Pin|F_Pin
                          |G_Pin|A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : E_Pin B_Pin D_Pin C_Pin
                           D1_Pin */
  GPIO_InitStruct.Pin = E_Pin|B_Pin|D_Pin|C_Pin
                          |D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DP_Pin */
  GPIO_InitStruct.Pin = DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DP_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//******** interrupt callback *********************
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	  if(GPIO_Pin == GPIO_PIN_13) {
		  manu++;

		  switch(manu){
		  case 1: //print PARA
			  update_manu('P','A','R','A');
			  for(int i=0; i<=1000; i++);
			  break;
		  case 2: //print INPT
			  update_manu('I','N','P','T');
			  for(int i=0; i<=1000; i++);
			  break;
		  case 3: //print 4-20
			  update_numbers(4020);
			  for(int i=0; i<=1000; i++);
			  break;
		  case 4: //print DP
			  update_manu('X','X','D','P');
			  for(int i=0; i<=1000; i++);
			  break;
		  case 5: //print 0000
			  update_numbers(0);
			  for(int i=0; i<=1000; i++);
			  break;
		  case 6: //print LRNG
			  update_manu('L','R','N','G');
			  for(int i=0; i<=1000; i++);
			  break;
		  case 7: //print 0000
			  update_numbers(45);
			  for(int i=0; i<=1000; i++);
			  break;
		  case 8: //print HRNG
			  update_manu('H','R','N','G');
			  for(int i=0; i<=1000; i++);
			  break;
		  case 9: //print 1440
			  update_numbers(1440);
			  for(int i=0; i<=1000; i++);
			  break;
		  case 10: //print FLTR
			  update_manu('F','L','T','R');
			  for(int i=0; i<=1000; i++);
			  break;
		  case 11: //print 5
			  update_numbers(5);
			  for(int i=0; i<=1000; i++);
			  break;
		  case 12: //print CRFC
			  update_manu('C','R','F','C');
			  for(int i=0; i<=1000; i++);
			  break;
		  case 13: //print 80
			  update_numbers(806);
			  for(int i=0; i<=1000; i++);
			  break;
		  case 14: //print SLL
			  update_manu('X','S','L','L');
			  for(int i=0; i<=1000; i++);
			  break;
		  case 15: //print 35
			  update_numbers(35);
			  for(int i=0; i<=1000; i++);
			  break;
		  }
		  if(manu>15){
			  manu=1;
		  }

	  } else {
	      __NOP();
	  }
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
