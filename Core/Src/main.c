/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "uart_trace.c"
#include "vl53l0x_api.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int status=0;
uint16_t position_courante=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t data[] = "Message \r\n";
uint8_t Rx_data[6];  //  creating a buffer of 10 bytes
double Voltage_value;
double Current_value;
double Power_value;

//PID
double order=126;
double commande;
double error;
double last_error;
double sum_error;


double Consigne = 110; //En cm par rapport au sol !!
double Mesure_sortie; //Mesure du capteur conversion en cm !!!
double error; //Ecart : error = Consigne - Mesure_sortie
double Kp=2; //Gain correcteur proportionel
double Ki=0.002; //Gain correcteur intégrale
double Kd = 50;

double up;
double ui;


char str[1024];
uint32_t adc_buffer[2];
int position;
int position_actuel_masse;
float prev_error= 0.0;
int apply_correction = 0;


double get_current(	uint32_t readValue){

	double current;

	current = (float)readValue*(3.3/4095.0);
	current/=(23 * 0.05);
	sprintf(str,"Courant : valeur releve : %ld - valeur courant : %.5f\r\n",readValue,current);
//	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);

	Current_value = current;
	return current;
}

float Recevoir_tension_SHUNT(ADC_HandleTypeDef* hadc, UART_HandleTypeDef* huart) {
	 // ADC CONVERTER
	 float voltage;

	 HAL_ADC_Start(hadc);
	 /* Attendre la fin de la conversion ADC */
	 HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);

	 /* Lisez la valeur brute */
	 uint16_t raw_value = HAL_ADC_GetValue(hadc);

	 /* Convertissez la valeur brute en tension (supposant une résolution de 12 bits et une tension de référence de 3.3 V) */
	 voltage = (raw_value / 4095.0) * 4.6;

	 /* Envoyez la tension via USART2 */
	 char buffer[50];
	 sprintf(buffer, "voltage=%.2f\r\n", voltage);
	 HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
	 return voltage;
}

double get_voltage(uint32_t readValue){

	double voltage;
	voltage = (float)readValue*(3.3/4095.0);
	sprintf(str,"Tension : valeur releve : %ld - valeur voltage : %.5f\r\n",readValue,voltage);
//	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);

	Voltage_value = voltage;
	return voltage;
}


void Correcteur(UART_HandleTypeDef* huart){
 // Variable pour stocker l'erreur précédente
 Mesure_sortie = position_actuel_masse;
 error = Consigne - Mesure_sortie;
 up = Kp * error;

 ui = ui + up * Ki;

 // Ajout du terme de dérivée (D)
 float derivative = Kd * (error - prev_error);
 prev_error = error;

 //commande = up + ui + derivative; //ne pas enlver please !!!
 commande = up + ui + derivative;
 commande =commande >25 ? 25 : commande;
 commande =commande <0 ? 0 : commande;
 char buffer[50];
 sprintf(buffer, "Commande=%.2f\r\n", commande);
 HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

 TIM1->CCR1 = commande; //Valeurs de 0 à 40 Max
}


int get_position(UART_HandleTypeDef* huart){

 uint16_t position;

 /* only one sensor */
 /* Call All-In-One blocking API function */
 status = VL53L0X_PerformSingleRangingMeasurement(&VL53L0XDevs[0],&RangingMeasurementData);
 position=RangingMeasurementData.RangeMilliMeter;
 if( status ==0 ){
 trace_printf("\r\n%d", position_courante);
 // Sensor_SetNewRange(&VL53L0XDevs[0],&RangingMeasurementData);
 }
 sprintf(str,"position_masse_en_mm=%d\r\n",position);
 HAL_UART_Transmit(huart, (uint8_t *)str, strlen(str), 100);

 return position;
}


void send_data(){
	//send data position courant tension et puissance par uart
	sprintf(str,"distance_data=%u;voltage_data=%f;current_data=%f\r\n",position_courante,Voltage_value, Current_value);
	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
	HAL_UART_Receive_IT(&huart2, Rx_data, 5);
	return;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   RangingConfig_e RangingConfig = LONG_RANGE;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start_IT(&htim2);
  TIM1->CCR1=0;
  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart2, Rx_data, 5);


  XNUCLEO53L0A1_hi2c = hi2c1;
  ResetAndDetectSensor(0); // UNCOMMENT THIS
  /* Reset and Detect all sensors */
  ResetAndDetectSensor(0); // UNCOMMENT THIS
  SetupSingleShot(RangingConfig);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	//HAL_ADC_Start_DMA(&hadc1, adc_buffer, 2);
	  // Recevoir_tension_SHUNT(&hadc1,&huart2);
	  if (apply_correction){
		  position_actuel_masse = get_position(&huart2);
		  Correcteur(&huart2);
		  Recevoir_tension_SHUNT(&hadc1,&huart2);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 830;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10001-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|RESET_VL53_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 RESET_VL53_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|RESET_VL53_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim2 )
  {
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
     // Recevoir_tension_SHUNT(&hadc1,&huart2);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Conversion Complete & DMA Transfer Complete As Well
    // So The AD_RES Is Now Updated & Let's Move IT To The PWM CCR1
    // Update The PWM Duty Cycle With Latest ADC Conversion Result
	if(hadc == &hadc1){
		get_current(adc_buffer[0]);
		get_voltage(adc_buffer[1]);
	}

}


double get_value_from_digits()
{
	int numDigits = Rx_data[1];
	// Assuming Rx_data[2], Rx_data[3], Rx_data[4], ..., Rx_data[numDigits + 1] contain the digits
	int receivedValue = 0;
	for (int i = 0; i < numDigits; ++i) {
		receivedValue = receivedValue * 10 + Rx_data[2 + i];
	}
	return receivedValue;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2){
		 // Re-arm the UART receive interrupt for the next data reception
		 HAL_UART_Receive_IT(&huart2, Rx_data, 5);
		 // HAL_UART_Receive_IT(&huart2, Rx_data, 5);
		 // HAL_UART_Transmit(&huart2, Rx_data, strlen(Rx_data), 100);
		 for(int i=0; i<5; i++){ //Passage des valeurs en entier en enlevant l'offset ascii
			 Rx_data[i]-=48;
		 }
		 if(Rx_data[0]==0x1){ //Recevoir Kp sur 4 chiffres entiers ( jusqu'à 9999 )
			 Kp= get_value_from_digits();
		 } else if(Rx_data[0]==0x2){ //Recevoir Ki sur 4 chiffres entiers ( jusqu'à 9999 )
			 Ki=get_value_from_digits();
		 } else if(Rx_data[0]==0x3){ //Recevoir Kd sur 4 chiffres entiers ( jusqu'à 9999 )
			 Kd=get_value_from_digits();
		 } else if(Rx_data[0]==0x4){ //Recevoir distance sur 3 chiffres entiers ( jusqu'à 999 mm mais bloqué par IHM à 200 mm)
			 Consigne=get_value_from_digits();
		 } else if (Rx_data[0]==0x5){
			 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		 } else if (Rx_data[0]==0x6){
			 HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		 } else if(Rx_data[0]==0x7){ //afficher la tension
//			 HAL_TIM_Base_Start_IT(&htim2);
			 Recevoir_tension_SHUNT(&hadc1,&huart2);
		 } else if(Rx_data[0]==0x8){ //Ne plus afficher la tension
//			 HAL_TIM_Base_Stop_IT(&htim2);
//			 HAL_TIM_Base_Stop(&htim2);
//			 HAL_TIM_Base_Stop_DMA(&htim2);
		 }else if(Rx_data[0]==0x9){ // Show corrction values or not
			 apply_correction = ! apply_correction;
		 }else{
			 return;
		 }

	}

}
/*
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
   HAL_UART_Transmit_IT(&huart2, fin, sizeof(fin));
}
*/

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
