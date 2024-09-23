/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"


#include <stdio.h>
//#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CHANNELS_NUM 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint16_t adcData[ADC_CHANNELS_NUM]; //Массив для не обработанных данных
float adcVoltage[ADC_CHANNELS_NUM];  //Массив для обработаных данных
volatile int current_current;


int time;

volatile uint32_t time_irq = 0;



unsigned long temp; //  Перемен времени для таймера и измерений

unsigned long led;  // Для светодиода индикации
int timeCharger;

int timeDisCharge;

int autos = 1; // Автоматы
//char msg[40];  // Масив для print
uint32_t count;

uint32_t newCount; // сохранение счетчика во flash каждые 10

float Vmin = 3.5;   //3.5 для акб, 0 для ионистра
float Vmax = 4.1;  //4.2 для акб, 5.0 для ионистра
int total_current;  // Общий ток
int num = 1;
int count_Current = 0;  // Счетчик измерений тока

bool chargeFlag = false; // флаг заряд/разряд

int CapacitorF;

int mA; // Средний ток mA

int percent;

int capacity;

char buffer[40];
char buffer2[40];

///test2
uint32_t count2;
bool chargeFlag2 = false; // флаг заряд/разряд
int timeDisCharge2;
int timeCharger2;
volatile int current_current2;
int total_current2;  // Общий ток
int count_Current2 = 0;
int mA2;
int CapacitorF2;
int percent2;

#define FLASH_USER_START_ADDR  ((uint32_t)0x0800F800)   // Начальный адрес флэш-памяти для сохранения переменных
#define VAR1_ADDRESS          (FLASH_USER_START_ADDR )   // Адрес для сохранения переменной 1
#define VAR2_ADDRESS          (FLASH_USER_START_ADDR + 4) // Адрес для сохранения переменной 2

uint32_t var1;
uint32_t var2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void saveVariablesToFlash();
void currMeasurment();
void Vega();

void getVariablesFromFlash();


void param(){

	switch(num){

	case 1:
	//test1
	  sprintf(buffer, "Test1  |  Test2");
	  ssd1306_Fill(Black);
	  ssd1306_SetCursor(13, 1);
	  ssd1306_WriteString(buffer, Font_7x10, White);

	  sprintf(buffer, "uF:%d",CapacitorF);
	  ssd1306_SetCursor(1, 13);
	  ssd1306_WriteString(buffer, Font_7x10, White);

	  sprintf(buffer, "%d%%",percent);
	  ssd1306_SetCursor(15, 23);
	  ssd1306_WriteString(buffer, Font_7x10, White);

	  sprintf(buffer, "cnt:%d", count);
	  ssd1306_SetCursor(1, 43);
	  ssd1306_WriteString(buffer, Font_7x10, White);

	  sprintf(buffer, "V:%.2f", adcVoltage[0]);
	  ssd1306_SetCursor(1, 33);
	  ssd1306_WriteString(buffer, Font_7x10, White);
	  if (chargeFlag) {
	  ssd1306_SetCursor(1, 53);
	  ssd1306_WriteString("Charge", Font_7x10,White);
	  }
	  else{
		  ssd1306_SetCursor(1, 53);
		  ssd1306_WriteString("Dischar", Font_7x10,White);
	  }

	  // test2
	  sprintf(buffer, "uF:%d",CapacitorF2);
	  ssd1306_SetCursor(70, 13);
	  ssd1306_WriteString(buffer, Font_7x10, White);

	  sprintf(buffer, "%d%%",percent2);
	  ssd1306_SetCursor(85, 23);
	  ssd1306_WriteString(buffer, Font_7x10, White);


	  sprintf(buffer, "V:%.2f", adcVoltage[2]);
	  ssd1306_SetCursor(70, 33);
	  ssd1306_WriteString(buffer, Font_7x10, White);

	  sprintf(buffer, "%d", count2);
	  ssd1306_SetCursor(75, 43);
	  ssd1306_WriteString(buffer, Font_7x10, White);

	  if (chargeFlag2) {
	  ssd1306_SetCursor(70, 53);
	  ssd1306_WriteString("Charge", Font_7x10,White);
	  }
	  else{
		  ssd1306_SetCursor(70, 53);
		  ssd1306_WriteString("Dischar", Font_7x10,White);
	  }




	  ssd1306_UpdateScreen();
	  break;

	case 2:

		  sprintf(buffer, "Bitwell");
		  ssd1306_Fill(Black);
		  ssd1306_SetCursor(25, 1);
		  ssd1306_WriteString(buffer, Font_11x18, White);

		  sprintf(buffer, "Charge:%d", timeCharger);
		  ssd1306_SetCursor(1, 20);
		  ssd1306_WriteString(buffer, Font_7x10, White);

		  sprintf(buffer, "Discharge:%d", timeDisCharge);
		  ssd1306_SetCursor(1, 30);
		  ssd1306_WriteString(buffer, Font_7x10, White);

		  sprintf(buffer, "Voltage:%.2f", adcVoltage[0]);
		  ssd1306_SetCursor(1, 40);
		  ssd1306_WriteString(buffer, Font_7x10, White);

		  sprintf(buffer, "Input:%.2f", adcVoltage[1]);
		  ssd1306_SetCursor(1, 50);
		  ssd1306_WriteString(buffer, Font_7x10, White);

		  sprintf(buffer, "%d%%",percent);
		  ssd1306_SetCursor(90, 40);
		  ssd1306_WriteString(buffer, Font_11x18, White);

		  ssd1306_UpdateScreen();
		break;

	case 3:

		  sprintf(buffer, "NEO");
		  ssd1306_Fill(Black);
		  ssd1306_SetCursor(25, 1);
		  ssd1306_WriteString(buffer, Font_11x18, White);

		  sprintf(buffer, "Charge:%d", timeCharger2);
		  ssd1306_SetCursor(1, 20);
		  ssd1306_WriteString(buffer, Font_7x10, White);

		  sprintf(buffer, "Discharge:%d", timeDisCharge2);
		  ssd1306_SetCursor(1, 30);
		  ssd1306_WriteString(buffer, Font_7x10, White);

		  sprintf(buffer, "Voltage:%.2f", adcVoltage[2]);
		  ssd1306_SetCursor(1, 40);
		  ssd1306_WriteString(buffer, Font_7x10, White);

		  sprintf(buffer, "Input:%.2f", adcVoltage[1]);
		  ssd1306_SetCursor(1, 50);
		  ssd1306_WriteString(buffer, Font_7x10, White);

		  sprintf(buffer, "%d%%",percent2);
		  ssd1306_SetCursor(90, 40);
		  ssd1306_WriteString(buffer, Font_11x18, White);
		  ssd1306_UpdateScreen();
		break;

	case 4:
		ssd1306_Fill(White);
		Vega();
		 ssd1306_UpdateScreen();
		break;

	case 0:
		  ssd1306_Fill(Black);
		  ssd1306_UpdateScreen();
		break;


	} //autos
}

void capacit2(){
	if (adcVoltage[2] == Vmin && chargeFlag2 == false){
		timeCharger2 = 0;
		mA2 = (total_current2 / count_Current2);

		CapacitorF2 = (mA2 * (timeDisCharge2)) / (Vmax - 0.250); // формула для вычесления емкости
		percent2 = CapacitorF2 / 15;
		//timeDisCharge2 = 0;
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); //выключаем прерывание
		HAL_GPIO_WritePin(Rele2_GPIO_Port, Rele2_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(Load2_GPIO_Port, Load2_Pin, GPIO_PIN_SET);
		HAL_TIM_Base_Start_IT(&htim3); // запускаем таймер
		count2 ++;
		chargeFlag2 = true;
	}

	if(adcVoltage[2] > Vmax && chargeFlag2 == true){
		timeDisCharge2 = 0;
		count_Current2 = 0;
		total_current2 = 0;
		//total_current2 = 0;
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); //выключаем прерывание
		HAL_GPIO_WritePin(Rele2_GPIO_Port, Rele2_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(Load2_GPIO_Port, Load2_Pin, GPIO_PIN_RESET);
		HAL_TIM_Base_Start_IT(&htim3); // запускаем таймер
		chargeFlag2 = false;
	}

	if (!(chargeFlag2)){
		timeDisCharge2 ++;
		currMeasurment();
	}

	if (chargeFlag2){
		timeCharger2++;

	}

}

void currMeasurment(){
	current_current2 = (((adcVoltage[2] * 2 - adcVoltage[2]) / 23) * 1000); // Вычесление тока при разрядке
	current_current2 = (int) (current_current2 / 1.1);
	current_current2 = (float) current_current2;

	//B = ((int)(A * 100))/100;
	if (current_current2 < 0) {
		current_current2 = 0;
	} //if
	if (current_current2 != 0) {
		 total_current2 += current_current2 ;
		count_Current2++;
	} //if
}




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

	//	saveVariablesToFlash(var1);
	//count = getVariablesFromFlash();
	getVariablesFromFlash();
	newCount = count;
	//count2 = getVariablesFromFlash2();

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
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


	// Зажать кнопку при включении для сброса счетчика циклов
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcData, ADC_CHANNELS_NUM);
	if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
		count = 0;  //151000
		count2 = 0; //106000



		saveVariablesToFlash();
	}
	//HAL_ADCEx_Calibration_Start(&hadc1); //
	ssd1306_Init();
	HAL_Delay(100);
	ssd1306_Test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		//HAL_TIM_Base_Start_IT(&htim3);

        //Функция измерения напряжения
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcData, ADC_CHANNELS_NUM);


		time = 0;

		if (count - newCount >= 10) { // Сохраняем если 10

			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); //выключаем прерывание
			saveVariablesToFlash();
			HAL_TIM_Base_Start_IT(&htim3); // запускаем таймер
			newCount = count;

		}





		total_current = 0;
		count_Current = 0;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		switch (autos) {
		case 1: // автомат разряда
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); //выключаем прерывание
			HAL_GPIO_WritePin(Rele1_GPIO_Port, Rele1_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(Load_GPIO_Port, Load_Pin, GPIO_PIN_RESET);
			HAL_TIM_Base_Start_IT(&htim3); // запускаем таймер
			//printf("Discharge\n");
			timeDisCharge = 0;
			led = 0;
			chargeFlag = false;

			while (adcVoltage[0] > Vmin) {
				if (HAL_GetTick() - temp >= 1000) {
					time++;
					HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
					temp = HAL_GetTick();
					HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcData,
					ADC_CHANNELS_NUM);
					capacit2();
				}
				timeDisCharge = time;
				//voltage = VoltageRead();
				param(); //тест
			}

			mA = (total_current / count_Current);
			CapacitorF = (mA * (timeDisCharge)) / (Vmax - 0.250); // формула для вычесления емкости
			percent = CapacitorF / 15;
			autos = 2;
			break;
		case 2: //автомат заряда
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); //выключаем прерывание
			HAL_GPIO_WritePin(Rele1_GPIO_Port, Rele1_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(Load_GPIO_Port, Load_Pin, GPIO_PIN_SET);
			HAL_TIM_Base_Start_IT(&htim3); // запускаем таймер
			timeCharger = 0;
			chargeFlag = true;
			count++;
			//printf("Charger\n");
			while (adcVoltage[0] < Vmax ){
				if (HAL_GetTick() - temp >= 1000) {
					time++;
					HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
					temp = HAL_GetTick();
					HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcData,
					ADC_CHANNELS_NUM);
					capacit2();
				}
				timeCharger = time;
				param(); // тест
				//voltage = VoltageRead();
			}// while заряд
			autos = 1;
			break;

			/* USER CODE BEGIN 3 */
		} // switch
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 300;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OLED_CS_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_Res_Pin|Load2_Pin|Rele2_Pin|Rele1_Pin
                          |Load_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_CS_Pin OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_CS_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_Res_Pin Load2_Pin Rele2_Pin */
  GPIO_InitStruct.Pin = OLED_Res_Pin|Load2_Pin|Rele2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Rele1_Pin Load_Pin */
  GPIO_InitStruct.Pin = Rele1_Pin|Load_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : bat1_Pin */
  GPIO_InitStruct.Pin = bat1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(bat1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void testInt(){

	}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_8) {
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); //выключаем прерывание
		//testInt();

		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		num++;
		if (num > 4) num = 0;
		HAL_TIM_Base_Start_IT(&htim3); // запускаем таймер


}
//	else if(GPIO_Pin == GPIO_PIN_0) {
//		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
        if(htim->Instance == TIM3)
        {
                HAL_TIM_Base_Stop_IT(&htim3); // останавливаем таймер
				  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);  // очищаем бит EXTI_PR
				  NVIC_ClearPendingIRQ(EXTI9_5_IRQn); // очищаем бит NVIC_ICPRx
				  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);   // включаем внешнее прерывание


        }

}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	if (hadc->Instance == ADC1) {
		for (uint8_t i = 0; i < ADC_CHANNELS_NUM; i++) {
			adcVoltage[i] = (adcData[i] * 17.39 / 4095) * 100.0;
			adcVoltage[i] = (int) adcVoltage[i] / 100.0;
			//adcVoltage[i] = (float)adcVoltage[i]+ 0.200;
			if (chargeFlag) {
				current_current = (((adcVoltage[1] - adcVoltage[0]) / 10) * 1000); // Вычесление тока при зарядке
				current_current = (int) (current_current / 1.1);
				current_current = (float) current_current;

				//B = ((int)(A * 100))/100;
				if (current_current < 0) {
					current_current = 0;
				}
			} else {

				current_current = (((adcVoltage[0] * 2 - adcVoltage[0]) / 23) * 1000); // Вычесление тока при разрядке
				current_current = (int) (current_current / 1.1);
				current_current = (float) current_current;

				//B = ((int)(A * 100))/100;
				if (current_current < 0) {
					current_current = 0;
				} //if
			} //else
		}
	} //if
	if (current_current != 0) {
		//total_current = total_current + A;
		 total_current += current_current ;
		count_Current++;
	} //if
} //HAL_ADC



void saveVariablesToFlash() {
	HAL_FLASH_Unlock(); // Разблокировка флэш-памяти

	// Стирание страницы флэш-памяти
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.PageAddress = FLASH_USER_START_ADDR;
	erase.NbPages = 1;
	uint32_t pageError = 0;
	HAL_FLASHEx_Erase(&erase, &pageError);

	// Сохранение переменных
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, VAR1_ADDRESS, count);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, VAR2_ADDRESS, count2);

	HAL_FLASH_Lock(); // Блокировка флэш-памяти
	HAL_Delay(100);
}

void getVariablesFromFlash() {
	count = *((volatile uint32_t*) VAR1_ADDRESS); // Чтение переменной 1
	count2 = *((volatile uint32_t*)VAR2_ADDRESS); // Чтение переменной 2


}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
