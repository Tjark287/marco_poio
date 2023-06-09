/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * define IS_POI or IS_WEARABLE depending on which device to program
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include<stdlib.h>
#include<string.h>
#include<stdio.h>
#include<stdbool.h>
#include "RFM69.h"
#include "helpHeidi.h"
#include "compass.h"
#include "gps.h"
#include "minmea.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct gpsdata{
	float lat, lon;
	uint8_t numsat;
};
typedef struct gpsdata gpsdata_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* DEVICE SELECTION ----------------------------------------------------------*/
/*	@brief uncomment POI or WEARABLE
*/

//#define IS_POI
#define IS_WEARABLE

/* Wireless communication setup ----------------------------------------------*/

#ifdef IS_POI
#define NODEID        3
#endif

#ifdef IS_WEARABLE
#define NODEID        2
#endif

/* common wireless settings --------------------------------------------------*/
#define NETWORKID     100  //the same on all nodes that talk to each other
#define FREQUENCY     RF69_868MHZ
#define ENCRYPTKEY    "heidiheidiheidii" // exactly 16 characters/bytes
#define IS_RFM69HW_HCW

// Vibration off state
#define VIB_OFF 0

// UART buffer
#define UARTbufLen 75
char uartData[UARTbufLen];

// GPS data struct's
gpsdata_t myGPS, extGPS;

// init bools
_Bool haveReceived = false;
_Bool haveGPS = false;
_Bool GPSvalid = false;
_Bool txTimeout = false;
_Bool rxTimeout = false;

// device-specific variables

#ifdef IS_POI
// distance variable for buzzer activation
float distance;
#define DISTANCE_TO_BUZZ 15.0f
#endif

#ifdef IS_WEARABLE
// heading search mode state variable for wearable + timer variable
_Bool IS_headingMode = false; // init true for debug
uint16_t headingModeTim = 0;
uint8_t txTimeoutTim = 0;
#define HEADINGMODE_TIME 300 - 1 // time device remains in headingmode x100ms

// heading variable
float my_heading = 0.0;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM11_Init(void);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  /*
   * setup both devices to be receiving, POI later starts sending after GPS data valid
   */

	// init of RFM69
	RFM69_reset();
	RFM69_initialize(RF69_868MHZ, NODEID, NETWORKID);
	// enable high power hardware -> RFM69HW
	RFM69_setHighPower(true);
	// start in RX Mode
	RFM69_setMode(RF69_MODE_RX);
	// spyMode ?
	RFM69_isSpy(true);
	// encryption
	RFM69_encrypt(ENCRYPTKEY);
	// begin receiving and interrupting
	RFM69_receiveBegin();

	// init of GPS
	extern char GPS_recBuffer[GPS_receiveLen];
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)GPS_recBuffer, GPS_receiveLen); // new DMA based approach

	// init of 10Hz heartbeat timer (11)
	HAL_TIM_Base_Start_IT(&htim11);

	// init buzzer pin to off
	HAL_GPIO_WritePin(UI_BUZZ_GPIO_Port, UI_BUZZ_Pin, RESET);

	// zero out GPS variables
	myGPS.lat = 0.0f;
	myGPS.lon = 0.0f;
	extGPS.lat = 0.0f;
	extGPS.lon = 0.0f;

	// wearable specific setup
	#ifdef IS_WEARABLE
	// init tim1 ch3 to zero output
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	TIM3->CCR3 = VIB_OFF;

	// init compass
	if(!CM_init()){
		sprintf(uartData, "compass init fail\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)uartData, sizeof(uartData), HAL_MAX_DELAY);
	}
	#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		#ifdef IS_WEARABLE
		#endif

		#ifdef IS_POI
		#endif


    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 96 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  htim2.Init.Prescaler = 96 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 250000 - 1;
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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 9600 - 1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000 -1 ;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RFM69_RST_Pin|RFM69_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD_1_Pin|LD_2_Pin|GPS_RST_Pin|GPS_FON_Pin
                          |GPS_STDBY_Pin|UI_BUZZ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RFM69_RST_Pin RFM69_NSS_Pin */
  GPIO_InitStruct.Pin = RFM69_RST_Pin|RFM69_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD_1_Pin LD_2_Pin GPS_RST_Pin GPS_FON_Pin
                           GPS_STDBY_Pin UI_BUZZ_Pin */
  GPIO_InitStruct.Pin = LD_1_Pin|LD_2_Pin|GPS_RST_Pin|GPS_FON_Pin
                          |GPS_STDBY_Pin|UI_BUZZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*
	 * RFM69 interrupt handler;
	 * on RFM69 interrupt pin callback, process received data.
	 */

    if(GPIO_Pin == GPIO_PIN_0 && RFM69_isMode(RF69_MODE_RX))
    {
    	// set "_havedata" to true
    	RFM69_havedata(true);

    	// call data management function
    	RFM69_receiveDone();

		// read RSSI
//		int16_t rssi = RFM69_readRSSI(false);

    	// get pointer to received data
		uint8_t* dataP = RFM69_returnData();

		// save received coords in external GPS variable
		extGPS.lat = combine_coords(dataP[0], dataP[1], dataP[2], dataP[3]);
		extGPS.lon = combine_coords(dataP[4], dataP[5], dataP[6], dataP[7]);

		// gets set on first received GPS data from other device
		if(extGPS.lat != 0.0f && extGPS.lon != 0.0f) haveReceived = true;

    }

    // user button interrupt
    if(GPIO_Pin == GPIO_PIN_1){
		#ifdef IS_WEARABLE
		IS_headingMode = true;
		#endif
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	extern char GPS_recBuffer[GPS_receiveLen];

	rxTimeout = true;

	float lat, lon;
	uint8_t numsat;
	char rmcline[MINMEA_MAX_SENTENCE_LENGTH];

	get_gnrmc_line(GPS_recBuffer, rmcline, MINMEA_MAX_SENTENCE_LENGTH);
	GPSvalid = GPS_process(rmcline, &lat, &lon, &numsat);

	if(GPSvalid){
		// sets haveGps true on first valid receive
		haveGPS = true;

		// save data only if valid
		myGPS.lat = lat;
		myGPS.lon = lon;
		myGPS.numsat = numsat;
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		// 250ms TIMER - UNUSED
	}

	// heartbeat timer (11) - 10Hz
	if (htim == &htim11){

		// indicator LED
		if(haveGPS && haveReceived){
			HAL_GPIO_WritePin(LD_1_GPIO_Port, LD_1_Pin, SET);
		} else {
			HAL_GPIO_WritePin(LD_1_GPIO_Port, LD_1_Pin, RESET);
		}

		// heartbeat LED
		HAL_GPIO_TogglePin(LD_2_GPIO_Port, LD_2_Pin);

		// UART rxTimeout reset
		extern char GPS_recBuffer[GPS_receiveLen];
		static uint8_t uartCounter;
		if (rxTimeout) uartCounter++;
		if(uartCounter > 2){
			rxTimeout = false;
			uartCounter = 0;
			HAL_UART_Receive_DMA(&huart1, (uint8_t*)GPS_recBuffer, GPS_receiveLen);
		}

		#ifdef IS_POI
		// set buzzer if wearable is near
		if(haveGPS && haveReceived){
			if(get_distance(myGPS.lat, myGPS.lon, extGPS.lat, extGPS.lon) < DISTANCE_TO_BUZZ){
				HAL_GPIO_WritePin(UI_BUZZ_GPIO_Port, UI_BUZZ_Pin, SET);
			} else {
				HAL_GPIO_WritePin(UI_BUZZ_GPIO_Port, UI_BUZZ_Pin, RESET);
			}

		}
		static uint8_t gpsTxCounter;
		static _Bool timeToSend;
		gpsTxCounter++;
		if(gpsTxCounter > 9){
			gpsTxCounter = 0;
			timeToSend = true;
		}
		if(GPSvalid && timeToSend){
			uint8_t sendLen = 8;
			uint8_t txData[sendLen];
			uint8_t d_lat, m_lat, s_lat, h_lat, d_lon, m_lon, s_lon, h_lon;
			uint16_t toAddress = 2;

			split_coords(myGPS.lat, &d_lat, &m_lat, &s_lat, &h_lat);
			split_coords(myGPS.lon, &d_lon, &m_lon, &s_lon, &h_lon);

			txData[0] = d_lat;
			txData[1] = m_lat;
			txData[2] = s_lat;
			txData[3] = h_lat;
			txData[4] = d_lon;
			txData[5] = m_lon;
			txData[6] = s_lon;
			txData[7] = h_lon;

			// send GPS data to other device
			RFM69_setMode(RF69_MODE_RX);

			if (RFM69_canSend()){
				RFM69_send(toAddress, txData, sendLen, false);
				sprintf(uartData, "\nTX Complete\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)uartData, sizeof(uartData), HAL_MAX_DELAY);
				RFM69_setMode(RF69_MODE_TX);
			}

			// put RFM69 back in receive mode, listen for more
			RFM69_receiveBegin();

			// reset timeToSend bool
			timeToSend = false;
		}
		#endif

		#ifdef IS_WEARABLE
	    // reset tx timeout depending on mode
	    if(txTimeout){
	    	if (txTimeoutTim > 9 && !IS_headingMode){
	    		txTimeout = false;
	    		txTimeoutTim = 0;
	    	}
	    	if (txTimeoutTim > 4 && IS_headingMode){
	    		txTimeout = false;
	    		txTimeoutTim = 0;
	    	}
	    }

		// wearable sends data, faster when in heading mode
		uint8_t sendLen = 8;
		if(GPSvalid && !txTimeout){
			uint8_t txData[sendLen];
			uint8_t d_lat, m_lat, s_lat, h_lat, d_lon, m_lon, s_lon, h_lon;
			uint16_t toAddress = 3;

			split_coords(myGPS.lat, &d_lat, &m_lat, &s_lat, &h_lat);
			split_coords(myGPS.lon, &d_lon, &m_lon, &s_lon, &h_lon);

			txData[0] = d_lat;
			txData[1] = m_lat;
			txData[2] = s_lat;
			txData[3] = h_lat;
			txData[4] = d_lon;
			txData[5] = m_lon;
			txData[6] = s_lon;
			txData[7] = h_lon;

			// send GPS data to other device
			RFM69_receiveBegin();

			if (RFM69_canSend()){
				RFM69_send(toAddress, txData, sendLen, false);
				txTimeout = true;
				RFM69_receiveBegin(); // start listening again
			}

		}

		if(IS_headingMode){

			// get my heading
			float heading = CM_getheading();

			// set vibration
			if(haveGPS && GPSvalid && haveReceived){
				float gpsHead = gps_to_heading(extGPS.lat, extGPS.lon, myGPS.lat, myGPS.lon);
				uint16_t vibstr = setVibr(heading, gpsHead, 45.0);
				TIM1->CCR3 = vibstr;
			}
			if(headingModeTim++ > HEADINGMODE_TIME){
				IS_headingMode = false;
				headingModeTim = 0;
			}
		} else {
			TIM1->CCR3 = VIB_OFF;
		}
		#endif
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

  clearBuffer((uint8_t*)uartData, UARTbufLen);
  sprintf(uartData, "SYSTEM ERROR, dude\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uartData, sizeof(uartData), HAL_MAX_DELAY);

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
