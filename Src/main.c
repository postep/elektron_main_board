/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#define HELP_B(x, y) if(!x && buttons[y] < 20){buttons[y]++;}; if(x && buttons[y] > 0){buttons[y]--;}
#include "yahdlc/yahdlc.h"
#include "circular_buffer.h"
#include "nf/nfv2.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

osThreadId defaultTaskHandle;
osThreadId outTaskHandle;
osThreadId motorsTaskHandle;
osThreadId pcTaskHandle;
osThreadId buttonsTaskHandle;
osThreadId shutdownTaskHandle;
osThreadId batteryTaskHandle;
osThreadId setMotorsTaskHandle;
osMessageQId shutdownQueueHandle;
osMessageQId setMotorsQueueHandle;
osMutexId batteryMutexHandle;
osMutexId drivesSpeedMutexHandle;
osMutexId drivesPositionMutexHandle;
osMutexId relaysMutexHandle;
osMutexId buttonsMutexHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t battery_voltage_adc;
uint8_t relays;
uint8_t buttons[5];
uint8_t buttons_register;
Buffer uart4RxBuffer;
char uart4TxBuffer[128];
Buffer uart2RxBuffer;
Buffer uart2TxBuffer;
char uart4Received[64];
char uart2Received;
char uart4Tx;
int drives_left_velocity = 0;
int drives_right_velocity = 0;

//Obiekty i struktury biblioteki NFv2
NF_STRUCT_ComBuf drives_NFCommunicationBuffer;	//Bufor komunikacyjny biblioteki NFv2.
uint8_t drives_txBuffer[256];	//Dane wysylane do plytki.
uint8_t drives_txCount;		//Dlugosc danych wysylanych do plytki.
uint8_t drives_commandArray[256];	//Tablica polecen do sterownika.
uint8_t drives_commandCount;		//Stopien zapelnienia tablicy polecen do sterownika.
uint8_t drives_rxCommandArray[256];	//Tablica danych przychodz¹cych od p³ytki.
uint8_t drives_rxCommandCount;	//Stopien zapelnienia tablicy danych od p³ytki.
uint8_t drives_rxBuffer[256];	//Dane odebrane do plytki.
uint8_t drives_rxCount = 0;		//Dlugosc danych odebranych od plytki.
double drives_l_pos_total=0;	//Aktualna pozycja kola prawego w SI [m].
double drives_r_pos_total=0;	//Aktualna pozycja kola lewego w SI [m].
double drives_l_speed_pic = 0;
double drives_r_speed_pic = 0;

//Obiekty i struktury do komunikacji z pc
char pc_rx_buffer[256];
short pc_rx_iter = 0;
unsigned last_timestamp = 0;
unsigned counter = 0;

typedef struct{
	unsigned id;
	unsigned tx_timestamp;
	unsigned rx_timestamp;
	short left_drive_speed;
	short right_drive_speed;
	char relays;
	char shutdown;
	char sound;
	//char lcd_segment;
	//char lcd_message[13];
}TxPackage;

typedef struct{
	unsigned id;
	unsigned tx_timestamp;
	unsigned rx_timestamp;
	double left_drive_position;
	double right_drive_position;
	char buttons;
	char shutdown;
	short battery;
}RxPackage;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);
void StartOutTask(void const * argument);
void StartMotorsTask(void const * argument);
void StartPcTask(void const * argument);
void StartButtonsTask(void const * argument);
void StartShutdownTask(void const * argument);
void StartBatteryTask(void const * argument);
void StartSetMotorsTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int drives_rx_handler(char newRx);
int pc_rx_handler(char newRx);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart) {
	if(huart->Instance == huart2.Instance){
		HAL_GPIO_WritePin(USART2_TXEN_GPIO_Port, USART2_TXEN_Pin, GPIO_PIN_RESET);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == huart2.Instance){
		buffer_put(uart2Received, &uart2RxBuffer);
		HAL_UART_Receive_IT(&huart2, &uart2Received, 1);
	}
	if(huart->Instance == huart4.Instance){
		buffer_putn(uart4Received, sizeof(uart4Received), &uart4RxBuffer);
  		HAL_UART_Receive_DMA(&huart4, uart4Received, sizeof(uart4Received));
	}
}

void drives_set_speed_request(int left, int right){
	drives_commandCount = 0;
	drives_NFCommunicationBuffer.SetDrivesMode.data[0] = NF_DrivesMode_SPEED;
	drives_NFCommunicationBuffer.SetDrivesMode.data[1] = NF_DrivesMode_SPEED;
	drives_NFCommunicationBuffer.SetDrivesSpeed.data[0] = left;
	drives_NFCommunicationBuffer.SetDrivesSpeed.data[1] = right;
	drives_commandArray[drives_commandCount++] = NF_COMMAND_SetDrivesMode;
	drives_commandArray[drives_commandCount++] = NF_COMMAND_SetDrivesSpeed;
	drives_commandArray[drives_commandCount++] = NF_COMMAND_ReadDeviceVitals;
	drives_commandArray[drives_commandCount++] = NF_COMMAND_ReadDrivesPosition;
	drives_txCount = NF_MakeCommandFrame(&drives_NFCommunicationBuffer, drives_txBuffer,
			(const uint8_t*)drives_commandArray, drives_commandCount, 0x10);
	HAL_GPIO_WritePin(USART2_TXEN_GPIO_Port, USART2_TXEN_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit_IT(&huart2, drives_txBuffer, drives_txCount);
}

int drives_rx_handler(char newRx){
	int ret = 0;
	if(drives_rxCount == 255){
		drives_rxCount = 0;
	}
	drives_rxBuffer[drives_rxCount] = newRx;
	if(NF_Interpreter(&drives_NFCommunicationBuffer, drives_rxBuffer, &drives_rxCount, drives_rxCommandArray, &drives_rxCommandCount) > 0){
		if(drives_NFCommunicationBuffer.ReadDrivesPosition.updated){
			drives_l_pos_total = drives_NFCommunicationBuffer.ReadDrivesPosition.data[0];
			drives_r_pos_total = drives_NFCommunicationBuffer.ReadDrivesPosition.data[1];
		 	drives_NFCommunicationBuffer.ReadDrivesPosition.updated=0;
		 	ret = 1;
		}
	}
	return ret;
}

void send_pc_response(){
	RxPackage package;
	package.tx_timestamp = last_timestamp;
	package.rx_timestamp = counter;
	++counter;
	package.left_drive_position = drives_l_pos_total;
	package.right_drive_position = drives_r_pos_total;
	package.battery = battery_voltage_adc;
	yahdlc_control_t control;
	// Initialize the control field structure with frame type and sequence number
	control.frame = YAHDLC_FRAME_DATA;
	// Create an empty frame with the control field information
	unsigned frame_length;
	yahdlc_frame_data(&control, (char *)(&package), sizeof(RxPackage), uart4TxBuffer, &frame_length);
	HAL_UART_Transmit_DMA(&huart4, uart4TxBuffer, frame_length);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  	  //init nf2, communication with drives
    NFv2_CrcInit();
    NFv2_Config(&drives_NFCommunicationBuffer, 0x01);

    //init battery measure
    HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK){
		battery_voltage_adc = 5*HAL_ADC_GetValue(&hadc1)/3.3;
	}

	//init relays and buttons
	relays = 0;
	buttons_register = 0;
	for(int i = 0; i < 5; i++){
		buttons[i] = 0;
	}
    //drives connection init
    buffer_init(&uart2RxBuffer);
    HAL_GPIO_WritePin(USART2_TXEN_GPIO_Port, USART2_TXEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIO6_GPIO_Port, DIO6_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIO7_GPIO_Port, DIO7_Pin, GPIO_PIN_SET);
    HAL_UART_Receive_IT(&huart2, &uart2Received, sizeof(uart2Received));
    //drives_set_speed_request(0, 0);

    //pc connection init
    buffer_init(&uart4RxBuffer);
    HAL_UART_Receive_DMA(&huart4, &uart4Received, sizeof(uart4Received));

    //battery voltage measure init
    //HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of batteryMutex */
  osMutexDef(batteryMutex);
  batteryMutexHandle = osMutexCreate(osMutex(batteryMutex));

  /* definition and creation of drivesSpeedMutex */
  osMutexDef(drivesSpeedMutex);
  drivesSpeedMutexHandle = osMutexCreate(osMutex(drivesSpeedMutex));

  /* definition and creation of drivesPositionMutex */
  osMutexDef(drivesPositionMutex);
  drivesPositionMutexHandle = osMutexCreate(osMutex(drivesPositionMutex));

  /* definition and creation of relaysMutex */
  osMutexDef(relaysMutex);
  relaysMutexHandle = osMutexCreate(osMutex(relaysMutex));

  /* definition and creation of buttonsMutex */
  osMutexDef(buttonsMutex);
  buttonsMutexHandle = osMutexCreate(osMutex(buttonsMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of outTask */
  osThreadDef(outTask, StartOutTask, osPriorityLow, 0, 128);
  outTaskHandle = osThreadCreate(osThread(outTask), NULL);

  /* definition and creation of motorsTask */
  osThreadDef(motorsTask, StartMotorsTask, osPriorityRealtime, 0, 512);
  motorsTaskHandle = osThreadCreate(osThread(motorsTask), NULL);

  /* definition and creation of pcTask */
  osThreadDef(pcTask, StartPcTask, osPriorityHigh, 0, 512);
  pcTaskHandle = osThreadCreate(osThread(pcTask), NULL);

  /* definition and creation of buttonsTask */
  osThreadDef(buttonsTask, StartButtonsTask, osPriorityLow, 0, 128);
  buttonsTaskHandle = osThreadCreate(osThread(buttonsTask), NULL);

  /* definition and creation of shutdownTask */
  osThreadDef(shutdownTask, StartShutdownTask, osPriorityIdle, 0, 128);
  shutdownTaskHandle = osThreadCreate(osThread(shutdownTask), NULL);

  /* definition and creation of batteryTask */
  osThreadDef(batteryTask, StartBatteryTask, osPriorityIdle, 0, 128);
  batteryTaskHandle = osThreadCreate(osThread(batteryTask), NULL);

  /* definition and creation of setMotorsTask */
  osThreadDef(setMotorsTask, StartSetMotorsTask, osPriorityHigh, 0, 128);
  setMotorsTaskHandle = osThreadCreate(osThread(setMotorsTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of shutdownQueue */
  osMessageQDef(shutdownQueue, 2, uint8_t);
  shutdownQueueHandle = osMessageCreate(osMessageQ(shutdownQueue), NULL);

  /* definition and creation of setMotorsQueue */
  osMessageQDef(setMotorsQueue, 16, uint32_t);
  setMotorsQueueHandle = osMessageCreate(osMessageQ(setMotorsQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    int i = 0;
    int j = 0;
    drives_left_velocity = 0;
    drives_right_velocity = 0;
  while (1)
  {
		if(i == 10000){
			int temp_l = drives_left_velocity;
			int temp_r = drives_right_velocity;
			drives_set_speed_request(temp_l, temp_r);
			//HAL_GPIO_TogglePin(ST_LED_GPIO_Port, ST_LED_Pin);
			i = 0;
		}
		++i;
		//HAL_Delay(2);
	  //drives_set_speed_request(100, 100);
	  //HAL_Delay(1);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Configure the Systick interrupt time 
    */
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
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

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : KEY_1_Pin KEY_2_Pin KEY_3_Pin KEY_4_Pin 
                           KEY_5_Pin */
  GPIO_InitStruct.Pin = KEY_1_Pin|KEY_2_Pin|KEY_3_Pin|KEY_4_Pin 
                          |KEY_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ST_LED_Pin LCD_RST_Pin LCD_RW_Pin LCD_LED_Pin */
  GPIO_InitStruct.Pin = ST_LED_Pin|LCD_RST_Pin|LCD_RW_Pin|LCD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : REL1_Pin REL3_Pin LCD_DB0_Pin */
  GPIO_InitStruct.Pin = REL1_Pin|REL3_Pin|LCD_DB0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : REL2_Pin ROBOT_OFF_Pin REL4_Pin DIO6_Pin 
                           DIO7_Pin LCD_DB4_Pin LCD_DB3_Pin LCD_DB2_Pin 
                           LCD_DB1_Pin */
  GPIO_InitStruct.Pin = REL2_Pin|ROBOT_OFF_Pin|REL4_Pin|DIO6_Pin 
                          |DIO7_Pin|LCD_DB4_Pin|LCD_DB3_Pin|LCD_DB2_Pin 
                          |LCD_DB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DB5_Pin LCD_DB6_Pin LCD_DB7_Pin LCD_CS1_Pin 
                           LCD_CS2_Pin USART2_TXEN_Pin */
  GPIO_InitStruct.Pin = LCD_DB5_Pin|LCD_DB6_Pin|LCD_DB7_Pin|LCD_CS1_Pin 
                          |LCD_CS2_Pin|USART2_TXEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DI_Pin LCD_E_Pin */
  GPIO_InitStruct.Pin = LCD_DI_Pin|LCD_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ST_LED_Pin|LCD_RST_Pin|LCD_RW_Pin|LCD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, REL1_Pin|REL3_Pin|LCD_DB0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, REL2_Pin|ROBOT_OFF_Pin|REL4_Pin|DIO6_Pin 
                          |DIO7_Pin|LCD_DB4_Pin|LCD_DB3_Pin|LCD_DB2_Pin 
                          |LCD_DB1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCD_DB5_Pin|LCD_DB6_Pin|LCD_DB7_Pin|LCD_CS1_Pin 
                          |LCD_CS2_Pin|USART2_TXEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_DI_Pin|LCD_E_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    HAL_GPIO_TogglePin(ST_LED_GPIO_Port, ST_LED_Pin);
  }
  /* USER CODE END 5 */ 
}

/* StartOutTask function */
void StartOutTask(void const * argument)
{
  /* USER CODE BEGIN StartOutTask */
  /* Infinite loop */
	LCD_init();
	LCD_clear();
	LCD_writeLine(4, "              1 2 3 4");
  for(;;)
  {
	  uint8_t temp_relays = 0x55;
	  xSemaphoreTake(relaysMutexHandle, portTICK_PERIOD_MS*4);
	  //temp_relays = relays;
	  xSemaphoreGive(relaysMutexHandle);
	  char rel1 = (temp_relays & 0x01);
	  char rel2 = (temp_relays & 0x02);
	  char rel3 = (temp_relays & 0x04);
	  char rel4 = (temp_relays & 0x08);
	  HAL_GPIO_WritePin(REL1_GPIO_Port, REL1_Pin, rel1);
	  HAL_GPIO_WritePin(REL2_GPIO_Port, REL2_Pin, rel2);
	  HAL_GPIO_WritePin(REL3_GPIO_Port, REL3_Pin, rel3);
	  HAL_GPIO_WritePin(REL4_GPIO_Port, REL4_Pin, rel4);
	int battery_copy = 0;
	xSemaphoreTake(batteryMutexHandle, portMAX_DELAY);
	battery_copy = battery_voltage_adc;
	xSemaphoreGive(batteryMutexHandle);
	if(battery_copy == 0){
		LCD_writeLine(0, "POMIAR BAT. NIEDOSTEPNY");
	}else{
		int battery_int = battery_copy/100;
		int battery_decimal = battery_copy%100;
		char string[20];
		sprintf(string, "Bateria:      %d.%dV     ", battery_int, battery_decimal);
		LCD_writeLine(0, string);
		if(battery_copy < 2200){
			LCD_writeLine(1, "BATERIA ROZLADOWANA");
		}else{
			LCD_writeLine(1, "                   ");
		}
	}
	char relayString[21];
	memcpy(relayString, "Przekazniki:  ", 14);
	relayString[13] = ' ';
	relayString[15] = ' ';
	relayString[17] = ' ';
	relayString[19] = ' ';
	if (rel1) {relayString[14] = '*';} else {relayString[14] = '.';}
	if (rel2) {relayString[16] = '*';} else {relayString[16] = '.';}
	if (rel3) {relayString[18] = '*';} else {relayString[18] = '.';}
	if (rel4) {relayString[20] = '*';} else {relayString[20] = '.';}
	relayString[21] = '\0';
	LCD_writeLine(5, relayString);
    osDelay(2000);
  }
  /* USER CODE END StartOutTask */
}

/* StartMotorsTask function */
void StartMotorsTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorsTask */
  /* Infinite loop */
  for(;;)
  {
	HAL_NVIC_DisableIRQ(USART2_IRQn);
	char uart2Rx;
	char newUart2Rx = buffer_get(&uart2RxBuffer, &uart2Rx);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	if (newUart2Rx){
		if(drives_rx_handler(uart2Rx)){
			//send_pc_response();
		}
	}else{
		osDelay(3);
	}
  }
  /* USER CODE END StartMotorsTask */
}

/* StartPcTask function */
void StartPcTask(void const * argument)
{
  /* USER CODE BEGIN StartPcTask */
  /* Infinite loop */
  for(;;)
  {
	NVIC_DisableIRQ(DMA2_Channel5_IRQn);
	char uart4Rx;
	char newUart4Rx = buffer_get(&uart4RxBuffer, &uart4Rx);
	NVIC_EnableIRQ(DMA2_Channel5_IRQn);
	if (newUart4Rx){
		int got_message = 0;
		unsigned int message_length = 0;
		TxPackage package;
		yahdlc_control_t control_recv;
		pc_rx_buffer[pc_rx_iter] = uart4Rx;
		pc_rx_iter++;
		pc_rx_iter &= 0xff;
		if(pc_rx_iter >= sizeof(TxPackage)){
			yahdlc_get_data(&control_recv, pc_rx_buffer, pc_rx_iter, (char*)(&package), &message_length);
			got_message = (message_length > 2);
		}
		if(got_message){
			pc_rx_iter = 0;
		}
		got_message = (got_message && package.tx_timestamp != last_timestamp);
		if(got_message){
			last_timestamp = package.tx_timestamp;
			uint16_t speed[2];
			speed[0] = package.left_drive_speed;
			speed[1] = package.right_drive_speed;
			xQueueSend(setMotorsQueueHandle, &speed, portTICK_PERIOD_MS);
		}
	}else{
		osDelay(2);
	}
  }
  /* USER CODE END StartPcTask */
}

/* StartButtonsTask function */
void StartButtonsTask(void const * argument)
{
  /* USER CODE BEGIN StartButtonsTask */
  /* Infinite loop */
  for(;;)
  {
	  uint8_t k1 = HAL_GPIO_ReadPin(KEY_1_GPIO_Port, KEY_1_Pin);
	  uint8_t k2 = HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin);
	  uint8_t k3 = HAL_GPIO_ReadPin(KEY_3_GPIO_Port, KEY_3_Pin);
	  uint8_t k4 = HAL_GPIO_ReadPin(KEY_4_GPIO_Port, KEY_4_Pin);
	  uint8_t k5 = HAL_GPIO_ReadPin(KEY_5_GPIO_Port, KEY_5_Pin);
	  HELP_B(k1, 0);
	  HELP_B(k2, 1);
	  HELP_B(k3, 2);
	  HELP_B(k4, 3);
	  HELP_B(k5, 4);
	  xSemaphoreTake(buttonsMutexHandle, portTICK_PERIOD_MS*4);
	  buttons_register = 0;
	  for(int i = 4; i >= 0; --i){
		  buttons_register <<= 1;
		  buttons_register |= (buttons[i] > 10);
	  }
	  xSemaphoreGive(buttonsMutexHandle);
	  osDelay(20);
  }
  /* USER CODE END StartButtonsTask */
}

/* StartShutdownTask function */
void StartShutdownTask(void const * argument)
{
  /* USER CODE BEGIN StartShutdownTask */
  /* Infinite loop */
  for(;;)
  {
	uint8_t shutdownTime;
	xQueueReceive(shutdownQueueHandle, &shutdownTime, portMAX_DELAY);
    osDelay(1000*shutdownTime);
    HAL_GPIO_TogglePin(ST_LED_GPIO_Port, ST_LED_Pin);
  }
  /* USER CODE END StartShutdownTask */
}

/* StartBatteryTask function */
void StartBatteryTask(void const * argument)
{
  /* USER CODE BEGIN StartBatteryTask */
  /* Infinite loop */
  for(;;)
  {
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK){
		xSemaphoreTake(batteryMutexHandle, portTICK_RATE_MS*2);
		battery_voltage_adc = 5*HAL_ADC_GetValue(&hadc1)/3.3;
		xSemaphoreGive(batteryMutexHandle);
	}else{
		osDelay(10);
	}
	osDelay(10000);
  }
  /* USER CODE END StartBatteryTask */
}

/* StartSetMotorsTask function */
void StartSetMotorsTask(void const * argument)
{
  /* USER CODE BEGIN StartSetMotorsTask */
  /* Infinite loop */
  for(;;)
  {
	uint16_t speed[2];
	xQueueReceive(setMotorsQueueHandle, speed, portMAX_DELAY);
	drives_set_speed_request(speed[0], speed[1]);
    osDelay(2);
  }
  /* USER CODE END StartSetMotorsTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
