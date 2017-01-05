/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
//#include "lcd.h"
#include "yahdlc/yahdlc.h"
#include "circular_buffer.h"
#include "nf/nfv2.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
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
}RxPackage;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);

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

int pc_rx_handler(char newRx){
	int got_message = 0;
	unsigned int message_length = 0;
	TxPackage package;
	yahdlc_control_t control_recv;
	pc_rx_buffer[pc_rx_iter] = newRx;
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
		drives_left_velocity = package.left_drive_speed;
		drives_right_velocity = package.right_drive_speed;
	}
	return got_message;
}

void send_pc_response(){
	RxPackage package;
	package.tx_timestamp = last_timestamp;
	package.rx_timestamp = counter;
	++counter;
	package.left_drive_position = drives_l_pos_total;
	package.right_drive_position = drives_r_pos_total;
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

  /* USER CODE BEGIN 2 */
    NFv2_CrcInit();
    NFv2_Config(&drives_NFCommunicationBuffer, 0x01);
    //drives connection init
    buffer_init(&uart2RxBuffer);
    HAL_GPIO_WritePin(USART2_TXEN_GPIO_Port, USART2_TXEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIO6_GPIO_Port, DIO6_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIO7_GPIO_Port, DIO7_Pin, GPIO_PIN_SET);
    HAL_UART_Receive_IT(&huart2, &uart2Received, sizeof(uart2Received));
    drives_set_speed_request(100, 100);

    //pc connection init
    buffer_init(&uart4RxBuffer);
    HAL_UART_Receive_DMA(&huart4, &uart4Received, sizeof(uart4Received));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    int i = 0;
    int j = 0;
    drives_left_velocity = 50;
    drives_right_velocity = 50;
  while (1)
  {
	  NVIC_DisableIRQ(DMA2_Channel5_IRQn);
		char uart4Rx;
		char newUart4Rx = buffer_get(&uart4RxBuffer, &uart4Rx);
		NVIC_EnableIRQ(DMA2_Channel5_IRQn);
		if (newUart4Rx){
			pc_rx_handler(uart4Rx);
		}

	  HAL_NVIC_DisableIRQ(USART2_IRQn);
		char uart2Rx;
		char newUart2Rx = buffer_get(&uart2RxBuffer, &uart2Rx);
		HAL_NVIC_EnableIRQ(USART2_IRQn);
		if (newUart2Rx){
			if(drives_rx_handler(uart2Rx)){
				send_pc_response();
			}
		}
		if(i == 10000){
			drives_set_speed_request(50, 50);
			HAL_GPIO_TogglePin(ST_LED_GPIO_Port, ST_LED_Pin);
			i = 0;
			j++;
			if(j == 100){
				j = 0;
			}
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : KEY_1_Pin KEY_2_Pin KEY_3_Pin KEY_4_Pin 
                           KEY_5_Pin */
  GPIO_InitStruct.Pin = KEY_1_Pin|KEY_2_Pin|KEY_3_Pin|KEY_4_Pin 
                          |KEY_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : ST_LED_Pin */
  GPIO_InitStruct.Pin = ST_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ST_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO1_Pin DIO2_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin|DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO3_Pin DIO4_Pin DIO5_Pin DIO6_Pin 
                           DIO7_Pin */
  GPIO_InitStruct.Pin = DIO3_Pin|DIO4_Pin|DIO5_Pin|DIO6_Pin 
                          |DIO7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USART2_TXEN_Pin */
  GPIO_InitStruct.Pin = USART2_TXEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USART2_TXEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ST_LED_GPIO_Port, ST_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIO1_Pin|DIO2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DIO3_Pin|DIO4_Pin|DIO5_Pin|DIO6_Pin 
                          |DIO7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USART2_TXEN_GPIO_Port, USART2_TXEN_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
