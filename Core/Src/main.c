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
#include <stdio.h>

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
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static CAN_TxHeaderTypeDef Tx_transmit;
static CAN_RxHeaderTypeDef Rx_receive;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t Mailbox;
uint32_t terima = 0;
uint8_t kirim = 0;
uint8_t current_power;
uint8_t new_power;
uint8_t current_rccConn;
uint8_t new_rccConn;
uint8_t sendto_tugConn;
uint8_t new_tugConn;
uint8_t sendto_tugStat;
uint8_t new_tugstat;
uint8_t sendto_signalVal;
uint8_t new_signalVal;
uint8_t sendto_hydroPres; // Data to write inside TxData[1]
uint8_t new_hydroPres;
uint8_t sendto_hydroLev; // Data to write inside TxData[2]
uint8_t new_hydroLev;
uint8_t sendto_batt1; // Data to write inside TxData[3]
uint8_t new_batt1;
uint8_t sendto_batt2; // Data to write inside TxData[4]
uint8_t new_batt2;
uint8_t byte1_value;
uint8_t byte2_value;
uint8_t byte3_value;
uint8_t byte4_value;
uint8_t byte5_value;
uint8_t byte6_value;
uint8_t byte7_value;
uint8_t byte8_value;
uint8_t byte1=0;
uint8_t byte2=0;
uint8_t byte3=0;
uint8_t byte4=0;
uint8_t byte5=0;
uint8_t mockdata[8] = {1, 2, 3, 4, 5, 6, 7, 0};
uint8_t counter;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void CAN_GeneralSetup(CAN_HandleTypeDef *hcan){
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

	sFilterConfig.FilterIdHigh = 0X0000;
	sFilterConfig.FilterIdLow = 0X0000;

	sFilterConfig.FilterMaskIdHigh = 0X0000;
	sFilterConfig.FilterMaskIdLow = 0X0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 0;

	HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
	HAL_CAN_Start(hcan);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);


//	Tx_Header_Msg.DLC = 8;
//	Tx_Header_Msg.IDE = CAN_ID_STD;
//	Tx_Header_Msg.RTR = CAN_RTR_DATA;
}

void NextDataSet(){
	nexTemp = RxData[0];
	power = nexTemp & 1;

	nexTemp = RxData[0];
	rcConn = nexTemp & 2;
	rcConn = rcConn >> 1;

	nexTemp = RxData[0];
	tugConn = nexTemp & 4;
	tugConn = tugConn >> 2;

	nexTemp = RxData[0];
	tugStat = nexTemp & 24;
	tugStat = tugStat >> 3;

	nexTemp = RxData[0];
	signalVal = nexTemp & 224;
	signalVal = signalVal >> 5;

	hydroPres=RxData[1];
	hydroLev=RxData[2];
	batt1=RxData[3];
	batt2=RxData[4];
}

void NEXTION_SendString (char *ID, char *string)
{
	char buf[50];
	int len = sprintf (buf, "%s.txt=\"%s\"", ID, string);
    HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 1000);
    HAL_UART_Transmit(&huart3, Cmd_End, 3, 100);
}

void NEXTION_SendInt (char *ID, uint8_t data)
{
	char buf[50];
	int len = sprintf (buf, "%s.val=%d", ID, data);
    HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 1000);
    HAL_UART_Transmit(&huart3, Cmd_End, 3, 100);
}

void nexSendAllData(){
	NextDataSet();
	NEXTION_SendInt("va0",signalVal);
	NEXTION_SendInt("va1",tugStat);
	NEXTION_SendInt("va2",power);
	NEXTION_SendInt("va3",rcConn);
	NEXTION_SendInt("va4",tugConn);
	NEXTION_SendInt("n0",hydroPres);
	NEXTION_SendInt("j0",hydroLev);
	NEXTION_SendInt("j1",batt1);
	NEXTION_SendInt("j2",batt2);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx_receive, RxData);
	nexSendAllData();
	terima++;

}
void Can_Send(uint8_t id, uint8_t data[8])
{
	Tx_transmit.DLC = 8;
	Tx_transmit.IDE = CAN_ID_STD;
	Tx_transmit.StdId = id;
	Tx_transmit.RTR = CAN_RTR_DATA;

	for(int i = 0; i<8; i++)
	{
		TxData[i] = data[i];
	}

	HAL_CAN_AddTxMessage(&hcan1, &Tx_transmit, TxData, &Mailbox);
	kirim++;

}

void Write_Bit()
{
	byte1_value = new_power;
	byte1_value = byte1_value << 1;
	byte1_value = byte1_value | new_rccConn;
	byte1_value = byte1_value << 1;
	byte1_value = byte1_value | new_tugConn;
	byte1_value = byte1_value << 2;
	byte1_value = byte1_value | new_tugstat;
	byte1_value = byte1_value << 3;
	byte1_value = byte1_value | new_signalVal;
 	byte1 = byte1_value;
	mockdata[7]=byte1;

	byte2_value = new_hydroPres;
	byte2 = byte2_value;
	mockdata[6]=byte2;

	byte3_value = new_hydroLev;
	byte3 = byte3_value;
	mockdata[5]=byte3;

	byte4_value = new_batt1;
	byte4 = byte4_value;
	mockdata[4]=byte3;

	byte5_value = new_batt2;
	byte5 = byte5_value;
	mockdata[3]=byte5;

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
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

//  if(HAL_CAN_Start(&hcan1) != HAL_OK)
//  {
//	  Error_Handler();
//  }
//  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
//  {
//	  Error_Handler();
//  }
 	CAN_GeneralSetup(&hcan1);
  	HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	
	  HAL_Delay(1);
	  Can_Send(1024, mockdata);
	  counter++;
	  new_power++;
	  new_rccConn++;
	  new_tugConn++;
	  new_tugstat++;
	  new_signalVal++;
	  new_hydroLev++;
	  new_hydroPres++;
	  new_batt1+=2;
	  new_batt2+=2;
	  Write_Bit();
	  switch(counter)
	  {
	  case 2:
		  new_power = 0;
		  new_tugConn = 0;
		  new_rccConn = 0;
		  break;
	  case 3:
		 new_tugstat = 0;
		 break;
	  case 6:
		  new_signalVal = 0;
		  break;
	  case 8:
		  counter = 0;
		  break;
	  }
	  HAL_Delay(50);

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	memset(RxData, 0, 8);
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx_receive, RxData);
//	terima++;
//
//}
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
