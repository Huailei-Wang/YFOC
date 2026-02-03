/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 8;
  hfdcan1.Init.NominalTimeSeg1 = 25;
  hfdcan1.Init.NominalTimeSeg2 = 8;
  hfdcan1.Init.DataPrescaler = 5;
  hfdcan1.Init.DataSyncJumpWidth = 8;
  hfdcan1.Init.DataTimeSeg1 = 25;
  hfdcan1.Init.DataTimeSeg2 = 8;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
__IO CAN_t can = {0};
/**
  * @brief   初始化滤波器
  */
void USER_FDCAN_Filter_Init(void){
  // 过滤器结构体
  FDCAN_FilterTypeDef  sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;		//标准帧
  sFilterConfig.FilterIndex = 0;					//几路can就是几
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
#if motor_type == 4310
  sFilterConfig.FilterID1 = 0x1ff;
  sFilterConfig.FilterID2 = 0x7ff;
#elif motor_type == 3505
  sFilterConfig.FilterID1 = 0x200;
  sFilterConfig.FilterID2 = 0x7ff;
#else
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;
#endif
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
  HAL_FDCAN_Start(&hfdcan1);
}


FDCAN_TxHeaderTypeDef tx_message;
uint8_t can_send_data[8];
void CAN_cmd(int16_t encoder, int16_t speed, int16_t current, uint8_t temperature){
  tx_message.IdType = FDCAN_STANDARD_ID;
#if motor_type == 3505
#if motor_id==4
  tx_message.Identifier = 0x204;//4
#elif motor_id==3
  tx_message.Identifier = 0x203;//3
#elif motor_id==2
  tx_message.Identifier = 0x202;//2
#elif motor_id==1
  tx_message.Identifier = 0x201;//1
#endif
#elif motor_type == 4310
#if motor_id==4
  tx_message.Identifier = 0x208;//4
#elif motor_id==3
  tx_message.Identifier = 0x207;//3
#elif motor_id==2
  tx_message.Identifier = 0x206;//2
#elif motor_id==1
  tx_message.Identifier = 0x205;//1
#endif
#endif
  tx_message.TxFrameType = FDCAN_DATA_FRAME;
  tx_message.DataLength = 0x08;
  can_send_data[0] = (encoder >> 8);
  can_send_data[1] = encoder;
  can_send_data[2] = (speed >> 8);
  can_send_data[3] = speed;
  can_send_data[4] = (current >> 8);
  can_send_data[5] = current;
  can_send_data[6] = temperature;
  can_send_data[7] = 0x00;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, can_send_data);
}

static CAN_Data_Callback can_data_callback = NULL;
uint8_t whl_num2 = 0;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan ,uint32_t RxFifo0ITs){
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE){
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
    whl_num2 ++;
    if (rx_header.IdType == FDCAN_STANDARD_ID){
      switch (rx_header.Identifier){
#if motor_type == 4310
      case 0x1FF:
        if (can_data_callback != NULL) can_data_callback(rx_data, rx_header.DataLength);
        break;
#elif motor_type == 3505
      case 0x200:
        if (can_data_callback != NULL) can_data_callback(rx_data, rx_header.DataLength);
        break;
#endif
      default:
        break;
      }
    }
  }
}


// 注册回调函数
void CAN_RegisterCallback(CAN_Data_Callback callback){
  can_data_callback = callback;
}

/* USER CODE END 1 */
