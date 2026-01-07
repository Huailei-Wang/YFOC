/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"

  typedef struct {
    __IO FDCAN_RxHeaderTypeDef CAN_RxMsg;
    __IO uint8_t rxData[32];

    __IO FDCAN_TxHeaderTypeDef CAN_TxMsg;
    __IO uint8_t txData[32];

    __IO bool rxFrameFlag;
  }CAN_t;


/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

  void USER_FDCAN_Filter_Init(void);
  typedef void (*CAN_Data_Callback)(uint8_t* data, size_t len);
  void CAN_cmd(int16_t encoder, int16_t speed, int16_t current, uint8_t temperature);
  void CAN_RegisterCallback(CAN_Data_Callback callback);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

