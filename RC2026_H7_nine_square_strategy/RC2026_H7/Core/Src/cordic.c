/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cordic.c
  * @brief   This file provides code for the configuration
  *          of the CORDIC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cordic.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CORDIC_HandleTypeDef hcordic;
DMA_HandleTypeDef hdma_cordic_rd;
DMA_HandleTypeDef hdma_cordic_wr;

/* CORDIC init function */
void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspInit 0 */

  /* USER CODE END CORDIC_MspInit 0 */
    /* CORDIC clock enable */
    __HAL_RCC_CORDIC_CLK_ENABLE();

    /* CORDIC DMA Init */
    /* CORDIC_RD Init */
    hdma_cordic_rd.Instance = DMA1_Stream4;
    hdma_cordic_rd.Init.Request = DMA_REQUEST_CORDIC_READ;
    hdma_cordic_rd.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_cordic_rd.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cordic_rd.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cordic_rd.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_cordic_rd.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_cordic_rd.Init.Mode = DMA_NORMAL;
    hdma_cordic_rd.Init.Priority = DMA_PRIORITY_LOW;
    hdma_cordic_rd.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_cordic_rd) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(cordicHandle,hdmaOut,hdma_cordic_rd);

    /* CORDIC_WR Init */
    hdma_cordic_wr.Instance = DMA1_Stream5;
    hdma_cordic_wr.Init.Request = DMA_REQUEST_CORDIC_WRITE;
    hdma_cordic_wr.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_cordic_wr.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cordic_wr.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cordic_wr.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_cordic_wr.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_cordic_wr.Init.Mode = DMA_NORMAL;
    hdma_cordic_wr.Init.Priority = DMA_PRIORITY_LOW;
    hdma_cordic_wr.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_cordic_wr) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(cordicHandle,hdmaIn,hdma_cordic_wr);

    /* CORDIC interrupt Init */
    HAL_NVIC_SetPriority(CORDIC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CORDIC_IRQn);
  /* USER CODE BEGIN CORDIC_MspInit 1 */

  /* USER CODE END CORDIC_MspInit 1 */
  }
}

void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspDeInit 0 */

  /* USER CODE END CORDIC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CORDIC_CLK_DISABLE();

    /* CORDIC DMA DeInit */
    HAL_DMA_DeInit(cordicHandle->hdmaOut);
    HAL_DMA_DeInit(cordicHandle->hdmaIn);

    /* CORDIC interrupt Deinit */
    HAL_NVIC_DisableIRQ(CORDIC_IRQn);
  /* USER CODE BEGIN CORDIC_MspDeInit 1 */

  /* USER CODE END CORDIC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
