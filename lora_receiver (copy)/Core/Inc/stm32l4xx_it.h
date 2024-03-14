/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#ifndef __STM32L4xx_IT_H
#define __STM32L4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define gps_RXBUFSIZE 800
extern uint8_t gps_rxBuffer1[gps_RXBUFSIZE];
extern uint8_t gps_rxBuffer2[gps_RXBUFSIZE];

extern volatile uint8_t *gps_rxBuffer;
extern volatile uint8_t gps_rxBufferPos;
extern volatile uint8_t tick;
extern volatile uint8_t tickGPS;
extern volatile uint8_t step;
extern volatile unsigned gps_data_ready;
extern volatile unsigned gps_send_ready;

#define mira_RXBUFSIZE 10
extern uint8_t mira_rxBuffer1[mira_RXBUFSIZE];
extern uint8_t mira_rxBuffer2[mira_RXBUFSIZE];

extern volatile uint8_t *mira_rxBuffer;
extern volatile uint8_t mira_rxBufferPos;
extern volatile unsigned mira_data_ready;
extern volatile unsigned mira_send_ready;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);
void TIM1_TRG_COM_TIM17_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void OTG_FS_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32L4xx_IT_H */
