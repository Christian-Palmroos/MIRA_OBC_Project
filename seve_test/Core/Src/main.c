/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include "bmp280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OUTPUT_BUF_SIZE 48
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
BMP280_HandleTypedef bmp280;
I2C_HandleTypeDef *bmp2_port = NULL;
struct data {
    uint32_t pressure, humidity;
    int32_t temperature;
} bmp_data;
//uint8_t output[sizeof(bmp_data) * 2 + 1];
uint8_t output[OUTPUT_BUF_SIZE];
uint8_t output_ctr = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void hex_byte(uint8_t data, uint8_t p[]) {
    uint8_t temp;

    temp = data >> 4;
    temp += '0';
    if (temp >= (10 + '0')) {
        temp += ('A' - 10 - '0');
    }
    p[0] = temp;
    temp = data & 0x0F;
    temp += '0';
    if (temp >= (10 + '0')) {
        temp += ('A' - 10 - '0');
    }
    p[1] = temp;
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
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_MspInit(&hlpuart1);
  HAL_I2C_MspInit(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&hlpuart1, "Hello!\n", 7, 100);
  HAL_Delay (5000);

  if (HAL_I2C_IsDeviceReady(&hi2c1, BMP280_ADDRESS, 10, 10) == HAL_OK) {
	  bmp2_port = &hi2c1;
	  HAL_UART_Transmit(&hlpuart1, "BMP280\n", 7, 100);
  }
  else {
	  HAL_UART_Transmit(&hlpuart1, "ERRINIT\n", 8, 100);
	  for (int i = 0; i < 20; i++) {
		  HAL_GPIO_TogglePin (LED1_GPIO_Port, LED1_Pin);
		  HAL_Delay (100);
	  }
  }

  if (bmp2_port) {
	  bmp280_init_default_params(&bmp280.params);
	  bmp280.addr = BMP280_I2C_ADDRESS_0;
	  bmp280.i2c = bmp2_port;
	  if (!bmp280_init(&bmp280, &bmp280.params)) {
		  // failure
	  } else {
		  //success
	  }
  }

  while (1)
  {
    HAL_GPIO_TogglePin (LED1_GPIO_Port, LED1_Pin);

    if (bmp2_port) {
		while (bmp280_is_measuring(&bmp280));
		bmp280_read_fixed(&bmp280, &bmp_data.temperature, &bmp_data.pressure, &bmp_data.humidity);
		for (uint32_t i = 0; i < sizeof(bmp_data); i++) {
			hex_byte(*((uint8_t *) &bmp_data + i), output + i * 2);
		}
		output[sizeof(bmp_data)] = '\n';
		HAL_UART_Transmit(&hlpuart1, output, sizeof(bmp_data) + 1, 100);
	} else {
		HAL_UART_Transmit(&hlpuart1, "Error\n", 6, 100);
		HAL_Delay(1000);
	}

    HAL_GPIO_TogglePin (LED1_GPIO_Port, LED1_Pin);
    HAL_Delay (1000);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
