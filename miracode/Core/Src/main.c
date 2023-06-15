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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "stm32l4xx_it.h"

#include "usbd_cdc.h"

#include "bmp3.h"
#include "bmp3_defs.h"
#include "bmp390_task.h"
#include "common_porting.h"

//include the library
#include "nmea_parse.h"
#define BufferSize 512

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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM17_Init(void);
static void MX_NVIC_Init(void);
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

	FRESULT res; /* FatFs function common result code */
	UINT byteswritten, bytesread; /* File write/read counts */
	uint8_t wtext[50] = "STM32 FATFS works great!"; /* File write buffer. */
	uint8_t rtext[100];/* File read buffer */
	uint8_t usberr;

	// For GPS Module
	HAL_StatusTypeDef UART2_Rx_STATUS;
	uint8_t UART2_RxBuffer[272];

	// For USB Transmission
	USBD_HandleTypeDef hUsbDeviceFS;
	uint8_t USB_Tx_STATUS;
	uint8_t *data = "Hello!\n";

	uint8_t USB_TxBuffer_FS;
	uint32_t USB_TxBuffer_Length = 1000;
	uint8_t USBD_TxBuffer_Status;

	//create a GPS data structure
	GPS myData;

	//read serial data to a buffer,
	//serial readout implementation may vary depending on your needs
	//the library is able to work with any buffer size, as long as it contains at least one whole NMEA message
	uint8_t DataBuffer[BufferSize];

	int8_t rslt;
	uint16_t settings_sel;
	struct bmp3_dev dev;
	struct bmp3_data bmpdata = { 0 };
	struct bmp3_settings settings = { 0 };
	struct bmp3_status status = { { 0 } };

	/* Interface reference is given as a parameter
	 *         For I2C : BMP3_I2C_INTF
	 *         For SPI : BMP3_SPI_INTF
	 */
	rslt = bmp3_interface_init(&dev, BMP3_I2C_INTF);
	bmp3_check_rslt("bmp3_interface_init", rslt);

	rslt = bmp3_init(&dev);
	bmp3_check_rslt("bmp3_init", rslt);

	settings.int_settings.drdy_en = BMP3_DISABLE;
	settings.press_en = BMP3_ENABLE;
	settings.temp_en = BMP3_ENABLE;

	settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
	settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
	settings.odr_filter.odr = BMP3_ODR_100_HZ;

	settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
				   BMP3_SEL_DRDY_EN;

	rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
	bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

	settings.op_mode = BMP3_MODE_NORMAL;
	rslt = bmp3_set_op_mode(&settings, &dev);
	bmp3_check_rslt("bmp3_set_op_mode", rslt);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  // SD reader
  MX_FATFS_Init();

  //HAL_UART_MspInit(&huart1);
  //HAL_UART_MspInit(&huart2);

  // This returned 0'\0', even though it's supposed to return either USBD_OK or USBD_FAIL
//   USBD_TxBuffer_Status = USBD_CDC_SetTxBuffer(&hUsbDeviceFS, USB_TxBuffer_FS, USB_TxBuffer_Length);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_TIM17_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  volatile unsigned tmp;

  // Setting the buffer for UART2 data reading
  rxBuffer = rxBuffer1;
  ATOMIC_SET_BIT(huart2.Instance->CR1, USART_CR1_UE);
  ATOMIC_SET_BIT(huart2.Instance->CR1, USART_CR1_RE);
  ATOMIC_SET_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE_RXFNEIE);


  // If not FR_OK, mounting failed, else it was successful
  if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)
      	{
  	  	  	  HAL_GPIO_TogglePin (LED0_GPIO_Port, LED0_Pin);
  	  	  	  HAL_Delay (300);
  	  	  	  HAL_GPIO_TogglePin (LED0_GPIO_Port, LED0_Pin);
  	  	  	  HAL_Delay (1000);
      	}
  // here f_mount == FR_OK -> mounting was a success
  else
      	{
	  // f_mkfs
	  if(f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, rtext, sizeof(rtext)) != FR_OK)
      	    {
  				  HAL_GPIO_TogglePin (LED1_GPIO_Port, LED1_Pin);
  				  HAL_Delay (300);
  				  HAL_GPIO_TogglePin (LED1_GPIO_Port, LED1_Pin);
  				  HAL_Delay (1000);
      	    }
	  else
      		{
			// Open file for writing (Create)
			if(f_open(&SDFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
				{
				  HAL_GPIO_TogglePin (LED2_GPIO_Port, LED2_Pin);
				  HAL_Delay (300);
				  HAL_GPIO_TogglePin (LED2_GPIO_Port, LED2_Pin);
				  HAL_Delay (1000);
				}
			else
				{

				// Write to the text file
				res = f_write(&SDFile, wtext, strlen((char *)wtext), (void *)&byteswritten);
				f_read(&SDFile, &rtext, 100, &bytesread);
				//f_read();

				usberr = CDC_Transmit_FS(rtext,  sizeof(rtext));
				if((byteswritten == 0) || (res != FR_OK))
					{
					  HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
					  HAL_Delay (300);
					  HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
					  HAL_Delay (1000);
					}
				else
					{

					f_close(&SDFile);
					}

      			}
      		}
      	}
      	f_mount(&SDFatFS, (TCHAR const*)NULL, 0);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Lesson learned: do NOT place delays between data transfers and receives; it will mess up the data flow

	  // Check here if data is ready
	  if (data_ready)
	  {

		  //when enough data is received point it to the parser,
		  //do it outside of a UART interrupt to avoid overrun errors
		  /*nmea_parse(&myData, DataBuffer);

		  //if(myData.fix == 1) {
		      //do something with the data
		      //at ex.
		      double latitude = myData.latitude;
		      double longitude = myData.longitude;
		      while (CDC_Transmit_FS ("Latitude and longitude:\n", 24) == USBD_BUSY);
		      while (CDC_Transmit_FS ((uint8_t)latitude, strlen((uint8_t)latitude)) == USBD_BUSY);
		      while (CDC_Transmit_FS ("\n", 1) == USBD_BUSY);
		      while (CDC_Transmit_FS ((uint8_t)longitude, strlen((uint8_t)longitude)) == USBD_BUSY);
		      while (CDC_Transmit_FS ("\n", 1) == USBD_BUSY);

		  //}*/

		  if (rxBuffer == rxBuffer1)
		  {
			  // USBD_TxBuffer_Status = USBD_CDC_SetTxBuffer (&hUsbDeviceFS, rxBuffer2, 120);

			  // Saving the transmit status for debugging
			  // USB_Tx_STATUS = CDC_Transmit_FS (rxBuffer2, strlen(rxBuffer2));
			  /*USBD_TxBuffer_Status = CDC_Transmit_FS (rxBuffer2, strlen(rxBuffer2));
			  while (USBD_TxBuffer_Status == USBD_BUSY);
			  	  USBD_TxBuffer_Status = CDC_Transmit_FS (rxBuffer2, strlen(rxBuffer2));*/

			  HAL_GPIO_TogglePin (LED2_GPIO_Port, LED2_Pin);
			  while (CDC_Transmit_FS (rxBuffer2, strlen(rxBuffer2)) == USBD_BUSY);
			  HAL_GPIO_TogglePin (LED2_GPIO_Port, LED2_Pin);
			  //rxBuffer = rxBuffer2;
			  data_ready ^= 1;
			  send_ready |= 1;
			  //while (CDC_Transmit_FS ("\n", 1) == USBD_BUSY);

		  }
		  else
		  {
			  // Saving the transmit status for debugging
			  // USB_Tx_STATUS = CDC_Transmit_FS (rxBuffer1, strlen(rxBuffer1));
			  HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
			  while (CDC_Transmit_FS (rxBuffer1, strlen(rxBuffer1)) == USBD_BUSY);
			  HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
			  //rxBuffer = rxBuffer1;
			  data_ready ^= 1;
			  send_ready |= 1;
			  //while (CDC_Transmit_FS ("\n", 1) == USBD_BUSY);

		  }

		  //data_ready = 0;
	  }
	  /*else
	  {
		  // Flash LED4
		  HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
		  //HAL_Delay(100);
	  }*/

	  rslt = bmp3_get_status(&status, &dev);
		bmp3_check_rslt("bmp3_get_status", rslt);


		/* Read temperature and pressure data iteratively based on data ready interrupt */
		if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE))
		{
			/*
			 * First parameter indicates the type of data to be read
			 * BMP3_PRESS_TEMP : To read pressure and temperature data
			 * BMP3_TEMP       : To read only temperature data
			 * BMP3_PRESS      : To read only pressure data
			 */
			rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &bmpdata, &dev);
			bmp3_check_rslt("bmp3_get_sensor_data", rslt);

			/* NOTE : Read status register again to clear data ready interrupt status */
			rslt = bmp3_get_status(&status, &dev);
			bmp3_check_rslt("bmp3_get_status", rslt);


			//#ifdef BMP3_FLOAT_COMPENSATION
			while (CDC_Transmit_FS ("BMP390\n", 7) == USBD_BUSY);
			while (CDC_Transmit_FS (&bmpdata.temperature, strlen((uint16_t)bmpdata.temperature)) == USBD_BUSY);
			while (CDC_Transmit_FS ("\n", 1) == USBD_BUSY);
			while (CDC_Transmit_FS (&bmpdata.pressure, strlen((uint16_t)bmpdata.pressure)) == USBD_BUSY);
			while (CDC_Transmit_FS ("\n", 1) == USBD_BUSY);
		}


	  // This returns HAL_TIMEOUT ???
	  // UART2_Rx_STATUS = HAL_UART_Receive (&huart2, UART2_RxBuffer, sizeof(UART2_RxBuffer), 5000);
	  // UART2_Rx_STATUS = HAL_BUSY;

	  // Transmit should be handled through the USB port
	  /*
	  if(UART2_Rx_STATUS == HAL_OK)
	  	  {
		  // HAL_UART_Transmit (&huart1, UART2_rxBuffer, sizeof(UART2_rxBuffer), 5000);
		  // HAL_UART_Transmit (&huart1, "b \n", 3, 5000);

		  CDC_Transmit_FS ("b \n", 3);

		  // Flash LED1 twice
		  HAL_GPIO_TogglePin (LED0_GPIO_Port, LED0_Pin);
		  HAL_Delay (200);
		  HAL_GPIO_TogglePin (LED0_GPIO_Port, LED0_Pin);
		  HAL_Delay (100);
		  HAL_GPIO_TogglePin (LED0_GPIO_Port, LED0_Pin);
		  HAL_Delay (200);
		  HAL_GPIO_TogglePin (LED0_GPIO_Port, LED0_Pin);
	  	  }
	  else
	  	  {

		  // Flash LED4 twice
		  HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
		  HAL_Delay (200);
		  HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
		  HAL_Delay (100);
		  HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
		  HAL_Delay (200);
		  HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
		  }
		*/

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV20;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_TRG_COM_TIM17_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
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
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  hsd1.Init.Transceiver = SDMMC_TRANSCEIVER_DISABLE;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 999;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 11999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart1.Init.BaudRate = 115000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RX_EN_2_Pin|TX_EN_2_Pin|RX_EN_1_Pin|TX_EN_1_Pin
                          |MIRA_EN_PWR_Pin|OCPEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LORA_RST_Pin|LORA_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RX_EN_2_Pin TX_EN_2_Pin RX_EN_1_Pin TX_EN_1_Pin
                           MIRA_EN_PWR_Pin OCPEN_Pin */
  GPIO_InitStruct.Pin = RX_EN_2_Pin|TX_EN_2_Pin|RX_EN_1_Pin|TX_EN_1_Pin
                          |MIRA_EN_PWR_Pin|OCPEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CHG_INT_Pin */
  GPIO_InitStruct.Pin = CHG_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHG_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_DIG0_Pin */
  GPIO_InitStruct.Pin = LORA_DIG0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_DIG0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_RST_Pin LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LORA_RST_Pin|LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OCPFAULT_Pin */
  GPIO_InitStruct.Pin = OCPFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(OCPFAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CARD_DETECT_Pin */
  GPIO_InitStruct.Pin = CARD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CARD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : POWERGOOD_Pin */
  GPIO_InitStruct.Pin = POWERGOOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(POWERGOOD_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  printf("Error_Handler() called");
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
