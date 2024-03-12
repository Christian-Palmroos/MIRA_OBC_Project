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

// BMP
//#include "bmp3.h"
//#include "bmp3_defs.h"
//#include "bmp390_task.h"
//#include "common_porting.h"

// Gyro, I2C
//#include "../../Drivers/BSP/Components/lsm6dso/lsm6dso.h"
//#include "../../Drivers/BSP/Components/lsm6dso/lsm6dso_reg.h"
#include "custom_bus.h"

//LoRa
#include "lora_sx1276.h"

//include the library
#include "nmea_parse.h"
#include <inttypes.h>

////MIRA
//#include "mira.h"

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

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
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
// Functions to translate from hex to bytes and chars, borrowed from Philipp
//char output[32];
//uint8_t output_ctr = 0;
//unsigned int hexstatus;
//
//void hex_byte(uint8_t data, uint8_t p[]) {
//	uint8_t temp;
//
//	temp = data >> 4;
//	temp += '0';
//	if (temp >= (10 + '0')) {
//		temp += ('A' - 10 - '0');
//	}
//	p[0] = temp;
//	temp = data & 0x0F;
//	temp += '0';
//	if (temp >= (10 + '0')) {
//		temp += ('A' - 10 - '0');
//	}
//	p[1] = temp;
//}
//
//uint32_t put_one_char(char c) {
//	output[output_ctr] = c;
//	if (output_ctr < 32 - 1) {
//		output_ctr++;
//		return 0;
//	}
//	return 1;
//}
//
//uint32_t puthex(uint8_t x) {
//	char c;
//	if (((x & 0xF0) >> 4) > 9) {
//		c = ((x & 0xF0) >> 4) + 55;
//	} else {
//		c = ((x & 0xF0) >> 4) + 48;
//	}
//	if (put_one_char(c)) return 1;
//
//	if (((x & 0x0F)) > 9) {
//		c = (x & 0x0F) + 55;
//	} else {
//		c = (x & 0x0F) + 48;
//	}
//	if (put_one_char(c)) return 1;
//
//	put_one_char(0);
//	return 0;
//}
//
//uint32_t puthexword(uint16_t x) {
//	if (puthex((x & 0xFF00) >> 8)) return 1;
//	if (puthex((x & 0x00FF))) return 1;
//	return 0;
//}
//
//uint32_t putdecimal16(uint16_t x, uint8_t zeros) {
//	char c;
//	uint8_t r;
//
//	r = x / 10000;
//	if ((r) || (zeros > 3)) put_one_char(r + 48);
//	x = x - 10000 * r;
//
//	r = x / 1000;
//	if ((r) || (zeros > 2)) put_one_char(r + 48);
//	x = x - 1000 * r;
//
//	r = x / 100;
//	if ((r) || (zeros > 1)) put_one_char(r + 48);
//	x = x - 100 * r;
//
//	r = x / 10;
//	if ((r) || (zeros)) put_one_char(r + 48);
//	x = x - 10 * r;
//
//	put_one_char(x + 48);
//
//	return 0;
//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//		HAL_GPIO_WritePin(RX_EN_1_GPIO_Port, RX_EN_1_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(TX_EN_1_GPIO_Port, TX_EN_1_Pin, GPIO_PIN_SET);
//		mira_ready_for_comm = 1;
//}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//		HAL_GPIO_WritePin(RX_EN_1_GPIO_Port, RX_EN_1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(TX_EN_1_GPIO_Port, TX_EN_1_Pin, GPIO_PIN_RESET);
//}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	//LoRa
	lora_sx1276 lora;


	// Time progress tracking using tick
	double system_time_counter;
	system_time_counter = 0;
	uint8_t system_time_buffer[25] = {0};

	// HAL status for status checks for HAL functions
	HAL_StatusTypeDef status;

	// USB buffers to receive PC commands (2-byte command, first byte target device or flightmode, second byte the command for the device. If no command desired input USB_PLACEHOLDER as second byte.)
	//uint8_t usb_Rx_buffer[2];
	int8_t usb_status;

	static uint8_t USB_PLACEHOLDER = 0x00;

	static uint8_t USB_CHECKSTATUS = 0x01;
	static uint8_t USB_TESTOUTPUT = 0x02;

	static uint8_t USB_MIRA = 0x01;
	static uint8_t USB_LORA = 0x02;
	static uint8_t USB_GYRO = 0x03;
	static uint8_t USB_BMP = 0x04;
	static uint8_t USB_GPS = 0x05;
	static uint8_t USB_SD = 0x06;
	static uint8_t USB_TIMERS = 0x07;

	static uint8_t USB_PING = 0x08;
	static uint8_t USB_FLIGHTMODE = 0x09;

	uint8_t lora_Rx_buffer[64];
	uint8_t lora_error;

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	// Initialize HAL for UART interrupts
	HAL_MspInit();
	// Initialize I2C2 with custom driver
	BSP_I2C2_Init();

	//Initialize Msp for both UARTs
	HAL_UART_MspInit(&huart1);
	HAL_UART_MspInit(&huart2);

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
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


	// WAIT FOR USB CONNECTION
	HAL_Delay(8000);
	while (CDC_Transmit_FS ("START\n", 6) == USBD_BUSY);


	/// LoRa Init /////////////////////////////////////////////////////////////////////////////////
	uint8_t lora_res = lora_init(&lora, &hspi1, LORA_NSS_GPIO_Port, LORA_NSS_Pin, LORA_BASE_FREQUENCY_US);
	if (lora_res != LORA_OK) {
		// Initialization failed
		while (CDC_Transmit_FS ("LORA INIT NOT OK!\n", 18) == USBD_BUSY);
	}
	lora_res = lora_send_packet(&lora, (uint8_t *)"test", 4);
	if (lora_res != LORA_OK) {
		// Send failed
		while (CDC_Transmit_FS ("LORA SEND NOT OK!\n", 18) == USBD_BUSY);
	}
	if (lora_res == LORA_OK) {
		// All good
		while (CDC_Transmit_FS ("LORA OK!\n", 9) == USBD_BUSY);
	}

	/// LoRa test send /////////////////////////////////////////////////////////////////////////////////




	/// LoRa send /////////////////////////////////////////////////////////////////////////////////





	/// System timers Init /////////////////////////////////////////////////////////////////////////////////

	// Start timers
	HAL_TIM_Base_Start_IT(&htim17);
	tick = 0;
	tickGPS = 0;



	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	/// Pre-main program /////////////////////////////////////////////////////////////////////////////////
	/// Pre-main program /////////////////////////////////////////////////////////////////////////////////
	/// Pre-main program /////////////////////////////////////////////////////////////////////////////////
	HAL_GPIO_TogglePin (LED0_GPIO_Port, LED0_Pin);
	while (1) {

		while (usb_Rx_ready == 0);
		usb_Rx_ready = 0;

		if (usb_Rx_buffer[0] == USB_LORA) {
			if (usb_Rx_buffer[1] == USB_CHECKSTATUS) {

			}
			else if (usb_Rx_buffer[1] == USB_TESTOUTPUT) {
				lora_res = lora_send_packet(&lora, &lora_test_packet, sizeof(lora_test_packet));
				if (lora_res != LORA_OK) {
					while (CDC_Transmit_FS ("\nERROR!\n", sizeof("\nERROR!\n")) == USBD_BUSY);
				}
				else {
					while (CDC_Transmit_FS ("\nLora packet sent!\n", sizeof("\nLora packet sent!\n")) == USBD_BUSY);
				}

			}

		}


		else if (usb_Rx_buffer[0] == USB_TIMERS) {
			if (usb_Rx_buffer[1] == USB_CHECKSTATUS) {

			}
			else if (usb_Rx_buffer[1] == USB_TESTOUTPUT) {
				if (tick == 0) {
					tick = 10;
					while (tick != 0);
					while (CDC_Transmit_FS ("\nTick works!\n", 13) == USBD_BUSY);
				}

				if (tickGPS == 0) {
					tickGPS = 10;
					while (tickGPS != 0);
					while (CDC_Transmit_FS ("\nGPStick works!\n", 16) == USBD_BUSY);
				}
			}

		}


		else if (usb_Rx_buffer[1] == USB_FLIGHTMODE) {
			while (CDC_Transmit_FS ("OK", 2) == USBD_BUSY);
			break;
		}


		else if (usb_Rx_buffer[1] == USB_PING) {

			while (CDC_Transmit_FS ("PONG", 4) == USBD_BUSY);
			//break;
		}

	}
	HAL_GPIO_TogglePin (LED0_GPIO_Port, LED0_Pin);
	// Reset timers before main program
	tick = 0;
	tickGPS = 0;

	HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
	/// Main program /////////////////////////////////////////////////////////////////////////////////
	/// Main program /////////////////////////////////////////////////////////////////////////////////
	/// Main program /////////////////////////////////////////////////////////////////////////////////
	while (1)
	{


		lora_res = lora_receive_packet(&lora, &lora_Rx_buffer, sizeof(lora_Rx_buffer), &lora_error);
		while (CDC_Transmit_FS (lora_Rx_buffer, sizeof(lora_Rx_buffer)) == USBD_BUSY);
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
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
	/* USART1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
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
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
	huart1.Init.BaudRate = 115200;
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
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

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
	HAL_GPIO_WritePin(GPIOC, LORA_RST_Pin|LORA_NSS_Pin, GPIO_PIN_SET);

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

	/*Configure GPIO pins : LORA_DIG0_Pin CARD_DETECT_Pin */
	GPIO_InitStruct.Pin = LORA_DIG0_Pin|CARD_DETECT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
