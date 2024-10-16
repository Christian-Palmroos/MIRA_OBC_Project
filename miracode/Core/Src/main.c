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
#include "bmp3.h"
#include "bmp3_defs.h"
#include "bmp390_task.h"
#include "common_porting.h"

// Gyro, I2C
#include "../../Drivers/BSP/Components/lsm6dso/lsm6dso.h"
#include "../../Drivers/BSP/Components/lsm6dso/lsm6dso_reg.h"
#include "custom_bus.h"

//LoRa
#include "lora_sx1276.h"

//include the library
#include "nmea_parse.h"
#include <inttypes.h>

//MIRA
#include "mira.h"

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
DMA_HandleTypeDef hdma_usart2_tx;

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
char output[32];
uint8_t output_ctr = 0;
unsigned int hexstatus;

/*
 * Converts 8-bit unsigned integer into its corresponding two-character
 * hexadecimal ASCII-representation.
 * @param data: integer to be converted into ascii hexadecimal string
 * @param p[]: array where the two-character hexadecimal representation will be stored
 */
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

/*
 * Inserts a character into an array "output" at the current position specified by output_ctr.
 * @param c: character
 */
uint32_t put_one_char(char c) {
	output[output_ctr] = c;
	if (output_ctr < 32 - 1) {
		output_ctr++;
		return 0;
	}
	return 1;
}

/*
 * Converts an unsigned integer to its two-character hexadecimal string representation and attempts
 * to insert each character into an output buffer.
 * @param x: integer input
 * @return 1 if array becomes full, otherwise 0
 */
uint32_t puthex(uint8_t x) {
	char c;
	if (((x & 0xF0) >> 4) > 9) {
		c = ((x & 0xF0) >> 4) + 55;
	} else {
		c = ((x & 0xF0) >> 4) + 48;
	}
	if (put_one_char(c)) return 1;

	if (((x & 0x0F)) > 9) {
		c = (x & 0x0F) + 55;
	} else {
		c = (x & 0x0F) + 48;
	}
	if (put_one_char(c)) return 1;

	put_one_char(0);
	return 0;
}

/*
 * Converts a 16-bit unsigned integer to its 4-character hexadecimal string representation.
 * @param x: 16-bit integer
 */
uint32_t puthexword(uint16_t x) {
	if (puthex((x & 0xFF00) >> 8)) return 1;
	if (puthex((x & 0x00FF))) return 1;
	return 0;
}

/*
 * Converts a 16-bit unsigned integer to its decimal representation and inserts it into an output buffer.
 * @param x: 16-bit unsigned integer
 * @param zeros: 8-bit unsigned integer that specifies the number of leading zeroes.
 * @returns 0
 */
uint32_t putdecimal16(uint16_t x, uint8_t zeros) {
	char c;
	uint8_t r;

	r = x / 10000;
	if ((r) || (zeros > 3)) put_one_char(r + 48);
	x = x - 10000 * r;

	r = x / 1000;
	if ((r) || (zeros > 2)) put_one_char(r + 48);
	x = x - 1000 * r;

	r = x / 100;
	if ((r) || (zeros > 1)) put_one_char(r + 48);
	x = x - 100 * r;

	r = x / 10;
	if ((r) || (zeros)) put_one_char(r + 48);
	x = x - 10 * r;

	put_one_char(x + 48);

	return 0;
}

/*
 * Adds ASCII data to a data buffer.
 * @param data_buffer: pointer to the data buffer
 * @param data: pointer to the data that is added to the buffer
 * @param size: int telling the size (length) of data
 */
void add_to_buffer(uint8_t* data_buffer, uint8_t* data, int size) {

	for (int i = 0; i < size; i++) {
		sprintf(data_buffer + strlen(data_buffer), "%c", data[i]);
	}

}

void add_to_buffer_hex(uint8_t* data_buffer, uint8_t* data, int size) {

	for (int i = 0; i < size; i++) {
		sprintf(data_buffer + strlen(data_buffer), "%02x ", data[i]);
	}

}




//int parse_comma_delimited_str(char *string, char **fields, int max_fields)
//{
//   int i = 0;
//   fields[i++] = string;
//   while ((i < max_fields) && NULL != (string = strchr(string, ','))) {
//      *string = '\0';
//      fields[i++] = ++string;
//   }
//   return --i;
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
	// The SD card mount, init, read, and write variables
	FRESULT sd_result_write; /* FatFs function common result code */
	UINT sd_err_byteswritten, sd_err_bytesread; /* File write/read counts */
	uint8_t sd_write_buffer[50] = "Start of operation\n"; /* File write buffer. */
	uint8_t sd_read_buffer[2048];/* File read buffer */
	FRESULT sd_status;

	// Buffer for all data to be stored into in the same way as it has been printed to the PC
	//uint8_t data_buffer[LORA_MAX_PACKET_SIZE]; //LORA_MAX_PACKET_SIZE
	uint8_t data_buffer[500];

	uint8_t gps_buffer[1000];
	char *gps_buffer_ptr1 = gps_buffer;
	char *gps_buffer_ptr2 = gps_buffer;
	uint8_t gps_parsed_buffer[1000];
	int parser_i;
	int parser_sub_i;
	gps_buffer[0] = '\0';
	gps_parsed_buffer[0] = '\0';

	// For GPS Module
	//HAL_StatusTypeDef UART2_Rx_STATUS;
	//uint8_t UART2_RxBuffer[272];

	// For USB Transmission
	//USBD_HandleTypeDef hUsbDeviceFS;
	//uint8_t USB_Tx_STATUS;
	//uint8_t *data = "Hello!\n";

	//uint8_t USB_TxBuffer_FS;
	//uint32_t USB_TxBuffer_Length = 1000;
	//uint8_t USBD_TxBuffer_Status;

	//create a GPS data structure
	//GPS myData;

	// The pressure sensor BMP390 variables
	int8_t bmp_result;
	uint16_t bmp_settings_select;
	struct bmp3_dev bmp_device;
	struct bmp3_data bmp_data = { 0 };
	struct bmp3_settings bmp_settings = { 0 };
	struct bmp3_status bmp_status = { { 0 } };
	uint8_t bmp_temperature_buffer[25] = {0};
	uint8_t bmp_pressure_buffer[25] = {0};

	//test
	//HAL_StatusTypeDef i2c2status;
	//uint8_t hello[7] = "Hello!\n";
	uint8_t i2c2check_active_address[25] = {0};
	uint8_t i2c2check_space[] = " - ";

	// The gyroscope LSM6DSO variables
	LSM6DSO_Object_t gyro_device;
	LSM6DSO_Axes_t gyro_acceleration_object;
	uint8_t gyro_acceleration_buffer[40] = {0};
	LSM6DSO_Axes_t gyro_angularvel_object;
	uint8_t gyro_angularvel_buffer[40] = {0};
	int32_t gyro_result_acceleration;
	int32_t gyro_result_angularvel;
	int32_t gyro_result_init;
	LSM6DSO_IO_t gyro_io;
	//uint8_t GyroErrBuff[25] = {0};

	//LoRa
	lora_sx1276 lora;

	//MIRA
	uint8_t mira_target_reg = 0x00;
	uint8_t mira_Tx_payload[4] = {0x00,0x00,0x00,0x00};
	uint8_t mira_Rx_buffer[9+1];
	uint8_t mira_Rx_spec_available[13];
	uint8_t mira_science_Rx_buffer[142];
	uint8_t mira_housekeeping_Rx_buffer[19];
	uint8_t mira_response_Rx_buffer[9+1];
	uint8_t mira_integration_time = 15;//(mira_write_IT[0]<<24)|(mira_write_IT[3]<<16)|(mira_write_IT[2]<<8)|mira_write_IT[3];
	mira_integration_time = mira_integration_time * 10;
	int MIRA_HK_check = 0;

	//	SX1278_hw_t SX1278_hw;
	//	SX1278_t SX1278;
	//	int master;
	//	//int ret;
	//	char buffer[512];
	//	int message;
	//	int message_length;

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

	uint8_t lora_test_packet[10] = {0,1,2,3,4,5,6,7,8,9};


	uint8_t uart_receive_dump[1000];

	int PRINT_TOGGLE = 0;
	int uart_dump = 1;

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

	// Initialize SD card
	BSP_SD_Init();

	// Power on LED
	HAL_GPIO_TogglePin (LED0_GPIO_Port, LED0_Pin);

	// WAIT FOR USB CONNECTION
	// Comment this out once build finished
	if (PRINT_TOGGLE == 1) {
		while (CDC_Transmit_FS ("START\n", 6) == USBD_BUSY); }

	// dump uart line contents to start clean
	HAL_UART_Receive(&huart1, uart_receive_dump, sizeof(uart_receive_dump), 1000);


	/// MIRA Init /////////////////////////////////////////////////////////////////////////////////


	// Enable MIRA power from OBC
	HAL_GPIO_WritePin(MIRA_EN_PWR_GPIO_Port, MIRA_EN_PWR_Pin, GPIO_PIN_SET);
	// Enable Over Current Protection at U4
	HAL_GPIO_WritePin(OCPEN_GPIO_Port, OCPEN_Pin, GPIO_PIN_SET);

	// disable channel 1 for MIRA communication
	HAL_GPIO_WritePin(RX_EN_1_GPIO_Port, RX_EN_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TX_EN_1_GPIO_Port, TX_EN_1_Pin, GPIO_PIN_RESET);

	// enable channel 2
	HAL_GPIO_WritePin(RX_EN_2_GPIO_Port, RX_EN_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TX_EN_2_GPIO_Port, TX_EN_2_Pin, GPIO_PIN_SET);

	HAL_Delay(8000);

	status = mira_init(&huart1, 5000);
	if (status != HAL_OK) {
		HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
		HAL_Delay(800);
		HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
		HAL_Delay(200);
		//status = mira_init(&huart1, 5000);
	}




	//	while(status != HAL_OK){
	//
	//		HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
	//		HAL_Delay(800);
	//		HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
	//		HAL_Delay(200);
	//		status = mira_init(&huart1, 5000);
	//
	//	}

	//		while(1) {
	//			HAL_Delay(2000);
	//			//status = mira_science_data(&huart1, mira_science_Rx_buffer, sizeof(mira_science_Rx_buffer), 5000);
	//			status = mira_housekeeping_data(&huart1, mira_housekeeping_Rx_buffer, 19, 5000);
	//			HAL_GPIO_TogglePin (LED2_GPIO_Port, LED2_Pin);
	//			HAL_Delay(800);
	//			HAL_GPIO_TogglePin (LED2_GPIO_Port, LED2_Pin);
	//			HAL_Delay(200);
	//		}


	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);


	/// LoRa Init /////////////////////////////////////////////////////////////////////////////////
	uint8_t lora_res = lora_init(&lora, &hspi1, LORA_NSS_GPIO_Port, LORA_NSS_Pin, LORA_BASE_FREQUENCY_435);
	// Comment this out once build finished
	if (lora_res != LORA_OK) {
		// Initialization failed
		if (PRINT_TOGGLE == 1) {
			while (CDC_Transmit_FS ("LORA INIT NOT OK!\n", 18) == USBD_BUSY);}
	}
	if (lora_res == LORA_OK) {
		// All good
		if (PRINT_TOGGLE == 1) {
			while (CDC_Transmit_FS ("LORA OK!\n", 9) == USBD_BUSY);}
	}
	lora_res = lora_send_packet(&lora, (uint8_t *)"test", 4);
	if (lora_res != LORA_OK) {
		// Send failed
		if (PRINT_TOGGLE == 1) {
			while (CDC_Transmit_FS ("LORA SEND NOT OK!\n", 18) == USBD_BUSY);}
	}

	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);



	/// Gyro Init /////////////////////////////////////////////////////////////////////////////////

	// Set gyro io functions and values
	gyro_io.Init = BSP_I2C2_Init;
	gyro_io.DeInit = BSP_I2C2_DeInit;
	gyro_io.BusType = 0;
	gyro_io.Address = LSM6DSO_I2C_ADD_L;
	gyro_io.WriteReg = BSP_I2C2_WriteReg;
	gyro_io.ReadReg = BSP_I2C2_ReadReg;
	gyro_io.GetTick = BSP_GetTick;
	gyro_io.Delay = HAL_Delay;

	// Initialize gyro
	LSM6DSO_RegisterBusIO(&gyro_device, &gyro_io);
	gyro_result_init = LSM6DSO_Init(&gyro_device);

	// Check and print gyro device status
	// Comment this out once build finished
	if (PRINT_TOGGLE == 1) {
		if (gyro_result_init == 0) {
			while (CDC_Transmit_FS ("GYRO OK!\n", 9) == USBD_BUSY);}
		else {
			while (CDC_Transmit_FS ("GYRO NOT OK!\n", 13) == USBD_BUSY);}
	}


	// Enabling translational and angular acceleration measurements
	LSM6DSO_ACC_Enable(&gyro_device);
	LSM6DSO_GYRO_Enable(&gyro_device);
	LSM6DSO_ACC_SetOutputDataRate(&gyro_device, 104.0f);
	LSM6DSO_GYRO_SetOutputDataRate(&gyro_device, 104.0f);
	//LSM6DSO_FIFO_Set_Mode(&gyro_device, (uint8_t)3);

	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

	/// BMP Init /////////////////////////////////////////////////////////////////////////////////
	/* Interface reference is given as a parameter
	 *         For I2C : BMP3_I2C_INTF
	 *         For SPI : BMP3_SPI_INTF
	 */
	bmp_result = bmp3_interface_init(&bmp_device, BMP3_I2C_INTF);
	bmp3_check_rslt("bmp3_interface_init", bmp_result);

	bmp_result = bmp3_init(&bmp_device);
	bmp3_check_rslt("bmp3_init", bmp_result);


	bmp_settings.int_settings.drdy_en = BMP3_DISABLE;
	bmp_settings.int_settings.latch = BMP3_ENABLE;
	bmp_settings.press_en = BMP3_ENABLE;
	bmp_settings.temp_en = BMP3_ENABLE;

	bmp_settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
	bmp_settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
	bmp_settings.odr_filter.odr = BMP3_ODR_100_HZ;

	bmp_settings_select = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_DRDY_EN;

	bmp_result = bmp3_set_sensor_settings(bmp_settings_select, &bmp_settings, &bmp_device);
	bmp3_check_rslt("bmp3_set_sensor_settings", bmp_result);

	// Comment this out once build finished
	if (bmp_result == BMP3_OK) {
		if (PRINT_TOGGLE == 1) {
			while (CDC_Transmit_FS ("BMP OK!\n", 8) == USBD_BUSY);}
	}
	//	else
	//	{while (CDC_Transmit_FS ("BMP NOT OK!\n", 12) == USBD_BUSY);}

	/*bmp_settings.op_mode = BMP3_MODE_NORMAL;
	bmp_result = bmp3_set_op_mode(&bmp_settings, &bmp_device);
	bmp3_check_rslt("bmp3_set_op_mode", bmp_result);*/

	//volatile unsigned tmp;

	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

	/// GPS Init /////////////////////////////////////////////////////////////////////////////////

	// Setting the buffer for UART2 data reading
//	gps_rxBuffer = gps_rxBuffer1;
//	ATOMIC_SET_BIT(huart2.Instance->CR1, USART_CR1_UE);
//	ATOMIC_SET_BIT(huart2.Instance->CR1, USART_CR1_RE);
//	ATOMIC_SET_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE_RXFNEIE);


	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	/// SD Init /////////////////////////////////////////////////////////////////////////////////

	// Initialize SD card
	// If not FR_OK, mounting failed, else it was successful
	//	if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)
	//	{
	//		while (CDC_Transmit_FS ("Mount failed!\n", 14) == USBD_BUSY);
	//	}
	//	// here f_mount == FR_OK -> mounting was a success
	//	else
	//	{
	//		// f_mkfs
	//		if(f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, sd_read_buffer, sizeof(sd_read_buffer)) != FR_OK)
	//		{
	//			while (CDC_Transmit_FS ("MKFS failed!\n", 13) == USBD_BUSY);
	//			hsd1.Init.ClockDiv = 0;
	//		}
	//		else
	//		{
	//			hsd1.Init.ClockDiv = 0;
	//			// Open file for writing (Create)
	//			if(f_open(&SDFile, "STM32.TXT", FA_OPEN_APPEND | FA_WRITE) != FR_OK)
	//			{
	//				while (CDC_Transmit_FS ("Open file failed!\n", 18) == USBD_BUSY);
	//			}
	//			else
	//			{
	//
	//				// Write to the text file
	//				sd_result_write = f_write(&SDFile, sd_write_buffer, strlen((char *)sd_write_buffer), (void *)&sd_err_byteswritten);
	//				f_read(&SDFile, &sd_read_buffer, 100, &sd_err_bytesread);
	//
	//				//while( CDC_Transmit_FS(sd_read_buffer,  sizeof(sd_read_buffer))  == USBD_BUSY);
	//				if((sd_err_byteswritten == 0) || (sd_result_write != FR_OK))
	//				{
	//					while (CDC_Transmit_FS ("Read/Write failed!\n", 19) == USBD_BUSY);
	//				}
	//				else
	//				{
	//					f_close(&SDFile);
	//				}
	//
	//			}
	//		}
	//	}
	//	f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
	sd_status = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
	// Comment this out once build finished
	GPIO_PinState pinstate = HAL_GPIO_ReadPin(CARD_DETECT_GPIO_Port, CARD_DETECT_Pin);


	if (pinstate == GPIO_PIN_RESET) {
		HAL_GPIO_TogglePin (LED2_GPIO_Port, LED2_Pin);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin (LED2_GPIO_Port, LED2_Pin);
	}


	if(sd_status != FR_OK)
	{
		if (PRINT_TOGGLE == 1) {
			while (CDC_Transmit_FS ("Mount failed!\n", 14) == USBD_BUSY);}

		HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
		HAL_Delay(800);
		HAL_GPIO_TogglePin (LED3_GPIO_Port, LED3_Pin);
		HAL_Delay(200);

		//while (CDC_Transmit_FS (, 14) == USBD_BUSY);
		//sd_status = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
	}

	if (sd_status == FR_OK) {
		sd_status = f_open(&SDFile, "STM32.TXT", FA_OPEN_APPEND | FA_WRITE);
		if(sd_status != FR_OK)
		{
			if (PRINT_TOGGLE == 1) {
				while (CDC_Transmit_FS ("Open file failed!\n", 18) == USBD_BUSY);}
		}
	}

	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

	/// System timers Init /////////////////////////////////////////////////////////////////////////////////

	// Start timers
	HAL_TIM_Base_Start_IT(&htim17);
	tick = 600;
	tickGPS = 0;


	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	/// I2C scanning /////////////////////////////////////////////////////////////////////////////////
	// Comment this out once build finished
	//-[ I2C Bus Scanning ]-
	uint8_t i = 0, ret;
	for(i = 1; i < 128; i++)
	{
		ret = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 3, 5);
		if (ret != HAL_OK) // No ACK Received At That Address
		{
			if (PRINT_TOGGLE == 1) {
				while (CDC_Transmit_FS (i2c2check_space, strlen(i2c2check_space)) == USBD_BUSY);}
		}
		else if(ret == HAL_OK)
		{
			sprintf(i2c2check_active_address, "0x%X", i);
			if (PRINT_TOGGLE == 1) {
				while (CDC_Transmit_FS (i2c2check_active_address, strlen(i2c2check_active_address)) == USBD_BUSY);}
		}
	}

	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	//while (CDC_Transmit_FS ("\n", 1) == USBD_BUSY);
	//--[ Scanning Done ]--

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	// Have a timer to break the USB loop in case of power cycle
	tick = 600;

	/// Pre-main program /////////////////////////////////////////////////////////////////////////////////
	/// Pre-main program /////////////////////////////////////////////////////////////////////////////////
	/// Pre-main program /////////////////////////////////////////////////////////////////////////////////

	while (0) {

		// 60s grace timer
		if (tick == 0) {break;}

		// if something through USB is received (toggled by interrupt in CDC)
		if (usb_Rx_ready != 0){

			tick = 600;
			usb_Rx_ready = 0;

			if (usb_Rx_buffer[0] == USB_MIRA) {
				if (usb_Rx_buffer[1] == USB_CHECKSTATUS) {

				}
				else if (usb_Rx_buffer[1] == USB_TESTOUTPUT) {

				}

			}
			else if (usb_Rx_buffer[0] == USB_LORA) {
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

			else if (usb_Rx_buffer[0] == USB_GYRO) {
				if (usb_Rx_buffer[1] == USB_CHECKSTATUS) {

				}
				else if (usb_Rx_buffer[1] == USB_TESTOUTPUT) {

				}

			}

			else if (usb_Rx_buffer[0] == USB_BMP) {
				if (usb_Rx_buffer[1] == USB_CHECKSTATUS) {

				}
				else if (usb_Rx_buffer[1] == USB_TESTOUTPUT) {

				}

			}

			else if (usb_Rx_buffer[0] == USB_GPS) {
				if (usb_Rx_buffer[1] == USB_CHECKSTATUS) {

				}
				else if (usb_Rx_buffer[1] == USB_TESTOUTPUT) {

				}

			}

			else if (usb_Rx_buffer[0] == USB_SD) {
				if (usb_Rx_buffer[1] == USB_CHECKSTATUS) {

				}
				else if (usb_Rx_buffer[1] == USB_TESTOUTPUT) {

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
				HAL_Delay(5000);
				break;
			}

			else if (usb_Rx_buffer[1] == USB_PING) {

				while (CDC_Transmit_FS ("PONG", 4) == USBD_BUSY);
			}
		}
	}


	HAL_GPIO_TogglePin (LED1_GPIO_Port, LED1_Pin);
	// Reset timers before main program
	tick = 0;
	tickGPS = 0;
	tickSync = 0;
	tickMIRA = 0;

	/// Main program /////////////////////////////////////////////////////////////////////////////////
	/// Main program /////////////////////////////////////////////////////////////////////////////////
	/// Main program /////////////////////////////////////////////////////////////////////////////////
	while (1) {

		// Flush SD card every minute
		if (tickSync == 0) {
			tickSync = 600;
			sd_status = f_sync(&SDFile);
			// Comment this out once build finished
			if(sd_status != FR_OK)
			{
				if (PRINT_TOGGLE == 1) {
					while (CDC_Transmit_FS ("Sync failed!\n", 14) == USBD_BUSY);}
			}

		}


		// Read temperature, pressure and gyro data every second and send everything to SD and lora every second
		if (tick == 0) {
			// Start timer again
			tick = 10;

			/// TIMER /////////////////////////////////////////////////////////////////////////////////
			// Print current time
			sprintf(system_time_buffer, "t:%.0f\n", system_time_counter);

			//while (CDC_Transmit_FS (system_time_buffer, strlen(system_time_buffer)) == USBD_BUSY);
			add_to_buffer(&data_buffer, &system_time_buffer, strlen(system_time_buffer));

			system_time_counter++;

			// Toggle LED on board to indicate succesful timer management
			HAL_GPIO_TogglePin (LED2_GPIO_Port, LED2_Pin);


			/// BMP  /////////////////////////////////////////////////////////////////////////////////
			// bmp needed to be forced for this kind of data reading, as now FIFO buffers or dready interrupts are being used
			bmp_settings.op_mode = BMP3_MODE_FORCED;
			bmp_result = bmp3_set_op_mode(&bmp_settings, &bmp_device);
			bmp3_check_rslt("bmp3_set_op_mode", bmp_result);

			/*
			 * First parameter indicates the type of data to be read
			 * BMP3_PRESS_TEMP : To read pressure and temperature data
			 * BMP3_TEMP       : To read only temperature data
			 * BMP3_PRESS      : To read only pressure data
			 */

			// Check sensor measurements
			bmp_result = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &bmp_data, &bmp_device);
			bmp3_check_rslt("bmp3_get_sensor_data", bmp_result);

			// NOTE : Read status register again to clear data ready interrupt status
			bmp_result = bmp3_get_status(&bmp_status, &bmp_device);
			bmp3_check_rslt("bmp3_get_status", bmp_result);

			// Print bmp measurements
			sprintf(bmp_temperature_buffer, "%.2f\n", bmp_data.temperature);
			sprintf(bmp_pressure_buffer, "%.2f\n", bmp_data.pressure);

			add_to_buffer(&data_buffer, &bmp_temperature_buffer, strlen(bmp_temperature_buffer));
			add_to_buffer(&data_buffer, &bmp_pressure_buffer, strlen(bmp_pressure_buffer));

			//while (CDC_Transmit_FS (bmp_temperature_buffer, strlen(bmp_temperature_buffer)) == USBD_BUSY);
			//while (CDC_Transmit_FS (bmp_pressure_buffer, strlen(bmp_pressure_buffer)) == USBD_BUSY);


			/// Gyro /////////////////////////////////////////////////////////////////////////////////
			// Read gyro acceleration and angular velocity data
			gyro_result_acceleration = LSM6DSO_ACC_GetAxes (&gyro_device, &gyro_acceleration_object);
			gyro_result_angularvel = LSM6DSO_GYRO_GetAxes (&gyro_device, &gyro_angularvel_object);

			sprintf(gyro_acceleration_buffer, "%"PRId32",%"PRId32",%"PRId32"\n", gyro_acceleration_object.x, gyro_acceleration_object.y, gyro_acceleration_object.z);
			sprintf(gyro_angularvel_buffer, "%"PRId32",%"PRId32",%"PRId32"\n", gyro_angularvel_object.x, gyro_angularvel_object.y, gyro_angularvel_object.z);

			add_to_buffer(&data_buffer, &gyro_acceleration_buffer, strlen(gyro_acceleration_buffer));
			add_to_buffer(&data_buffer, &gyro_angularvel_buffer, strlen(gyro_angularvel_buffer));
			// Print gyro measurements
			//while (CDC_Transmit_FS (gyro_acceleration_buffer, strlen(gyro_acceleration_buffer)) == USBD_BUSY);
			//while (CDC_Transmit_FS (gyro_angularvel_buffer, strlen(gyro_angularvel_buffer)) == USBD_BUSY);


			/// GPS DATA /////////////////////////////////////////////////////////////////////////////////
			// Add all gathered GPS data to data buffer

			//			while (NULL != (gps_buffer_ptr = strchr(*gps_buffer_ptr, ','))) {
			//				add_to_buffer(&gps_parsed_buffer, gps_buffer_ptr, strlen(gps_buffer_ptr));
			//				*gps_buffer_ptr = '\0';
			//			      gps_buffer_ptr++;
			//			}
			//			for (parser_i = 1; parser_i < 10; parser_i++) {
			//				gps_buffer_ptr1 = strchr(*gps_buffer_ptr1, ',');
			//				*gps_buffer_ptr1 = '\0';
			//				gps_buffer_ptr1++;
			//				if ((parser_i == 1) || (parser_i == 2) || (parser_i == 4) || (parser_i == 9)) {
			//					add_to_buffer(&data_buffer, gps_buffer_ptr2, strlen(gps_buffer_ptr2));
			//					sprintf(data_buffer + strlen(data_buffer), "\n");
			//				}
			//				gps_buffer_ptr2 = gps_buffer_ptr1;
			//
			//			}


			//add_to_buffer(&data_buffer, &gps_buffer, strlen(gps_buffer));
			char *token;
			char *gps_buffer_copy = strdup(gps_buffer);

			parser_i = 0;
			token = strtok(strstr(gps_buffer_copy, "$GNGGA"), ",");
			while (token != NULL && parser_i < 10) {
				if ((parser_i == 1) || (parser_i == 2) || (parser_i == 4) || (parser_i == 9)) {
					add_to_buffer(&data_buffer, token, strlen(token));
					strcat(data_buffer, ",");
				}
				token = strtok(NULL, ",");
				parser_i++;
			}


			free(gps_buffer_copy); // Free the memory allocated for the copy
			free(token);

			gps_buffer[0] = '\0';


			/// MIRA /////////////////////////////////////////////////////////////////////////////////
			strcat(data_buffer, "\n");
			if (tickMIRA < 20 & MIRA_HK_check == 1) {
				MIRA_HK_check = 0;
				status = mira_housekeeping_data(&huart1, mira_housekeeping_Rx_buffer, sizeof(mira_housekeeping_Rx_buffer), 5000);
				add_to_buffer_hex(&data_buffer, &mira_housekeeping_Rx_buffer, sizeof(mira_housekeeping_Rx_buffer));
				strcat(data_buffer, "\n");
			}
			if (tickMIRA == 0) {
				tickMIRA = mira_integration_time;
				status = mira_science_data(&huart1, mira_science_Rx_buffer, 142, mira_Rx_spec_available, mira_Rx_buffer, write_bool, 5000);
//				if (write_bool == 0x01){
				add_to_buffer_hex(&data_buffer, &mira_science_Rx_buffer, sizeof(mira_science_Rx_buffer));
				strcat(data_buffer, "\n");
//				write_bool = 0x00;
//				}
				MIRA_HK_check = 1;
			}
			strcat(data_buffer, "\n");



			/// DATA RECORDING /////////////////////////////////////////////////////////////////////////////////
			//write gps data to SD
			//if (sd_status == FR_OK){(char *)
			if (sd_status == FR_OK) {
				sd_result_write = f_write(&SDFile, data_buffer, strlen(data_buffer), (void *)&sd_err_byteswritten);
			}

			if (uart_dump == 1) {
				HAL_UART_Transmit_IT(&huart2, &data_buffer, strlen(data_buffer));
			}
			//}
			// Sendgps data to LORA
			if (lora_res == LORA_OK) {
				lora_send_packet(&lora, data_buffer, strlen(data_buffer));
			}

			// Comment this out once build finished
			if (PRINT_TOGGLE == 1) {
				while (CDC_Transmit_FS (data_buffer, strlen(data_buffer)) == USBD_BUSY);}
			data_buffer[0] = '\0';


		}

		/// GPS /////////////////////////////////////////////////////////////////////////////////
		// Read GPS data whenever UART interrupt raises gps_data_ready flag
//		if (gps_data_ready) {
//			// Choose the buffer from the two data buffers that is not currently being written into and print it
//			if (gps_rxBuffer == gps_rxBuffer1) {
//				//while (CDC_Transmit_FS (gps_rxBuffer2, strlen(gps_rxBuffer2)) == USBD_BUSY);
//				add_to_buffer(&gps_buffer, &gps_rxBuffer2, strlen(gps_rxBuffer2));
//
//			}
//			else {
//				//while (CDC_Transmit_FS (gps_rxBuffer1, strlen(gps_rxBuffer1)) == USBD_BUSY);
//				add_to_buffer(&gps_buffer, &gps_rxBuffer1, strlen(gps_rxBuffer1));
//			}
//
//			// Toggle flags to allow for buffer swapping and next data batch sending
//			gps_data_ready ^= 1;
//			gps_send_ready |= 1;
//
//		}


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

	/* DMA interrupt init */
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
