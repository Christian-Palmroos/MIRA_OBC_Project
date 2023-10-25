/*
 * mira.c
 *
 *  Created on: Sep 12, 2023
 *      Author: shonyb
 */

// Command codes to MIRA
#define STATUS_REQUEST 0x00
#define READ_REGISTER 0x02
#define WRITE_REGISTER 0x03
#define GET_HOUSEKEEPING 0x30
#define	GET_SCIENCE_DATA 0x40
#define	GET_CALBRATION_DATA 0x42
#define	SET_TIME 0x50
#define GET_TIME 0x51

// MIRA responses
#define STATUS_PACKET 0x71
#define REGISTER_CONTENT 0x72
#define HK_PACKET 0x31
#define SCIENCE_DATA_PACKET 0x41
#define TIME_PACKET 0x52
#define	CALIBRATION_PACKET 0x82

void mira_read(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout){

	// read register at given address and set to given pointer
	HAL_UART_Receive(huart, pData, Size, Timeout);

	// return status

}

void mira_write(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout){

	// write given value to register at given address
	HAL_UART_Transmit(huart, pData, Size, Timeout);

	// return status

}

void mira_science_data(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout){

	// mira_read() to given pointer
	mira_read(huart, pData, Size, Timeout);

	// set mark_as_read
	mira_write(huart, pData, Size, Timeout);

	// return status

}

void mira_housekeeping_data(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout){

	// mira_read() to given pointer
	mira_read(huart, pData, Size, Timeout);

	// return status

}

void mira_init(){

	// Turn on MIRA power (GPIO)
	HAL_GPIO_TogglePin (MIRA_EN_PWR_GPIO_Port, MIRA_EN_PWR_Pin);

	// OBC queries MIRA time
	mira_read(huart, pData, Size, Timeout);

	// activate powersave
	mira_write(huart, pData, Size, Timeout);

	// OBC writes configuration data to MIRA registers
	mira_write(huart, pData, Size, Timeout);

	// OBC sets MIRA time
	mira_write(huart, pData, Size, Timeout);

	// return from powersave
	mira_write(huart, pData, Size, Timeout);

}

