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

HAL_StatusTypeDef mira_read(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t Size, uint32_t Timeout){

	HAL_StatusTypeDef STATUS;

	// read register at given address and set to given pointer
	STATUS = HAL_UART_Receive(huart, rxBuffer, Size, Timeout);

	// return status
	return STATUS;

}

HAL_StatusTypeDef mira_write(UART_HandleTypeDef *huart, const uint8_t *commandHex, uint16_t Size, uint32_t Timeout){

	HAL_StatusTypeDef STATUS;

	// write given value to register at given address
	STATUS = HAL_UART_Transmit(huart, commandHex, Size, Timeout);

	// return status
	return STATUS;

}

HAL_StatusTypeDef mira_science_data(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout){

	HAL_StatusTypeDef STATUS;

	// ask to read data
	STATUS = mira_write(huart, GET_SCIENCE_DATA, len(GET_SCIENCE_DATA), Timeout);
	// receive asked data
	if (STATUS) {
		STATUS = mira_read(huart, pData, Size, Timeout);
	}
	else {
		return STATUS;
	}

	// set mark_as_read
	STATUS = mira_write(huart, 0x80:payload:0x81, Size, Timeout);

	// return status
	return STATUS;
}

HAL_StatusTypeDef mira_housekeeping_data(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout){

	HAL_StatusTypeDef STATUS;

	// ask to read data
	STATUS = mira_write(huart, GET_HOUSEKEEPING, len(GET_HOUSEKEEPING), Timeout);
	// receive asked data
	if (STATUS) {
		STATUS = mira_read(huart, pData, Size, Timeout);
	}
	else {
		return STATUS;
	}

	// return status
	return STATUS;

}

HAL_StatusTypeDef mira_init(){

	HAL_StatusTypeDef STATUS;
	uint8_t time;

	// Turn on MIRA power (GPIO)
	HAL_GPIO_TogglePin (MIRA_EN_PWR_GPIO_Port, MIRA_EN_PWR_Pin);

	// OBC queries MIRA time
	STATUS = mira_write(huart, GET_TIME, len(GET_TIME), Timeout);
	if (STATUS != HAL_OK) return STATUS;
	STATUS = mira_read(huart, time, len(time), Timeout);
	if (STATUS != HAL_OK) return STATUS;

	// activate powersave
	STATUS = mira_write(huart, ADDRESS, Size, Timeout);
	if (STATUS != HAL_OK) return STATUS;

	// OBC writes configuration data to MIRA registers
	STATUS = mira_write(huart, ADDRESS, Size, Timeout);
	if (STATUS != HAL_OK) return STATUS;

	// OBC sets MIRA time
	STATUS = mira_write(huart, ADDRESS, Size, Timeout);
	if (STATUS != HAL_OK) return STATUS;

	// return from powersave
	STATUS = mira_write(huart, ADDRESS, Size, Timeout);
	if (STATUS != HAL_OK) return STATUS;

	return STATUS;

}

