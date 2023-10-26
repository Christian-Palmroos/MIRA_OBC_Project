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

// MIRA configurations
#define CHECK_FOR_READ 0x80
#define MARK_AS_READ 0x81
#define POWERSAVE 0xC0

HAL_StatusTypeDef mira_read(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t rxSize, uint32_t Timeout){

	HAL_StatusTypeDef STATUS;

	// read register at given address and set to given pointer
	STATUS = HAL_UART_Receive(huart, rxBuffer, rxSize, Timeout);

	// return status
	return STATUS;

}

HAL_StatusTypeDef mira_write(UART_HandleTypeDef *huart, const uint8_t *commandHex, uint32_t Timeout){

	HAL_StatusTypeDef STATUS;

	// write given value to register at given address
	STATUS = HAL_UART_Transmit(huart, commandHex, sizeof(commandHex), Timeout);

	// return status
	return STATUS;

}

HAL_StatusTypeDef mira_science_data(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t rxSize, uint32_t Timeout){

	HAL_StatusTypeDef STATUS;

	// ask to read data
	STATUS = mira_write(huart, GET_SCIENCE_DATA, sizeof(GET_SCIENCE_DATA), Timeout);
	// receive asked data
	if (STATUS) {
		STATUS = mira_read(huart, rxBuffer, rxSize, Timeout);
	}
	else {
		return STATUS;
	}

	// set mark_as_read
	STATUS = mira_write(huart, CHECK_FOR_READ, sizeof(CHECK_FOR_READ), Timeout);
	STATUS = mira_write(huart, MARK_AS_READ, sizeof(MARK_AS_READ), Timeout);

	// return status
	return STATUS;
}

HAL_StatusTypeDef mira_housekeeping_data(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t rxSize, uint32_t Timeout){

	HAL_StatusTypeDef STATUS;

	// ask to read data
	STATUS = mira_write(huart, GET_HOUSEKEEPING, sizeof(GET_HOUSEKEEPING), Timeout);
	// receive asked data
	if (STATUS) {
		STATUS = mira_read(huart, rxBuffer, rxSize, Timeout);
	}
	else {
		return STATUS;
	}

	// return status
	return STATUS;

}

HAL_StatusTypeDef activate_powersave(UART_HandleTypeDef *huart, uint32_t Timeout){

	HAL_StatusTypeDef STATUS;

	STATUS = mira_write(huart, POWERSAVE, sizeof(POWERSAVE), Timeout);
	if (STATUS == HAL_OK) STATUS = mira_write(huart, 0x01, sizeof(0x01), Timeout);

	if (STATUS == HAL_OK) STATUS = mira_write(huart, POWERSAVE, sizeof(POWERSAVE), Timeout);
	if (STATUS == HAL_OK) STATUS = mira_write(huart, 0x03, sizeof(0x03), Timeout);

	if (STATUS == HAL_OK) STATUS = mira_write(huart, POWERSAVE, sizeof(POWERSAVE), Timeout);
	if (STATUS == HAL_OK) STATUS = mira_write(huart, 0x07, sizeof(0x07), Timeout);

	if (STATUS == HAL_OK) STATUS = mira_write(huart, POWERSAVE, sizeof(POWERSAVE), Timeout);
	if (STATUS == HAL_OK) STATUS = mira_write(huart, 0x0F, sizeof(0x0F), Timeout);

	return STATUS;
}

HAL_StatusTypeDef deactivate_powersave(UART_HandleTypeDef *huart, uint32_t Timeout){

	HAL_StatusTypeDef STATUS;

	STATUS = mira_write(huart, POWERSAVE, sizeof(POWERSAVE), Timeout);
	if (STATUS == HAL_OK) STATUS = mira_write(huart, 0x0F, sizeof(0x0F), Timeout);

	if (STATUS == HAL_OK) STATUS = mira_write(huart, POWERSAVE, sizeof(POWERSAVE), Timeout);
	if (STATUS == HAL_OK) STATUS = mira_write(huart, 0x07, sizeof(0x07), Timeout);

	if (STATUS == HAL_OK) STATUS = mira_write(huart, POWERSAVE, sizeof(POWERSAVE), Timeout);
	if (STATUS == HAL_OK) STATUS = mira_write(huart, 0x03, sizeof(0x03), Timeout);

	if (STATUS == HAL_OK) STATUS = mira_write(huart, POWERSAVE, sizeof(POWERSAVE), Timeout);
	if (STATUS == HAL_OK) STATUS = mira_write(huart, 0x01, sizeof(0x01), Timeout);

	return STATUS;
}

HAL_StatusTypeDef mira_init(UART_HandleTypeDef *huart){

	HAL_StatusTypeDef STATUS;
	int size_time = 30;
	uint8_t time[size_time];
	uint32_t Timeout = 2000;

	// Turn on MIRA power (GPIO)
	HAL_GPIO_TogglePin (MIRA_EN_PWR_GPIO_Port, MIRA_EN_PWR_Pin);

	// OBC queries MIRA time
	STATUS = mira_write(huart, GET_TIME, Timeout);
	if (STATUS != HAL_OK) return STATUS;
	STATUS = mira_read(huart, time, size_time, Timeout);
	if (STATUS != HAL_OK) return STATUS;

	// activate powersave
	STATUS = activate_powersave(huart, Timeout);
	if (STATUS != HAL_OK) return STATUS;

	// OBC writes configuration data to MIRA registers
	//STATUS = mira_write(huart, ADDRESS, Timeout);
	//if (STATUS != HAL_OK) return STATUS;

	// OBC sets MIRA time
	STATUS = mira_write(huart, SET_TIME, Timeout);
	if (STATUS != HAL_OK) return STATUS;

	// return from powersave
	STATUS = deactivate_powersave(huart, Timeout);

	return STATUS;

}

