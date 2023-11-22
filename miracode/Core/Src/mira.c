/*
 * mira.c
 *
 *  Created on: Sep 12, 2023
 *      Author: shonyb
 */

#include <string.h>

// Command codes to MIRA
const uint8_t STATUS_REQUEST = 0x00;
const uint8_t READ_REGISTER = 0x02;
const uint8_t WRITE_REGISTER = 0x03;
const uint8_t GET_HOUSEKEEPING = 0x30;
const uint8_t GET_SCIENCE_DATA = 0x40;
const uint8_t GET_CALBRATION_DATA = 0x42;
const uint8_t SET_TIME = 0x50;
const uint8_t GET_TIME = 0x51;

// MIRA responses
const uint8_t STATUS_PACKET = 0x71;
const uint8_t REGISTER_CONTENT = 0x72;
const uint8_t HK_PACKET = 0x31;
const uint8_t SCIENCE_DATA_PACKET = 0x41;
const uint8_t TIME_PACKET = 0x52;
const uint8_t CALIBRATION_PACKET = 0x82;

// MIRA configurations
const uint8_t CHECK_FOR_READ = 0x80;
const uint8_t MARK_AS_READ = 0x81;
const uint8_t POWERSAVE = 0xC0;

//empty payload
const uint8_t EMPTY_PAYLOAD[1] = {0x99}; //Check that this is not used


uint8_t calc_checksum() {

	uint8_t sum[2] = {0x00, 0x00};

	return sum; //check how to return 2 bytes
}

// real checksum probably just return 00 ^
unsigned int checksum(char *str) {
   unsigned int sum = 0;
   while (*str) {
      sum += *str;
      str++;
   }
   return sum;
}

uint8_t build_message(uint8_t *command, uint8_t *payload) {

	// Command code 1 bytes
	// Payload 256 bytes

	int msg_size = 9 + sizeof(payload);
	uint8_t message[msg_size];
	uint8_t *sync[2] = {0x5a, 0xce};
	uint8_t *length[2] = {0x00, 0x05};
	uint8_t *src[1] = {0xc1};
	uint8_t *dest[1] = {0xe1};
	uint8_t *checksum[2] = calc_checksum();

	strcat(message, sync);
	strcat(message, length);
	strcat(message, src);
	strcat(message, dest);
	strcat(message, command);
	if (payload[0] != 0x99){
		strcat(message, payload);
	}
	strcat(message, checksum);

	return message;
}


HAL_StatusTypeDef mira_read(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t rxSize, uint32_t Timeout){

	HAL_StatusTypeDef STATUS;

	// read register at given address and set to given pointer
	STATUS = HAL_UART_Receive(huart, rxBuffer, rxSize, Timeout);

	// return status
	return STATUS;

}


HAL_StatusTypeDef mira_write(UART_HandleTypeDef *huart, uint8_t *message, uint32_t Timeout){

	HAL_StatusTypeDef STATUS;

	// write given value to register at given address
	STATUS = HAL_UART_Transmit(huart, message, sizeof(message), Timeout);

	// return status
	return STATUS;

}

HAL_StatusTypeDef mira_test_rw(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t rxSize){

	uint8_t message;

	message = build_message(WRITE_REGISTER, 0x02);
	//print()
	STATUS = mira_write(*huart, *message, 5000);
	if (STATUS != HAL_OK) return STATUS;
	STATUS = mira_read(*huart, *rxBuffer, rxSize, 5000);
	//print()
	if (STATUS != HAL_OK) return STATUS;

	return STATUS;

}


HAL_StatusTypeDef mira_science_data(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t rxSize, uint32_t Timeout){

	HAL_StatusTypeDef STATUS;
	uint8_t message;

	message = build_message(GET_SCIENCE_DATA, EMPTY_PAYLOAD);

	// ask to read data

	STATUS = mira_write(huart, message, Timeout);
	if (STATUS != HAL_OK) return STATUS;
	// receive asked data

	STATUS = mira_read(huart, rxBuffer, rxSize, Timeout);
	if (STATUS != HAL_OK) return STATUS;

	// set mark_as_read
	message = build_message(CHECK_FOR_READ, EMPTY_PAYLOAD);
	STATUS = mira_write(huart, message, Timeout);
	if (STATUS != HAL_OK) return STATUS;
	STATUS = mira_read(huart, rxBuffer, rxSize, Timeout);
	if (STATUS != HAL_OK) return STATUS;


	message = build_message(MARK_AS_READ, EMPTY_PAYLOAD);
	STATUS = mira_write(huart, message, Timeout);
	if (STATUS != HAL_OK) return STATUS;
	STATUS = mira_read(huart, rxBuffer, rxSize, Timeout);

	// return status
	return STATUS;
}


HAL_StatusTypeDef mira_housekeeping_data(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t rxSize, uint32_t Timeout){

	HAL_StatusTypeDef STATUS;
	uint8_t message;
	message = build_message(GET_HOUSEKEEPING, EMPTY_PAYLOAD);

	// ask to read data
	STATUS = mira_write(huart, message, Timeout);
	if (STATUS != HAL_OK) return STATUS;
	// receive asked data
	STATUS = mira_read(huart, rxBuffer, rxSize, Timeout);

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
	uint8_t message;

	// Turn on MIRA power (GPIO)
	HAL_GPIO_TogglePin (MIRA_EN_PWR_GPIO_Port, MIRA_EN_PWR_Pin);

	// OBC queries MIRA time
	message = build_message(GET_TIME, EMPTY_PAYLOAD);
	STATUS = mira_write(huart, message, Timeout);
	if (STATUS != HAL_OK) return STATUS;
	STATUS = mira_read(huart, time, size_time, Timeout);
	if (STATUS != HAL_OK) return STATUS;

	// activate powersave
	STATUS = activate_powersave(huart, Timeout);
	if (STATUS != HAL_OK) return STATUS;

	// OBC writes configuration data to MIRA registers
	STATUS = mira_write(huart, ADDRESS, Timeout);
	if (STATUS != HAL_OK) return STATUS;

	// OBC sets MIRA time
	message = build_message(SET_TIME, );
	STATUS = mira_write(huart, message, Timeout);
	if (STATUS != HAL_OK) return STATUS;

	// return from powersave
	STATUS = deactivate_powersave(huart, Timeout);

	return STATUS;

}

