/*
 * mira.c
 *
 *  Created on: Sep 12, 2023
 *      Author: shonyb
 */

#include <string.h>
#include "main.h"

// Command codes to MIRA
const uint8_t STATUS_REQUEST = 0x00;
const uint8_t READ_REGISTER = 0x02;
const uint8_t WRITE_REGISTER = 0x03;
const uint8_t GET_HOUSEKEEPING = 0x30;
const uint8_t GET_SCIENCE_DATA = 0x40;
const uint8_t GET_CALIBRATION_DATA = 0x42;
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
const uint8_t MARK_AS_READ[1] = {0x81};
const uint8_t POWERSAVE = 0xC0;

// MIRA init parameters
const uint8_t mira_write_IT[4] = {0x00,0x00,0x00, 0x0F}; // 15 s
volatile uint8_t write_bool = 0x00;
//MIRA communication status
//volatile unsigned mira_ready_for_comm = 1;


/*
 * Computes the CRC16 (Cyclic Redundancy Check) for a given data array.
 * @param *nData: pointer to the data array
 * @param wLength: length of the data array
 * @return 16-bit unsigned integer which is the computed CRC16 value.
 */
uint16_t CRC16 (uint8_t *nData, uint16_t wLength)
{
	static const uint16_t wCRCTable[] = {
			0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
			0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
			0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
			0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
			0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
			0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
			0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
			0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
			0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
			0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
			0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
			0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
			0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
			0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
			0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
			0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
			0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
			0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
			0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
			0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
			0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
			0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
			0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
			0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
			0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
			0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
			0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
			0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
			0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
			0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
			0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
			0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

	uint8_t nTemp;
	uint16_t wCRCWord = 0xFFFF;

	while (wLength--)
	{
		nTemp = *nData++ ^ wCRCWord;
		wCRCWord >>= 8;
		wCRCWord ^= wCRCTable[nTemp];
	}
	return wCRCWord;

}

/*
 * Sends a command with an empty payload to MIRA over UART and receives a response.
 * @param *huart: pointer to the UART handle structure
 * @param command: the command to be sent (integer)
 * @param reg: the register address.
 * @param *rxBuffer: pointer to the buffer that stores the received data
 * @param Timeout: timeout duration for UART communication in milliseconds
 * @return status: status of the UART operation
 */
HAL_StatusTypeDef mira_command_empty_payload(UART_HandleTypeDef *huart, uint8_t command, uint8_t *rxBuffer, uint8_t rx_size, uint32_t Timeout){

	//Wait that previous instance of communication is done (toggled by HAL_UART_RxCpltCallback)
	//while (!mira_ready_for_comm);//{HAL_Delay(100);}
//	HAL_Delay(500);
//	mira_ready_for_comm = 0;

	HAL_StatusTypeDef status;
	uint8_t message_len = 9;
	uint8_t message[message_len];
	int j;
	for (j = 0; j < message_len; j++) {
		message[j] = 0;
	}
	uint8_t sync[2] = {0x5a, 0xce};
	// do this (below) properly some other time
	uint8_t length[2] = {0x00, 0x00};
	uint8_t src[1] = {0xc1};
	uint8_t dest[1] = {0xe1};
	uint16_t sum = 0;

	message[0] = sync[0];
	message[1] = sync[1];
	message[2] = length[0];
	message[3] = length[1];
	message[4] = src[0];
	message[5] = dest[0];
	message[6] = command;

	sum = CRC16(message+2, 7);

	message[7] = (sum&0xFF00)>>8;
	message[8] = (sum&0x00FF);

	//while (huart->RxState != HAL_UART_STATE_READY) {HAL_Delay(1);}

	// Enable transmitter and disable receiver
	HAL_GPIO_WritePin(RX_EN_2_GPIO_Port, RX_EN_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TX_EN_2_GPIO_Port, TX_EN_2_Pin, GPIO_PIN_SET);

	// write given value to register at given address
	status = HAL_UART_Transmit(huart, message, 9, Timeout);

	// Enable receiver and disable transmitter
	HAL_GPIO_WritePin(RX_EN_2_GPIO_Port, RX_EN_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TX_EN_2_GPIO_Port, TX_EN_2_Pin, GPIO_PIN_RESET);
	if (status != HAL_OK) {return status;}

	status = HAL_UART_Receive_DMA(huart, rxBuffer, rx_size);
	HAL_Delay(3);


	return status;

}

HAL_StatusTypeDef mira_command_empty_payload_with_reg(UART_HandleTypeDef *huart, uint8_t command, uint8_t reg, uint8_t *rxBuffer, uint8_t rx_size, uint32_t Timeout){

	//Wait that previous instance of communication is done (toggled by HAL_UART_RxCpltCallback)
	//while (!mira_ready_for_comm);//{HAL_Delay(100);}
//	HAL_Delay(500);
//	mira_ready_for_comm = 0;

	HAL_StatusTypeDef status;
	uint8_t message_len = 9;
	uint8_t message[message_len];
	int j;
	for (j = 0; j < message_len; j++) {
		message[j] = 0;
	}
	uint8_t sync[2] = {0x5a, 0xce};
	// do this (below) properly some other time
	uint8_t length[2] = {0x00, 0x00};
	uint8_t src[1] = {0xc1};
	uint8_t dest[1] = {0xe1};
	uint16_t sum = 0;

	message[0] = sync[0];
	message[1] = sync[1];
	message[2] = length[0];
	message[3] = length[1];
	message[4] = src[0];
	message[5] = dest[0];
	message[6] = command;
	message[7] = reg;

	sum = CRC16(message+2, 8);

	message[8] = (sum&0xFF00)>>8;
	message[9] = (sum&0x00FF);

	//while (huart->RxState != HAL_UART_STATE_READY) {HAL_Delay(1);}

	// Enable transmitter and disable receiver
	HAL_GPIO_WritePin(RX_EN_2_GPIO_Port, RX_EN_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TX_EN_2_GPIO_Port, TX_EN_2_Pin, GPIO_PIN_SET);

	// write given value to register at given address
	status = HAL_UART_Transmit(huart, message, 10, Timeout);

	// Enable receiver and disable transmitter
	HAL_GPIO_WritePin(RX_EN_2_GPIO_Port, RX_EN_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TX_EN_2_GPIO_Port, TX_EN_2_Pin, GPIO_PIN_RESET);
	if (status != HAL_OK) {return status;}

	status = HAL_UART_Receive_DMA(huart, rxBuffer, rx_size);
	HAL_Delay(3);


	return status;

}

/*
 * Sends a command to MIRA over UART and receives a response.
 * @param *huart: pointer to the UART handle structure
 * @param command: the command to be sent (integer)
 * @param reg: the register address.
 * @param *data: pointer to the buffer that has the payload of the message
 * @param *rxBuffer: pointer to the buffer that stores the received data
 * @param Timeout: timeout duration for UART communication in milliseconds
 * @return status: status of the UART operation
 */
HAL_StatusTypeDef mira_command(UART_HandleTypeDef *huart, uint8_t command, uint8_t reg, uint8_t *data, uint8_t size_data, uint8_t *rxBuffer, uint32_t Timeout){

	//Wait that previous instance of communication is done (toggled by HAL_UART_RxCpltCallback)
	//while (!mira_ready_for_comm);//{HAL_Delay(100);}
//	HAL_Delay(500);
//	mira_ready_for_comm = 0;

	uint8_t payload_len = size_data;
	uint8_t message_len = payload_len + 10;
	HAL_StatusTypeDef status;
	//uint8_t message[9+length_val];
	uint8_t message[message_len];
	int j;
	for (j = 0; j < sizeof(message); j++) {
		message[j] = 0;
	}
	uint8_t sync[2] = {0x5a, 0xce};
	// do this (below) properly some other time
	uint8_t length[2] = {0x00, payload_len+1};
	uint8_t src[1] = {0xc1};
	uint8_t dest[1] = {0xe1};
	uint16_t sum = 0;

	message[sizeof(message) - message_len + 0] = sync[0];
	message[sizeof(message) - message_len + 1] = sync[1];
	message[sizeof(message) - message_len + 2] = length[0];
	message[sizeof(message) - message_len + 3] = length[1];
	message[sizeof(message) - message_len + 4] = src[0];
	message[sizeof(message) - message_len + 5] = dest[0];
	message[sizeof(message) - message_len + 6] = command;
	message[sizeof(message) - message_len + 7] = reg;

	int i;
	for (i = 0; i < payload_len; i++) {
		message[sizeof(message) - message_len + 8 + i] = data[i];
	}

	sum = CRC16(message + 2, message_len-2);

	message[sizeof(message)] = (sum&0xFF00)>>8;
	message[sizeof(message) - message_len + 10 + i-1] = (sum&0x00FF);

	//while (huart->RxState != HAL_UART_STATE_READY) {HAL_Delay(1);}

	// Enable transmitter and disable receiver
	HAL_GPIO_WritePin(RX_EN_2_GPIO_Port, RX_EN_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TX_EN_2_GPIO_Port, TX_EN_2_Pin, GPIO_PIN_SET);

	// write given value to register at given address
	status = HAL_UART_Transmit(huart, message, sizeof(message), Timeout);

	// Enable receiver and disable transmitter
	HAL_GPIO_WritePin(RX_EN_2_GPIO_Port, RX_EN_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TX_EN_2_GPIO_Port, TX_EN_2_Pin, GPIO_PIN_RESET);
	if (status != HAL_OK) {return status;}

	status = HAL_UART_Receive_DMA(huart, rxBuffer, (uint8_t)10);
	HAL_Delay(3);


	return status;

}

/*
 * Retrieves science data and marks data as read in MIRA register.
 * @param *huart: pointer to the UART handle structure
 * @param *science_Rx: pointer to the buffer that stores the science data
 * @param *response_Rx: pointer to the buffer that stores the response from MIRA
 * @param Timeout: timeout duration for UART communication in milliseconds
 * @return status: status of the UART operation
 */
HAL_StatusTypeDef mira_science_data(UART_HandleTypeDef *huart, uint8_t *science_Rx, uint8_t science_size, uint8_t *specRxBuffer, uint8_t *rxBuffer, uint8_t write_bool, uint32_t Timeout){

	HAL_StatusTypeDef status;

	//status = mira_command_empty_payload_with_reg(huart, READ_REGISTER, 0x05, specRxBuffer, (uint8_t)sizeof(specRxBuffer), Timeout);
	//if (status != HAL_OK) {return status;}
	//if (specRxBuffer[10] == 0x01) {
		 //Get the science data and save it to science_Rx
		status = mira_command_empty_payload(huart, GET_SCIENCE_DATA, science_Rx, science_size, Timeout);
		//write_bool = 0x01;
	//}


//	// Mark data as read
//	status = mira_command(huart, WRITE_REGISTER, CHECK_FOR_READ, MARK_AS_READ, sizeof(MARK_AS_READ), rxBuffer, Timeout);

	// return status
	return status;
}

HAL_StatusTypeDef mira_science_data_with_check(UART_HandleTypeDef *huart, uint8_t *science_Rx, uint8_t science_size, uint8_t *specRxBuffer, uint8_t *rxBuffer, uint8_t write_bool, uint32_t Timeout){

	HAL_StatusTypeDef status;

	status = mira_command_empty_payload_with_reg(huart, READ_REGISTER, 0x05, specRxBuffer, 13, Timeout);
	if (status != HAL_OK) {return status;}
	if (specRxBuffer[10] == 0x01) {
		 //Get the science data and save it to science_Rx
		status = mira_command_empty_payload(huart, GET_SCIENCE_DATA, science_Rx, science_size, Timeout);
		write_bool = 0x01;
	}


//	// Mark data as read
//	status = mira_command(huart, WRITE_REGISTER, CHECK_FOR_READ, MARK_AS_READ, sizeof(MARK_AS_READ), rxBuffer, Timeout);

	// return status
	return status;
}


/*
 * Retrieves housekeeping data from MIRA.
 * @param *huart: pointer to the UART handle structure
 * @param *rxBuffer: pointer to the buffer that stores the MIRA housekeeping data
 * @param Timeout: timeout duration for UART communication in milliseconds
 * @return status: status of the UART operation
 */
HAL_StatusTypeDef mira_housekeeping_data(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint8_t rx_size, uint32_t Timeout){

	HAL_StatusTypeDef status;

	// Call for housekeeping data  // 96 bytes of HK data
	status = mira_command_empty_payload(huart, GET_HOUSEKEEPING, rxBuffer, rx_size, Timeout);

	// return status
	return status;

}

/*
 * Initialization sequence for MIRA.
 * @param *huart: pointer to the UART handle structure
 * @param Timeout: timeout duration for UART communication in milliseconds
 * @return status: status of the UART operation
 */
HAL_StatusTypeDef mira_init(UART_HandleTypeDef *huart, uint32_t Timeout){

	HAL_StatusTypeDef status;

	uint8_t mira_Rx_buffer[10];

	// Set AD address
	uint8_t AD_addr = 0x03;
	uint8_t mira_write_AD_addr[4] = {0x00,0x00,0x00,0x02}; // set value 2
	status =  mira_command(huart, WRITE_REGISTER, AD_addr, mira_write_AD_addr, sizeof(mira_write_AD_addr), mira_Rx_buffer, Timeout);
	if (status != HAL_OK) {return status;}

	HAL_Delay(10);

	// Enable AD converter
	uint8_t AD_en = 0x02;
	uint8_t mira_write_AD_en[4] = {0x00,0x00,0x00,0x01};
	status =  mira_command(huart, WRITE_REGISTER, AD_en, mira_write_AD_en, sizeof(mira_write_AD_en), mira_Rx_buffer, Timeout);
	if (status != HAL_OK) {return status;}

	HAL_Delay(1000);

	// Enable high voltage
	uint8_t HV_enable = 0x14;
	uint8_t mira_write_HV_enable[4] = {0x00,0x00,0x00,0x01};
	status =  mira_command(huart, WRITE_REGISTER, HV_enable, mira_write_HV_enable, sizeof(mira_write_HV_enable), mira_Rx_buffer, Timeout);
	if (status != HAL_OK) {return status;}

	HAL_Delay(1000);

	// Set calibration values regs 7-10, 14, 15
	// Set integration time IT
	uint8_t IT = 0x07;
	// Instead of inputting the value here, the value is at the top of mira.c with the name mira_write_IT
	status =  mira_command(huart, WRITE_REGISTER, IT, mira_write_IT, sizeof(mira_write_IT), mira_Rx_buffer, Timeout);
	if (status != HAL_OK) {return status;}
	HAL_Delay(10);

	// Set main trigger level
	uint8_t Trigger = 0x0B;
	uint8_t mira_write_Trigger[4] = {0x00,0x00,0x00, 0x0A}; // set value 10
	status =  mira_command(huart, WRITE_REGISTER, Trigger, mira_write_Trigger, sizeof(mira_write_Trigger), mira_Rx_buffer, Timeout);
	if (status != HAL_OK) {return status;}
	HAL_Delay(10);

	// Set fast noise level
	uint8_t Fast_noise = 0x08;
	uint8_t mira_write_Fast_noise[4] = {0x00, 0x00,0x00,0x0C}; // set value 12
	status =  mira_command(huart, WRITE_REGISTER, Fast_noise, mira_write_Fast_noise, sizeof(mira_write_Fast_noise), mira_Rx_buffer, Timeout);
	if (status != HAL_OK) {return status;}
	HAL_Delay(10);

	// Set fast trigger level
	uint8_t Fast_trigger = 0x09;
	uint8_t mira_write_Fast_trigger[4] = {0x00, 0x00,0x00,0x0A}; // set value 10
	status =  mira_command(huart, WRITE_REGISTER, Fast_trigger, mira_write_Fast_trigger, sizeof(mira_write_Fast_trigger), mira_Rx_buffer, Timeout);
	if (status != HAL_OK) {return status;}
	HAL_Delay(10);

	// Set filter settings to 0
	uint8_t Filter_settings = 0x0A;
	uint8_t mira_write_Filter_settings[4] = {0x00,0x00,0x00,0x00};
	status =  mira_command(huart, WRITE_REGISTER, Filter_settings, mira_write_Filter_settings, sizeof(mira_write_Filter_settings), mira_Rx_buffer, Timeout);
	if (status != HAL_OK) {return status;}
	HAL_Delay(10);

	// Set fast calibration multiplier
	uint8_t Calib_m = 0x0E;
	uint8_t mira_write_Calib_m[4] = {0x00,0x00,0x47, 0x10}; // set value XXXXXX
	status =  mira_command(huart, WRITE_REGISTER, Calib_m, mira_write_Calib_m, sizeof(mira_write_Calib_m), mira_Rx_buffer, Timeout);
	if (status != HAL_OK) {return status;}
	HAL_Delay(10);

	// Set time in unix time (s)
	uint8_t Time = 0x0F;
	uint8_t mira_write_Time[4];
	int i;
	for (i = 0; i < 4; i++) {
		mira_write_Time[i] = 0;
	}
	status =  mira_command(huart, WRITE_REGISTER, Time, mira_write_Time, sizeof(mira_write_Time), mira_Rx_buffer, Timeout);
	if (status != HAL_OK) {return status;}
	HAL_Delay(10);

	// Go to science mode
	uint8_t Science_mode = 0x06;
	uint8_t mira_write_Science_mode[4] = {0x00,0x00,0x00,0x01};
	status =  mira_command(huart, WRITE_REGISTER, Science_mode, mira_write_Science_mode, sizeof(mira_write_Science_mode), mira_Rx_buffer, Timeout);
	if (status != HAL_OK) {return status;}


	return status;

}
