/*
 * mira.h
 *
 *  Created on: Sep 12, 2023
 *      Author: shonyb
 */

#ifndef INC_MIRA_H_
#define INC_MIRA_H_

// Command codes to MIRA
extern const uint8_t STATUS_REQUEST;
extern const uint8_t READ_REGISTER;
extern const uint8_t WRITE_REGISTER;
extern const uint8_t GET_HOUSEKEEPING;
extern const uint8_t GET_SCIENCE_DATA;
extern const uint8_t GET_CALBRATION_DATA;
extern const uint8_t SET_TIME;
extern const uint8_t GET_TIME;

// MIRA responses
extern const uint8_t STATUS_PACKET;
extern const uint8_t REGISTER_CONTENT;
extern const uint8_t HK_PACKET;
extern const uint8_t SCIENCE_DATA_PACKET;
extern const uint8_t TIME_PACKET;
extern const uint8_t CALIBRATION_PACKET;

// MIRA configurations
extern const uint8_t CHECK_FOR_READ;
extern const uint8_t MARK_AS_READ;
extern const uint8_t POWERSAVE;

// MIRA init parameters

extern const uint8_t mira_write_IT[6];
extern volatile uint8_t write_bool;

//MIRA communication status
//extern volatile unsigned mira_ready_for_comm;

HAL_StatusTypeDef mira_science_data(UART_HandleTypeDef *huart, uint8_t *science_Rx, uint8_t science_size, uint8_t *specRxBuffer, uint8_t *rxBuffer, uint8_t write_bool, uint32_t Timeout);
HAL_StatusTypeDef mira_science_data_with_check(UART_HandleTypeDef *huart, uint8_t *science_Rx, uint8_t science_size, uint8_t *specRxBuffer, uint8_t *rxBuffer, uint8_t write_bool, uint32_t Timeout);
HAL_StatusTypeDef mira_housekeeping_data(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t rxSize, uint32_t Timeout);
HAL_StatusTypeDef mira_test_sequence(UART_HandleTypeDef *huart, uint8_t *science_Rx, uint8_t *response_Rx, uint32_t Timeout);
HAL_StatusTypeDef mira_init(UART_HandleTypeDef *huart, uint32_t Timeout);

#endif /* INC_MIRA_H_ */
