/*
 * mira.h
 *
 *  Created on: Sep 12, 2023
 *      Author: shonyb
 */

#ifndef INC_MIRA_H_
#define INC_MIRA_H_

HAL_StatusTypeDef mira_write_register(UART_HandleTypeDef *huart, uint8_t *reg, uint32_t *data, uint32_t Timeout);
int build_message(uint8_t *message, uint8_t *command, uint8_t *payload);
HAL_StatusTypeDef mira_write(UART_HandleTypeDef *huart, uint8_t *message, uint32_t Timeout);
HAL_StatusTypeDef mira_read(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t rxSize, uint32_t Timeout);
HAL_StatusTypeDef mira_science_data(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t rxSize, uint32_t Timeout);
HAL_StatusTypeDef mira_housekeeping_data(UART_HandleTypeDef *huart, uint8_t *rxBuffer, uint16_t rxSize, uint32_t Timeout);
HAL_StatusTypeDef mira_init(UART_HandleTypeDef *huart);

#endif /* INC_MIRA_H_ */
