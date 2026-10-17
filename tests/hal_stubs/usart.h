#pragma once
#include "can.h"
struct UART_HandleTypeDef {};
extern UART_HandleTypeDef huart6;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
