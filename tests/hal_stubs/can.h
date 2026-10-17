#pragma once
#include <cstdint>

// 仅供电脑上的逻辑测试使用；固件工程不会包含这个目录。
struct CAN_TypeDef { uint32_t ESR = 0; };
struct CAN_HandleTypeDef { CAN_TypeDef *Instance; };
struct CAN_FilterTypeDef {
    uint32_t FilterBank, FilterMode, FilterScale, FilterIdHigh, FilterIdLow;
    uint32_t FilterMaskIdHigh, FilterMaskIdLow, FilterFIFOAssignment;
    uint32_t FilterActivation, SlaveStartFilterBank;
};
struct CAN_RxHeaderTypeDef { uint32_t StdId, IDE, RTR, DLC; };
struct CAN_TxHeaderTypeDef { uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime; };
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR };
constexpr uint32_t CAN_RX_FIFO0 = 0, CAN_RX_FIFO1 = 1;
constexpr uint32_t CAN_FILTER_FIFO0 = 0, CAN_FILTER_FIFO1 = 1;
constexpr uint32_t CAN_IT_RX_FIFO0_MSG_PENDING = 2, CAN_IT_RX_FIFO1_MSG_PENDING = 16;
constexpr uint32_t CAN_ID_STD = 0, CAN_RTR_DATA = 0;
constexpr uint32_t CAN_FILTERMODE_IDMASK = 0, CAN_FILTERSCALE_32BIT = 1, ENABLE = 1;
extern CAN_HandleTypeDef hcan1, hcan2;
uint32_t HAL_GetTick();
uint32_t __get_PRIMASK();
void __disable_irq();
void __set_PRIMASK(uint32_t);
void Error_Handler();
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *, CAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t, CAN_RxHeaderTypeDef *, uint8_t *);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *, const CAN_TxHeaderTypeDef *, const uint8_t *, uint32_t *);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *, uint32_t);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *);
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *, uint32_t);
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *);
