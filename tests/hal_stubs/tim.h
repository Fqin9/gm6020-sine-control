#pragma once
#include "can.h"
struct TIM_HandleTypeDef {};
extern TIM_HandleTypeDef htim6;
constexpr uint32_t TIM_FLAG_UPDATE = 1;
inline void __HAL_TIM_SET_COUNTER(TIM_HandleTypeDef *, uint32_t) {}
inline void __HAL_TIM_CLEAR_FLAG(TIM_HandleTypeDef *, uint32_t) {}
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *);
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);
