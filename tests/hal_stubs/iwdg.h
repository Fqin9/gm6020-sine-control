#pragma once
#include "can.h"
struct IWDG_HandleTypeDef {};
extern IWDG_HandleTypeDef hiwdg;
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *);
