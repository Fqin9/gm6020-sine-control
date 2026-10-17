#pragma once
#include "can.h"
struct GPIO_TypeDef {};
extern GPIO_TypeDef gpio_a;
#define GPIOA (&gpio_a)
constexpr uint32_t GPIO_PIN_0 = 1, GPIO_MODE_INPUT = 0, GPIO_PULLUP = 1;
enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };
struct GPIO_InitTypeDef { uint32_t Pin, Mode, Pull; };
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
