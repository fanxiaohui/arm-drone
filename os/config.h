#pragma once

#if defined(STM32F030x6)
#include <stm32f0xx.h>
#elif defined(STM32F103xB)
#include <stm32f1xx.h>
#else
#error "CPU model is not defined"
#endif
