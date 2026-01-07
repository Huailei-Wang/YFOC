#pragma once


#include <stdint.h>



void BSP_DWT_Init();

float BSP_DWT_GetDeltaT(uint32_t *last_tick);

double BSP_DWT_GetDeltaT64(uint32_t *last_tick);

float BSP_DWT_GetTime_second();

float BSP_DWT_GetTime_ms();

float BSP_DWT_GetTime_us();