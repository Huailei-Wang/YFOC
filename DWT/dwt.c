#include "dwt.h"
#include "main.h"

static uint64_t tick64;
static uint32_t last_tick;



void BSP_DWT_Init() {                                   //stm32f1用不了
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     // 使能DWT外设
    DWT->CYCCNT = 0;                                    // DWT CYCCNT寄存器计数清0
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                // 使能Cortex-M DWT CYCCNT寄存器
}
static void DWT_TICK64_Update() {
    const uint32_t cnt_now = DWT->CYCCNT;
    tick64 += cnt_now - last_tick;
    last_tick = cnt_now;
}
float BSP_DWT_GetDeltaT(uint32_t *last_tick) {
    DWT_TICK64_Update();
    if (*last_tick == 0) {
        *last_tick = DWT->CYCCNT;
        return 0.001f;
    }
    const uint32_t cnt_now = DWT->CYCCNT;
    const float dt = (float) (cnt_now - *last_tick) / (float) SystemCoreClock;
    *last_tick = cnt_now;
    return dt;
}
double BSP_DWT_GetDeltaT64(uint32_t *last_tick) {
    DWT_TICK64_Update();
    if (*last_tick == 0) {
        *last_tick = DWT->CYCCNT;
        return 0.001;
    }
    const uint32_t cnt_now = DWT->CYCCNT;
    const double dt = (cnt_now - *last_tick) / (double) SystemCoreClock;
    *last_tick = cnt_now;
    return dt;
}
float BSP_DWT_GetTime_second() {
    DWT_TICK64_Update();
    const float result = (float) tick64 / (float) SystemCoreClock;
    return result;
}
float BSP_DWT_GetTime_ms() {
    DWT_TICK64_Update();
    const float result = (float) tick64 / (float) SystemCoreClock * 1e3f;
    return result;
}
float BSP_DWT_GetTime_us() {
    DWT_TICK64_Update();
    const float result = (float) tick64 / (float) SystemCoreClock * 1e6f;
    return result;
}

