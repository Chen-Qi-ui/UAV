// USB_Clock.c - 安全版（不修改系统时钟，只启用 USB OTG FS）
// 要求：SystemClock_Config 已配置主PLL，且 PLLQ 提供 48MHz 给 USB

#include "stm32f4xx.h"

void USB_EnablePLL48CLK(void) {
    // 对有 CK48MSEL 的芯片（如 F446），选择 48MHz 源为主PLLQ（一般默认即可）
    #ifdef RCC_DCKCFGR
    RCC->DCKCFGR |= RCC_DCKCFGR_CK48MSEL;
    #endif

    // 使能 USB OTG FS 外设时钟
    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
}