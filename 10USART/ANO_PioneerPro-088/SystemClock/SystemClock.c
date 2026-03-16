#include "SystemClock.h"
#include "stm32f4xx.h"
#undef HSE_VALUE
#define HSE_VALUE ((uint32_t)8000000)
void SystemClock_Config(void) {
    // 1. 开启外部高速晶振 (HSE) 并等待就绪
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // 2. 开启电源接口时钟，配置调压器输出电压级别 1 模式（跑 168MHz 必需）
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    // 3. 配置 HCLK (AHB), PCLK1 (APB1), PCLK2 (APB2) 预分频器
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // HCLK = 168MHz
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // PCLK2 = 84MHz (Max 84)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // PCLK1 = 42MHz (Max 42)

    /** * 4. 配置主 PLL 参数以匹配 8MHz 晶振
     * 公式: F_sys = (HSE / PLL_M) * PLL_N / PLL_P
     * PLL_M = 8   -> 得到 1MHz 的基准频率
     * PLL_N = 336 -> 倍频到 336MHz
     * PLL_P = 2   -> 二分频得到 168MHz 主频
     * PLL_Q = 7   -> 得到 48MHz (用于 USB/SDIO)
     */
    RCC->PLLCFGR = 8 | (336 << 6) | (((2 >> 1) - 1) << 16) | (RCC_PLLCFGR_PLLSRC_HSE) | (7 << 24);

    // 5. 开启 PLL 并等待就绪
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // 6. 配置 Flash 等待周期 (168MHz 需要 5WS)
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

    // 7. 切换系统时钟源到 PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    
    // 8. 更新 SystemCoreClock 全局变量
    SystemCoreClockUpdate();
}