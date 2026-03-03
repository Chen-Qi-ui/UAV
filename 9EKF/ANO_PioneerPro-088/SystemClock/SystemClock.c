#include "stm32f4xx.h"
#include "SystemClock.h"

void SystemClock_Config(void)
{
    // 1) 使能电源控制时钟，并设置电压缩放（Scale 1）以支持高主频
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS; // Scale 1 mode

    // 2) 使能HSI并等待稳定
    RCC->CR |= RCC_CR_HSION;
    while((RCC->CR & RCC_CR_HSIRDY) == 0);

    // 3) 配置Flash等待周期与缓存（168MHz需要5个等待周期）
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

    // 4) 配置总线分频：AHB=SYSCLK/1，APB1=SYSCLK/4(42MHz)，APB2=SYSCLK/2(84MHz)
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    // 5) 配置主PLL：源HSI(16MHz)
    //    VCO输入 = 16MHz / PLLM = 1MHz
    //    VCO输出 = 1MHz * PLLN = 336MHz
    //    SYSCLK  = VCO / PLLP = 336 / 2 = 168MHz
    //    USB48   = VCO / PLLQ = 336 / 7 = 48MHz
    {
        uint32_t cfgr = RCC->PLLCFGR;
        // 清除 PLLM(5:0), PLLN(14:6), PLLP(17:16), PLLSRC(22), PLLQ(27:24)
        cfgr &= ~((0x3FU << 0) | (0x1FFU << 6) | (0x3U << 16) | (1U << 22) | (0xFU << 24));
        // 设定 PLLSRC=HSI(0), PLLM=16, PLLN=336, PLLP=2(编码00b), PLLQ=7 -> 48MHz
        cfgr |= (0U << 22)         // PLLSRC = HSI
              | (16U << 0)         // PLLM = 16
              | (336U << 6)        // PLLN = 336
              | (0U << 16)         // PLLP = /2 (00b)
              | (7U << 24);        // PLLQ = 7 (48MHz)
        RCC->PLLCFGR = cfgr;
    }

    // 6) 使能PLL并等待锁定
    RCC->CR |= RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == 0);

    // 7) 切换系统时钟到PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // 8) 更新SystemCoreClock变量为168MHz
    SystemCoreClock = 168000000U;
}
