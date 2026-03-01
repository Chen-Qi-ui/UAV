#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "../SystemClock/SystemClock.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
uint32_t count = 0;
uint32_t tick_count = 0;
void vLEDTask(void *pvParameters)
{
    
    
    while(1)
    {
        // 记录当前tick
        tick_count = xTaskGetTickCount();
//		GPIO_ToggleBits(GPIOE,GPIO_Pin_1);
        
        // 延时500毫秒
        vTaskDelay(pdMS_TO_TICKS(500));
        


    }
}

/* LED初始化函数 */
void LED_Init(void)
{
    // 使能 GPIOF 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // 输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   // 无上下拉
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    // 初始化时熄灭所有LED
    GPIO_ResetBits(GPIOE, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2);
}

