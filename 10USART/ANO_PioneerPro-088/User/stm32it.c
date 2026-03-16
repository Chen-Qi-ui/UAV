#include "FreeRTOS.h"
#include "stm32f4xx.h"  
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "USART.H"
#include "mavlink_types.h"
#include "portmacro.h"
#include "projdefs.h"
#include "Task_Create.h"
#include "Task.h"
#include "semphr.h"
extern TaskHandle_t xRxTaskHandle;
extern SemaphoreHandle_t xDmaTxSem;
// stm32it.c 的最上面
uint8_t os_is_running = 0; // 新增：RTOS 运行状态护盾

// 拦截 USART3 中断
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3,USART_IT_IDLE)==SET)
    {
        volatile uint32_t temp;
        temp = USART3->SR;
        temp = USART3->DR;
        (void)temp;
        
        // 【核心防御】：只有 os_is_running = 1 时，才允许操作 FreeRTOS！
        if(xRxTaskHandle != NULL && os_is_running == 1) 
        {
            BaseType_t highTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(xRxTaskHandle, &highTaskWoken);
            portYIELD_FROM_ISR(highTaskWoken);
        }
    }
}

// 同样拦截 DMA 中断
void DMA1_Stream4_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_TCIF4))
    {
        DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF4);
        
        // 【核心防御】
        if(xDmaTxSem != NULL && os_is_running == 1) 
        {
            BaseType_t highTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(xDmaTxSem, &highTaskWoken);
            portYIELD_FROM_ISR(highTaskWoken);
        }
    }
}



