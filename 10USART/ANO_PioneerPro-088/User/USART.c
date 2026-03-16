#include "stm32f4xx.h"  
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "USART.H"
#include "mavlink_types.h"
// 使用 __attribute__ 强制分配到普通数据区，避开 CCM
uint8_t g_tx_buffer[MAVLINK_MAX_PACKET_LEN] __attribute__((section(".data")));
uint8_t g_rx_buffer[MAV_RX_BUF_SIZE] __attribute__((section(".data")));


void USART3_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_DMA1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); // 配置 TX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); // 配置 RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3,&USART_InitStructure);
	//DMA TX
// ------------------- DMA TX 配置 -------------------
    DMA_DeInit(DMA1_Stream4);
    DMA_InitStructure.DMA_Channel = DMA_Channel_7;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)g_tx_buffer; 
    DMA_InitStructure.DMA_BufferSize = 0;                            // 发送任务中会重新赋值
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 明确：外设地址不自增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;          // 明确：内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 必须是Byte (8位)
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 必须是Byte (8位)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                    // TX使用普通模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;               
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           // 关闭FIFO，防止配置错误
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream4, &DMA_InitStructure);

//    // ------------------- DMA RX 配置 -------------------
    DMA_DeInit(DMA1_Stream1);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)g_rx_buffer;
    DMA_InitStructure.DMA_BufferSize = MAV_RX_BUF_SIZE;              // 你的宏定义大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;          
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                  // RX必须是循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;              // RX优先级设高一点防丢包
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1, &DMA_InitStructure);
////	
//	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; //抢占优先级，在使用FREERTOS时必须大于5否则没用
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; // 或 7，按你的FreeRTOS配置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
	
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Stream1, ENABLE);
    USART_Cmd(USART3, ENABLE);  //RX不需要开启DMA中断，他的标志是USART中断
	
}