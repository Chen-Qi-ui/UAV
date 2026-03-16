#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "../SystemClock/SystemClock.h"
#include "../User/LED.h"
#include "../USB/USB_Clock.h"
#include "../USB/USB_CDC_App.h"   
#include "MySPI.h"
#include "../User/BMI088.h"
#include "../User/AK8975.h"
#include "../main/Queues.h"
#include "rl_usb.h"
#include "EKF.h"
#include "Calibaration.h"
#include "mavlink_types.h"
#include "semphr.h"
#include "stm32f4xx_dma.h"
#include "USART.h"
#include "Task_Create.h"
#include "../mavlink2/minimal/mavlink.h"
#include "stm32f4xx_usart.h"
extern QueueHandle_t xMavTxQueue;
extern SemaphoreHandle_t xDmaTxSem;
extern uint8_t g_tx_buffer[MAVLINK_MAX_PACKET_LEN];
extern uint8_t g_rx_buffer[MAV_RX_BUF_SIZE];
int tmpp;
void vMavRx_Task(void *pvParameters)
{
    mavlink_message_t msg;
    mavlink_status_t status;
    static uint16_t last_pos = 0;
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint16_t current_pos = MAV_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1);
        
        while(last_pos != current_pos)
        {            
            if(mavlink_parse_char(MAVLINK_COMM_0, g_rx_buffer[last_pos], &msg, &status))
            {
               tmpp++;
            }
            last_pos = (last_pos + 1) % MAV_RX_BUF_SIZE; 
        }
    }
}


void vMavTx_Task(void *pvParameters)
{
    mavlink_message_t tx_msg;
    uint16_t len;
    
    for(;;)
    {
        if(xQueueReceive(xMavTxQueue, &tx_msg, portMAX_DELAY) == pdPASS)
        {
            len = mavlink_msg_to_send_buffer(g_tx_buffer, &tx_msg);
            
            DMA_Cmd(DMA1_Stream4, DISABLE);
            while(DMA_GetCmdStatus(DMA1_Stream4) != DISABLE);
            
            DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4 | DMA_FLAG_TEIF4 | DMA_FLAG_DMEIF4 | DMA_FLAG_FEIF4);
            
            DMA1_Stream4->M0AR = (uint32_t)g_tx_buffer;
            DMA1_Stream4->NDTR = len;
            

            DMA_Cmd(DMA1_Stream4, ENABLE);
            
           
            

            
            xSemaphoreTake(xDmaTxSem, pdMS_TO_TICKS(100));
        }
    }
}

//void vMavTx_Task(void *pvParameters)
//{
//    mavlink_message_t tx_msg;
//    uint16_t len;
//    
//    for(;;)
//    {
//        if(xQueueReceive(xMavTxQueue, &tx_msg, portMAX_DELAY) == pdPASS)
//        {
//            temp++; // 留着观察
//            len = mavlink_msg_to_send_buffer(g_tx_buffer, &tx_msg);
//            
//            // ====== 终极测试：绕过 DMA，使用 CPU 暴力直发 ======
//            for(uint16_t i = 0; i < len; i++)
//            {
//                // 死等串口发送寄存器空
//                while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
//                // 强行发送一个字节
//                USART_SendData(USART3, g_tx_buffer[i]);
//            }
//            
//            // 等待最后一个字节发送完成
//            while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
//            // ====== 直发结束 ======
//            
//            // ⚠️ 绝不能在这里加 xSemaphoreTake，因为根本没有触发中断！
//        }
//    }
//}

void Mav_Send(mavlink_message_t msg)
{
	 xQueueSend(xMavTxQueue, &msg, 0);
}



