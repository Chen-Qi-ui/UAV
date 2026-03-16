#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "../SystemClock/SystemClock.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "MySPI.h"
#include "W25Q32_Ins.h"
void W25Q32_Init(void)
{
	MySPI_Init();

}

void W25Q32_ReadID(uint8_t *MID,uint16_t *DID)
{
	MySPI_Start();
	
	MySPI_SwapByte(W25Q32_READ_UNIQUE_ID);  //JEDEC ID
	*MID=MySPI_SwapByte(0xFF);  
	*DID=MySPI_SwapByte(0xFF);   //返回高8位
	*DID<<=8;
	*DID |= MySPI_SwapByte(0xFF);
	MySPI_Stop();
}

