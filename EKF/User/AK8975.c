#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "../SystemClock/SystemClock.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "MySPI.h"
#include "AK8975.h"
#include "../main/Queues.h"
uint8_t AK_id=0;
uint8_t st2 = 0;
AK8975_Info ak8975_info;
void AK_Init(void)
{
    MySPI_Acc_Stop();
	MySPI_Gyro_Stop();

    vTaskDelay(pdMS_TO_TICKS(10));  
    
}

void AK_st1(void)
{
    MySPI_AK_Start();
	MySPI_SwapByte(0x0A & 0x7F); 
	MySPI_SwapByte(0x01); 
    MySPI_AK_Stop();
	vTaskDelay(pdMS_TO_TICKS(10)); 
}
void AK_ReadID(uint8_t *AK_id)
{
    MySPI_W_Gyro_SS(1); // 保险，确保其他从机不选中
    MySPI_W_Acc_SS(1);

    MySPI_AK_Start();                        // CS 低
    MySPI_SwapByte(0x00 | 0x80);             // 发送地址+读位
    *AK_id = MySPI_SwapByte(0xFF);           // 发送哑元，接收ID
    MySPI_AK_Stop();                         // CS 高
}
int16_t magData[3];
void AK_ReadMagData(int16_t *magData) 
{
    uint8_t magRawData[7];
    MySPI_AK_Start();
    MySPI_SwapByte(0x03 | 0x80);  // Address of HXL register with read bit set
    
    for(int i = 0; i < 7; i++)
    {
        magRawData[i] = MySPI_SwapByte(0xFF);  // Dummy byte to receive data
    }
    MySPI_AK_Stop();

    // Convert raw data to 16-bit signed values

    magData[0] = (int16_t)((magRawData[1] << 8) | magRawData[0]);
    magData[1] = (int16_t)((magRawData[3] << 8) | magRawData[2]);
    magData[2] = (int16_t)((magRawData[5] << 8) | magRawData[4]);
	
}


void vAK8975_Task(void *pvParameters)
{
    
	AK_Init();
    AK_ReadID(&AK_id);
	
    for(;;)
    {
		AK_st1();
        AK_ReadMagData(magData); // Read magnetometer data
        ak8975_info.mag_x = magData[0]*0.3f;	
        ak8975_info.mag_y = magData[1]*0.3f;
        ak8975_info.mag_z = magData[2]*0.3f;
		xQueueOverwrite(xAK8975Queue, &ak8975_info);
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}
