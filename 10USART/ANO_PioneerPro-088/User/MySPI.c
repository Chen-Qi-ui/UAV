#include "stm32f4xx.h"                  // Device header
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "../SystemClock/SystemClock.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "BMI088.h"
#include "stm32f4xx_spi.h"
void MySPI_W_Gyro_SS(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOC,Gyro_CS,(BitAction)BitValue);
}

void MySPI_W_Acc_SS(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOD,ACC_CS,(BitAction)BitValue);
}

void MySPI_W_SCK(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_13,(BitAction)BitValue);
}

void MySPI_W_MOSI(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_15,(BitAction)BitValue);
}

uint8_t MySPI_R_MISO(void)
{
	return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);
}

void MySPI_W_AK_SS(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_12,(BitAction)BitValue);
}


void MySPI_W_AK_SCK(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_13,(BitAction)BitValue);
}

void MySPI_W_AK_MOSI(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB,GPIO_Pin_15,(BitAction)BitValue);
}

uint8_t MySPI_R_AK_MISO(void)
{
	return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);
}
// ==================== 统一的SPI初始化函数 ====================
void MySPI_All_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;
    
    // =========== 1. 使能所有时钟 ===========
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | 
                          RCC_AHB1Periph_GPIOC | 
                          RCC_AHB1Periph_GPIOD, ENABLE);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    
    // =========== 2. 配置SPI2引脚为复用功能 ===========
    // SCK: PB13, MISO: PB14, MOSI: PB15
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          // 复用模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 配置引脚复用为SPI2
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);  // PB13 -> SPI2_SCK
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);  // PB14 -> SPI2_MISO
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);  // PB15 -> SPI2_MOSI
    
    // =========== 3. 配置CS引脚 ===========
    // 3.1 BMI088 陀螺仪CS (PC10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  // 上拉，默认不选中
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // 3.2 BMI088 加速度计CS (PD0)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // 3.3 AK8975 CS (PB12)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // =========== 4. 配置硬件SPI2 ===========
    // 注意：BMI088和AK8975都使用SPI Mode 3
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;          // CPOL=1 (空闲高电平)
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;         // CPHA=1 (第二个边沿采样)
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;            // 软件NSS控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  // 
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    
    SPI_Init(SPI2, &SPI_InitStructure);
    SPI_Cmd(SPI2, ENABLE);
    
    // =========== 5. 设置初始状态 ===========
    // 所有CS引脚拉高（不选中任何设备）
    MySPI_W_Gyro_SS(1);   // 陀螺仪CS高
    MySPI_W_Acc_SS(1);    // 加速度计CS高
    MySPI_W_AK_SS(1);     // 磁力计CS高
    
    // 可选：测试SPI是否正常工作
    // SPI_Test();
}


void MySPI_Gyro_Start(void)
{
	MySPI_W_Gyro_SS(0);
}

void MySPI_Gyro_Stop(void)
{
	MySPI_W_Gyro_SS(1);
}

void MySPI_Acc_Start(void)
{
	MySPI_W_Acc_SS(0);
}

void MySPI_Acc_Stop(void)
{
	MySPI_W_Acc_SS(1);
}

void MySPI_AK_Start(void)
{
	MySPI_W_AK_SS(0);
}

void MySPI_AK_Stop(void)
{
	MySPI_W_AK_SS(1);
}


uint8_t MySPI_SwapByte(uint8_t ByteSend)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) != SET);	//等待发送数据寄存器空
	
	SPI_I2S_SendData(SPI2, ByteSend);								//写入数据到发送数据寄存器，开始产生时序
	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) != SET);	//等待接收数据寄存器非空
	
	return SPI_I2S_ReceiveData(SPI2);								//读取接收到的数据并返回
}



