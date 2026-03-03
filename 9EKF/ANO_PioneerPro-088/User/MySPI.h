#ifndef __MYSPI_H
#define __MYSPI_H
void MySPI_W_Gyro_SS(uint8_t BitValue);
void MySPI_W_Acc_SS(uint8_t BitValue);
void MySPI_W_SCK(uint8_t BitValue);
void MySPI_W_MOSI(uint8_t BitValue);
uint8_t MySPI_R_MISO(void);
void MySPI_All_Init(void);
void MySPI_Gyro_Start(void);
void MySPI_Gyro_Stop(void);
void MySPI_Acc_Start(void);
void MySPI_Acc_Stop(void);
void MySPI_W_AK_SS(uint8_t BitValue);
void MySPI_W_AK_SCK(uint8_t BitValue);
void MySPI_W_AK_MOSI(uint8_t BitValue);
uint8_t MySPI_R_AK_MISO(void);
void MySPI_AK_Start(void);
void MySPI_AK_Stop(void);
uint8_t MySPI_SwapByte_AK_Mode3(uint8_t ByteSend);
uint8_t MySPI_SwapByte(uint8_t ByteSend);
#endif
