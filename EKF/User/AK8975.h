#ifndef __AK8975_H
#define __AK8975_H
void AK_Init(void);
void AK_ReadID(uint8_t *AK_id);
typedef struct{
	float mag_x;
	float mag_y;
	float mag_z;
	
}AK8975_Info;
#endif 
