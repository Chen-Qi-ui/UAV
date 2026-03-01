#ifndef __EKF_H
#define __EKF_H
#include "arm_math.h"
#include "math.h"
void EKF_Init(void);
void EKF_Predict(float32_t Wx,float32_t Wy,float32_t Wz,float32_t dt);
void EKF_Update_Accel(float32_t ax, float32_t ay, float32_t az);
void EKF_Update_Mag(float32_t mx, float32_t my, float32_t mz);
void Get_EulerAngles(float32_t* roll, float32_t* pitch, float32_t* yaw);
#endif