// File: Queues.h
#ifndef __QUEUES_H
#define __QUEUES_H

#include "FreeRTOS.h"
#include "queue.h"
#include "../User/BMI088.h"
#include "../User/AK8975.h"

// 声明队列句柄为外部变量（在其他文件中定义）
extern QueueHandle_t xBMI088Queue;
extern QueueHandle_t xAK8975Queue;
extern QueueHandle_t xEulerQueue;
// 还可以声明其他需要的队列
// extern QueueHandle_t xControlQueue;
// extern QueueHandle_t xMavlinkQueue;

#endif