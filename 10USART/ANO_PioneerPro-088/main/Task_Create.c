#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "../SystemClock/SystemClock.h"
#include "../User/LED.h"

#include "../USB/USB_Clock.h"
#include "../USB/USB_CDC_App.h"   
#include "MySPI.h"
#include "../main/Task_Create.h"
#include "../User/BMI088.h"
#include "../User/AK8975.h"
#include "../main/Queues.h"
#include "rl_usb.h"
#include "EKF.h"
#include "Calibaration.h"
#include "mavlink_types.h"
#include "semphr.h"
#include "USART.h"
#include "misc.h"
extern void vUSBTickTask(void *pvParameters);
extern void vLEDTask(void *pvParameters);
extern void vBMI088_Task(void *pvParameters);
extern void vAK8975_Task(void *pvParameters);
extern void vSensor_ReceiveTask(void *pvParameters);
extern void vEKF_ProcessTask(void *pvParameters);
extern void vMavTx_Task(void *pvParameters);
extern void vMavRx_Task(void *pvParameters);
QueueHandle_t xBMI088Queue = NULL;
QueueHandle_t xAK8975Queue = NULL;
QueueHandle_t xEulerQueue = NULL;
QueueHandle_t xMavTxQueue = NULL;
SemaphoreHandle_t xDmaTxSem;
TaskHandle_t xRxTaskHandle;
void Task_Create(void)
{
	xTaskCreate(vLEDTask,"LED",  512, NULL, 1, NULL);
    xTaskCreate(vUSBTickTask,"HID",  512, NULL, 8, NULL);
//    xTaskCreate(vBMI088_Task,"Acc",  512, NULL, 5, NULL);
//	xTaskCreate(vAK8975_Task,"AK",  512, NULL, 5, NULL);
//	xTaskCreate(vSensor_ReceiveTask,"Sensor_Receive",  512, NULL, 4, NULL);
//	xTaskCreate(vEKF_ProcessTask,"EKF",1024,NULL, 7, NULL);
	xTaskCreate(vMavTx_Task,"MavTx",1024,NULL,6,NULL);
	xTaskCreate(vMavRx_Task,"MavRx",1024,NULL,6,&xRxTaskHandle);
}

void create_all_queue(void)
{
	xBMI088Queue = xQueueCreate(1,sizeof(BMI088_Info));
    xAK8975Queue = xQueueCreate(1,sizeof(AK8975_Info));
	xEulerQueue = xQueueCreate(1,sizeof(IMU_Data));
	xMavTxQueue = xQueueCreate(1,sizeof(mavlink_message_t));
}

void Create_Semaphore(void)
{
   xDmaTxSem=xSemaphoreCreateBinary();
	
	
}

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}


extern uint8_t os_is_running;

void Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    SystemClock_Config(); // 必须有！
    USB_EnablePLL48CLK();
    usbd_init();
    LED_Init();
    MySPI_All_Init();
    USART3_Config(); 
    
    Create_Semaphore(); 
    create_all_queue();
    Task_Create();      
    DWT_Init();
    
    vTaskStartScheduler();
}



void assert_failed(uint8_t* file, uint32_t line)
{
    (void)file;
    (void)line;
}