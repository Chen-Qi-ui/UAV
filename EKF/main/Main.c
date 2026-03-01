#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "../SystemClock/SystemClock.h"
#include "../User/LED.h"

#include "../USB/USB_Clock.h"
#include "../USB/USB_CDC_App.h"   /* 替换 Ano_USB.h */
#include "../main/Task_Create.h"





int main(void)
{
    Init();
    while (1) {

    }
}
