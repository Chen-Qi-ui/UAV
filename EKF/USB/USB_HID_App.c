#include "rl_usb.h"
#include <stdint.h>

/* 初始化 HID（内部调用 usbd_init + usbd_connect） */
void HID_Init(void) {
    /* 初始化并连接 USB 设备栈 */
    usbd_init();
    usbd_connect(1);
}

/* CDC_SendData 的实现由 usb_cdc_port.c 提供，这里不重复定义 */