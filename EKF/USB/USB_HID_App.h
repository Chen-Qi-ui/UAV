#ifndef USB_CDC_App_H
#define USB_CDC_App_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 HID（内部调用 usbd_init + usbd_connect） */
void HID_Init(void);

/* 发送一条文本。最多发送 63 字节有效负载（会被封装成 64 字节 HID 报告）。
   返回 1 表示已受理发送，0 表示忙（下次再试）。*/


#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_App_H */
