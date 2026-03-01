#include "rl_usb.h"
#include "usb_for_lib.h"

// 简单端口初始化/反初始化/复位，返回成功
int32_t USBD_CDC_ACM_PortInitialize(void) { return 1; }
int32_t USBD_CDC_ACM_PortUninitialize(void) { return 1; }
int32_t USBD_CDC_ACM_PortReset(void) { return 1; }

// 设置/获取 Line Coding：直接接受主机设置；获取时给出一个默认值
int32_t USBD_CDC_ACM_PortSetLineCoding(CDC_LINE_CODING *lc) {
  // 可在此处同步到你实际的串口参数，如果只是虚拟串口，接受即可
  return (lc != NULL) ? 1 : 0;
}

int32_t USBD_CDC_ACM_PortGetLineCoding(CDC_LINE_CODING *lc) {
  if (!lc) return 0;
  lc->dwDTERate   = 9600;
  lc->bCharFormat = 0; // 1 stop bit
  lc->bParityType = 0; // none
  lc->bDataBits   = 8;
  return 1;
}

// 控制线状态（DTR/RTS），枚举时主机会发，接受即可
int32_t USBD_CDC_ACM_PortSetControlLineState(uint16_t ctrl_bmp) {
  (void)ctrl_bmp;
  return 1;
}

// 对外暴露一个 CDC 发送包装函数，供你的任务调用
int32_t CDC_SendData(const uint8_t *buf, int32_t len) {
  return USBD_CDC_ACM_DataSend(buf, len);
}