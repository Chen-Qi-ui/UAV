#include <stdint.h>
// FreeRTOS 头，防止隐式声明 vTaskDelay/pdMS_TO_TICKS（即使当前实现未用到也建议保留）
#include "FreeRTOS.h"
#include "task.h"
// MAVLink 头
#include "../mavlink2/minimal/mavlink.h"
#include "usb_mavlink_port.h"
#include "../USBStack/SRC/usb_cdc_port.h"  // CDC_SendData
// 用静态缓冲避免在任务栈上分配大数组


int mavlink_send_msg(const mavlink_message_t *msg) {
	uint8_t s_tx_buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(s_tx_buf, msg);
    if (len <= 0) return 0;

    // 可选：检查 CDC 环形发送缓冲空间，避免一次塞满
    // 如果空间不足，稍等再试
    int tries = 0;
    while (USBD_CDC_ACM_DataFree() < len) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (++tries > 50) break; // 最多等500ms
    }

    int sent = CDC_SendData(s_tx_buf, len);
    return sent;
}