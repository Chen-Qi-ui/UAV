#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "rl_usb.h"

#include "../mavlink2/minimal/mavlink.h"
#include "../USBStack/SRC/usb_cdc_port.h"  
#include "Mavlink_Task.h"
extern uint8_t os_is_running;
void vUSBTickTask(void *pvParameters) {
	os_is_running = 1;
    usbd_connect(1);
    vTaskDelay(1);

    const uint8_t system_id    = 1;                         // 系统ID
    const uint8_t component_id = MAV_COMP_ID_AUTOPILOT1;    // 组件ID（飞控）
    const uint8_t type         = MAV_TYPE_QUADROTOR;        // 或 MAV_TYPE_GENERIC
    const uint8_t autopilot    = MAV_AUTOPILOT_GENERIC;     // 自研/简易飞控
    const uint8_t base_mode    = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    const uint8_t system_status= MAV_STATE_ACTIVE;



    for (;;) {
        // HEARTBEAT 每秒一次（QGC用它判断在线状态和类型）
        mavlink_message_t msg;
        uint32_t custom_mode = 0;
        mavlink_msg_heartbeat_pack(system_id, component_id, &msg,
                                   type, autopilot, base_mode, custom_mode, system_status);
        Mav_Send(msg);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

