#include "EKF.h"
#include "Calibaration.h"
#include "../main/Queues.h"
#include "../mavlink2/common/mavlink.h"

float last_time = 0;
void vEKF_ProcessTask(void *pvParameters) {
    IMU_Data imu_in;
    float roll, pitch, yaw;
    mavlink_message_t msg_attitude;
    EKF_Init(); //
    for (;;) {
        if (xQueueReceive(xEulerQueue, &imu_in, portMAX_DELAY) == pdPASS) {
            // 在 vEKF_ProcessTask 中修改
float current_time = get_time_usec_precise(); // 获取微秒时间
float32_t real_dt = (last_time == 0) ? 0.010f : (current_time - last_time) * 0.000001f;
last_time = current_time;
// 将真正的测量值传给 EKF
EKF_Predict(imu_in.gyro[0]*1.2857, imu_in.gyro[1]*1.2857, imu_in.gyro[2]*1.2857, real_dt);
           
            EKF_Update_Accel(imu_in.acc[0], imu_in.acc[1], imu_in.acc[2]);



if ((imu_in.mag[0] * imu_in.mag[0] + 
                 imu_in.mag[1] * imu_in.mag[1] + imu_in.mag[2] * imu_in.mag[2]) < 2.8f) 
            {
                    EKF_Update_Mag(imu_in.mag[0], imu_in.mag[1], imu_in.mag[2]);
			}


            Get_EulerAngles(&roll, &pitch, &yaw);
            mavlink_msg_attitude_pack(
                1, MAV_COMP_ID_AUTOPILOT1, &msg_attitude,
                get_time_usec_precise() / 1000,
                roll * 0.0174533f, pitch * 0.0174533f, yaw * 0.0174533f,
                imu_in.gyro[0], imu_in.gyro[1], imu_in.gyro[2]
            );

           
        }
		 mavlink_send_msg(&msg_attitude);
    }
}


