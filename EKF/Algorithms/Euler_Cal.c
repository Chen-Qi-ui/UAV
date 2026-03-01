#include "EKF.h"
#include "Calibaration.h"
#include "../main/Queues.h"
#include "../mavlink2/common/mavlink.h"


float ROLL,YAW,PITCH;
 	float ROLL,PITCH,YAW;
void vEKF_ProcessTask(void *pvParameters) {
    IMU_Data imu_in;
    float roll, pitch, yaw;
    mavlink_message_t msg_attitude;
    
    EKF_Init(); //

    for (;;) {
        if (xQueueReceive(xEulerQueue, &imu_in, portMAX_DELAY) == pdPASS) {
            EKF_Predict(imu_in.gyro[0], imu_in.gyro[1], imu_in.gyro[2], 0.010f);
           
            EKF_Update_Accel(imu_in.acc[0], imu_in.acc[1], imu_in.acc[2]);



if ((imu_in.mag[0] * imu_in.mag[0] + 
                 imu_in.mag[1] * imu_in.mag[1] + imu_in.mag[2] * imu_in.mag[2]) < 1.8f) 
            {
                    EKF_Update_Mag(imu_in.mag[0], imu_in.mag[1], imu_in.mag[2]);
			}


            Get_EulerAngles(&roll, &pitch, &yaw);
            ROLL = roll,PITCH=pitch,YAW=yaw;
            mavlink_msg_attitude_pack(
                1, MAV_COMP_ID_AUTOPILOT1, &msg_attitude,
                get_time_usec_simple() / 1000,
                roll * 0.0174533f, pitch * 0.0174533f, yaw * 0.0174533f,
                imu_in.gyro[0], imu_in.gyro[1], imu_in.gyro[2]
            );

           
        }
		 mavlink_send_msg(&msg_attitude);
    }
}