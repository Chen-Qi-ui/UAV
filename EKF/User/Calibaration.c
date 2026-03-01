#include "../User/BMI088.h"
#include "../User/AK8975.h"
#include "../USB/USB_CDC_App.h"
#include "../USBStack/usb_mavlink_port.h"
#include "../main/Queues.h"
#include "../mavlink2/minimal/mavlink.h"
#include "../mavlink2/common/mavlink_msg_highres_imu.h"
#include "../USBStack/usb_mavlink_port.h" 
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "../main/Task_Create.h"
#include "../main/Queues.h"
#include "Calibaration.h"
#include "core_cm4.h"
uint64_t get_time_usec_simple(void) {
    return (uint64_t)xTaskGetTickCount() * 1000;  // 假设1ms tick
}

void vSensor_ReceiveTask(void *pvParameters)
{

    const float bg_prime[3] = {-0.00221256f, -0.00055181f, 0.00194304f};
    const float Ta[3][3] = {
        { 1.00000000f, -0.00000000f,  0.00028534f},
        { 0.00000000f,  1.00000000f, -0.00193542f},
        {-0.00028534f,  0.00193542f,  1.00000000f}
    };
    const float Ka_acc[3] = {1.00061646f, 1.00475640f, 1.00443049f};
    const float ba_prime_acc[3] = {0.16162917f, 0.01555071f, -0.12598775f};


    const float ba_prime_mag[3] = {40.582975f, -39.493498f, 26.187746f};

    const float Ka_mag[3] = {0.037873f, 0.024459f, 0.025385f};

    BMI088_Info bmi_data;
    AK8975_Info ak_data;
    BaseType_t xbmi_Status, xak8975_Status;
	IMU_Data Cal_Data;
    for(;;)
    {
		
        mavlink_message_t msg_sensor;
        xbmi_Status = xQueueReceive(xBMI088Queue, &bmi_data, pdMS_TO_TICKS(5));
        xak8975_Status = xQueueReceive(xAK8975Queue, &ak_data, pdMS_TO_TICKS(5));

        if(xbmi_Status == pdPASS || xak8975_Status == pdPASS)
        {
            float cal_acc[3], temp_acc[3], cal_gyro[3], cal_mag[3];

            cal_gyro[0] = bmi_data.gyro_x_w * 0.0174533f + bg_prime[0];
            cal_gyro[1] = bmi_data.gyro_y_w * 0.0174533f + bg_prime[1];
            cal_gyro[2] = bmi_data.gyro_z_w * 0.0174533f + bg_prime[2];

            temp_acc[0] = (bmi_data.acc_x_g * 9.80665f + ba_prime_acc[0]) * Ka_acc[0];
            temp_acc[1] = (bmi_data.acc_y_g * 9.80665f + ba_prime_acc[1]) * Ka_acc[1];
            temp_acc[2] = (bmi_data.acc_z_g * 9.80665f + ba_prime_acc[2]) * Ka_acc[2];
            
            cal_acc[0] = Ta[0][0]*temp_acc[0] + Ta[0][1]*temp_acc[1] + Ta[0][2]*temp_acc[2];
            cal_acc[1] = Ta[1][0]*temp_acc[0] + Ta[1][1]*temp_acc[1] + Ta[1][2]*temp_acc[2];
            cal_acc[2] = Ta[2][0]*temp_acc[0] + Ta[2][1]*temp_acc[1] + Ta[2][2]*temp_acc[2];


            cal_mag[0] = (ak_data.mag_x + ba_prime_mag[0]) * Ka_mag[0];
            cal_mag[1] = (ak_data.mag_y + ba_prime_mag[1]) * Ka_mag[1];
            cal_mag[2] = (ak_data.mag_z + ba_prime_mag[2]) * Ka_mag[2];
            Cal_Data.gyro[0]=cal_gyro[0];Cal_Data.gyro[1]=cal_gyro[1];Cal_Data.gyro[2]=cal_gyro[2];
            Cal_Data.acc[0]=cal_acc[0]/9.80665f;Cal_Data.acc[1]=cal_acc[1]/9.80665f;Cal_Data.acc[2]=cal_acc[2]/9.80665f;
			Cal_Data.mag[0]=cal_mag[0];Cal_Data.mag[1]=cal_mag[1];Cal_Data.mag[2]=cal_mag[2];
			static float lpf_mx = 0, lpf_my = 0, lpf_mz = 0;
            const float alpha = 0.05f; // 滤波系数，越小越平滑 (0.01~0.2)


// 应用低通滤波
          lpf_mx = alpha * cal_mag[0] + (1.0f - alpha) * lpf_mx;
          lpf_my = alpha * cal_mag[1] + (1.0f - alpha) * lpf_my;
          lpf_mz = alpha * cal_mag[2] + (1.0f - alpha) * lpf_mz;

// 将滤波后的值传给队列或 EKF
          Cal_Data.mag[0] = lpf_mx; 
          Cal_Data.mag[1] = lpf_my; 
          Cal_Data.mag[2] = lpf_mz;
            mavlink_msg_highres_imu_pack(
                1, MAV_COMP_ID_AUTOPILOT1, &msg_sensor, get_time_usec_simple(),
                cal_acc[0], cal_acc[1], cal_acc[2],
                cal_gyro[0], cal_gyro[1], cal_gyro[2],
                cal_mag[0], cal_mag[1], cal_mag[2],
                0.0f, 0.0f, 0.0f, 12.0f, 0x01FF, 0
            );
          xQueueOverwrite(xEulerQueue,&Cal_Data );  
        }  
		mavlink_send_msg(&msg_sensor); 
		vTaskDelay(pdMS_TO_TICKS(10));
    }
}