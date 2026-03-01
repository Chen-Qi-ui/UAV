#include "arm_math.h"
#include "math.h"


typedef struct{
	float32_t q[4];
	float32_t P_data[16];
	float32_t Q_data[16];
	float32_t R_acc_data[9];
	float32_t R_mag_data[9];
	
	arm_matrix_instance_f32 mat_P, mat_Q, mat_R_acc, mat_R_mag;
}EKF_Object;

EKF_Object imu_ekf;
static float32_t F_data[16], FT_data[16];
static float32_t H_data[12], HT_data[12];
static float32_t K_data[12], S_data[9], SI_data[9];
static float32_t tmp_4x4_1[16], tmp_4x4_2[16];
static float32_t tmp_3x4[12], tmp_4x3[12];

arm_matrix_instance_f32 mat_F, mat_FT, mat_H, mat_HT, mat_K, mat_S, mat_SI;


void EKF_Init(void)
{
	imu_ekf.q[0]=1.0f;imu_ekf.q[1]=0.0f;imu_ekf.q[2]=0.0f;imu_ekf.q[3]=0.0f;
	memset(imu_ekf.P_data,0,sizeof(imu_ekf.P_data));
	//初始协方差P 含义：对初始状态的“不确定度”  如果 $P$ 很大，意味着你不相信初始值，EKF 会在起飞前几次更新中非常快地修正姿态。
	imu_ekf.P_data[0]=imu_ekf.P_data[5]=imu_ekf.P_data[10]=imu_ekf.P_data[15]=0.1f;
	//过程噪声Q（陀螺仪），过程噪声越大，越不信任  Q 越大：意味着你觉得陀螺仪积分很不准，
	//系统会更加依赖加速度计和磁力计来修正姿态。表现为反应快，但波形抖动大。
	memset(imu_ekf.Q_data,0,sizeof(imu_ekf.Q_data));  //清0
	imu_ekf.Q_data[0]=imu_ekf.Q_data[5]=imu_ekf.Q_data[10]=imu_ekf.Q_data[15]=0.01f;
	//加速度测量噪声R
	memset(imu_ekf.R_acc_data,0,sizeof(imu_ekf.R_acc_data));
	imu_ekf.R_acc_data[0]=imu_ekf.R_acc_data[4]=imu_ekf.R_acc_data[8]=0.10f;
	//磁力计测量噪声R
	//除非磁力计的数据非常稳定且持续很久，否则不要轻易修改我的 Yaw 角（判断稳定由S矩阵）
	memset(imu_ekf.R_mag_data,0,sizeof(imu_ekf.R_mag_data));
	imu_ekf.R_mag_data[0]=imu_ekf.R_mag_data[4]=imu_ekf.R_mag_data[8]=100.0f;
	
	    // 初始化 CMSIS 矩阵
    arm_mat_init_f32(&imu_ekf.mat_P, 4, 4, imu_ekf.P_data);  //将矩阵结构体的指针指向imu_ekf.P_data
    arm_mat_init_f32(&imu_ekf.mat_Q, 4, 4, imu_ekf.Q_data);
    arm_mat_init_f32(&mat_F, 4, 4, F_data);
    arm_mat_init_f32(&mat_FT, 4, 4, FT_data);
    arm_mat_init_f32(&mat_H, 3, 4, H_data);
    arm_mat_init_f32(&mat_HT, 4, 3, HT_data);
    arm_mat_init_f32(&mat_K, 4, 3, K_data);
    arm_mat_init_f32(&mat_S, 3, 3, S_data);
    arm_mat_init_f32(&mat_SI, 3, 3, SI_data);
}



void EKF_Predict(float32_t Wx,float32_t Wy,float32_t Wz,float32_t dt)
{
	float32_t hdt = 0.5f*dt;
	float32_t q0=imu_ekf.q[0],q1=imu_ekf.q[1],q2=imu_ekf.q[2],q3=imu_ekf.q[3];
	//1.状态预测，一阶龙格库塔法
	imu_ekf.q[0] +=  (-q1*Wx-q2*Wy-q3*Wz)*hdt;
	imu_ekf.q[1] +=  (q0*Wx+q2*Wz-q3*Wy)*hdt;
	imu_ekf.q[2] +=  (q0*Wy-q1*Wz+q3*Wx)*hdt;
	imu_ekf.q[3] +=  (q0*Wz+q1*Wy-q2*Wx)*hdt;
	  // 四元数归一化
    float32_t norm = sqrtf(imu_ekf.q[0]*imu_ekf.q[0] + imu_ekf.q[1]*imu_ekf.q[1] + imu_ekf.q[2]*imu_ekf.q[2] + imu_ekf.q[3]*imu_ekf.q[3]);
    imu_ekf.q[0] /= norm; imu_ekf.q[1] /= norm; imu_ekf.q[2] /= norm; imu_ekf.q[3] /= norm;

    // 2. 构建状态转移矩阵雅可比 F
    F_data[0] = 1.0f;    F_data[1] = -hdt*Wx; F_data[2] = -hdt*Wy; F_data[3] = -hdt*Wz;
    F_data[4] = hdt*Wx;  F_data[5] = 1.0f;    F_data[6] = hdt*Wz;  F_data[7] = -hdt*Wy;
    F_data[8] = hdt*Wy;  F_data[9] = -hdt*Wz; F_data[10] = 1.0f;   F_data[11] = hdt*Wx;
    F_data[12] = hdt*Wz; F_data[13] = hdt*Wy; F_data[14] = -hdt*Wx; F_data[15] = 1.0f;

    // 3. 协方差预测: P = F * P * F' + Q
    arm_matrix_instance_f32 tmp1, tmp2;
    arm_mat_init_f32(&tmp1, 4, 4, tmp_4x4_1);
    arm_mat_init_f32(&tmp2, 4, 4, tmp_4x4_2);

    arm_mat_trans_f32(&mat_F, &mat_FT);
    arm_mat_mult_f32(&mat_F, &imu_ekf.mat_P, &tmp1);     // F*P
    arm_mat_mult_f32(&tmp1, &mat_FT, &tmp2);            // F*P*FT
    arm_mat_add_f32(&tmp2, &imu_ekf.mat_Q, &imu_ekf.mat_P); // +Q

}


void EKF_Update_Accel(float32_t ax, float32_t ay, float32_t az) {
    float32_t q0 = imu_ekf.q[0], q1 = imu_ekf.q[1], q2 = imu_ekf.q[2], q3 = imu_ekf.q[3];

    // 1. 预测观测值 h(q) (重力在机体系投影)
    float32_t vx = 2.0f * (q1*q3 - q0*q2);
    float32_t vy = 2.0f * (q0*q1 + q2*q3);
    float32_t vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 2. 构建观测雅可比 H (3x4)
    H_data[0] = -2.0f*q2; H_data[1] =  2.0f*q3; H_data[2] = -2.0f*q0; H_data[3] =  2.0f*q1;
    H_data[4] =  2.0f*q1; H_data[5] =  2.0f*q0; H_data[6] =  2.0f*q3; H_data[7] =  2.0f*q2;
    H_data[8] =  2.0f*q0; H_data[9] = -2.0f*q1; H_data[10] = -2.0f*q2; H_data[11] =  2.0f*q3;

    // 3. 计算卡尔曼增益 K = P*H' * inv(H*P*H' + R)
    arm_matrix_instance_f32 tmp_PH34, tmp_HPH33,tmp1;
    arm_mat_init_f32(&tmp_PH34, 4, 3, tmp_4x3);
    arm_mat_init_f32(&tmp_HPH33, 3, 3, S_data);

    arm_mat_trans_f32(&mat_H, &mat_HT);
    arm_mat_mult_f32(&imu_ekf.mat_P, &mat_HT, &tmp_PH34); // P*HT
    arm_mat_mult_f32(&mat_H, &tmp_PH34, &tmp_HPH33);    // H*P*HT
    
    // S = H*P*HT + R
    S_data[0]+=0.01f; S_data[4]+=0.01f; S_data[8]+=0.01f;
    arm_mat_inverse_f32(&tmp_HPH33, &mat_SI);           // S的倒数
    arm_mat_mult_f32(&tmp_PH34, &mat_SI, &mat_K);       // K = P*HT*inv(S)
    arm_mat_init_f32(&tmp1, 4, 4, tmp_4x4_1);
    // 4. 修正状态
    float32_t res[3] = {ax - vx, ay - vy, az - vz};
    for(int i=0; i<4; i++) {
        imu_ekf.q[i] += K_data[i*3+0]*res[0] + K_data[i*3+1]*res[1] + K_data[i*3+2]*res[2];
    }
	float32_t norm = sqrtf(imu_ekf.q[0]*imu_ekf.q[0] + imu_ekf.q[1]*imu_ekf.q[1] + 
                   imu_ekf.q[2]*imu_ekf.q[2] + imu_ekf.q[3]*imu_ekf.q[3]);
    imu_ekf.q[0] /= norm; imu_ekf.q[1] /= norm; imu_ekf.q[2] /= norm; imu_ekf.q[3] /= norm;

    // P = (I - K*H) * P
    arm_matrix_instance_f32 mat_I4, mat_KH;
    float32_t I_data[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    float32_t KH_data[16];
    arm_mat_init_f32(&mat_I4, 4, 4, I_data);
    arm_mat_init_f32(&mat_KH, 4, 4, KH_data);
    arm_mat_mult_f32(&mat_K, &mat_H, &mat_KH);
    arm_mat_sub_f32(&mat_I4, &mat_KH, &mat_FT); // Borrow FT_data for (I-KH)
    arm_mat_mult_f32(&mat_FT, &imu_ekf.mat_P, &tmp1); // Borrow tmp1
    memcpy(imu_ekf.P_data, tmp_4x4_1, sizeof(imu_ekf.P_data));
}


void EKF_Update_Mag(float32_t mx, float32_t my, float32_t mz) {
    float32_t q0 = imu_ekf.q[0];
    float32_t q1 = imu_ekf.q[1];
    float32_t q2 = imu_ekf.q[2];
    float32_t q3 = imu_ekf.q[3];


    // 1. 动态参考场对齐 
    // 将机体系测量值投影回地理系(n)，以计算当前的 bx 和 bz
    float32_t hx = mx*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + my*2.0f*(q1*q2 - q0*q3) + mz*2.0f*(q1*q3 + q0*q2);
    float32_t hy = mx*2.0f*(q1*q2 + q0*q3) + my*(q0*q0 - q1*q1 + q2*q2 - q3*q3) + mz*2.0f*(q2*q3 - q0*q1);
    
    // 令世界系 y 轴为 0，计算水平分量 bx 和垂直分量 bz
	//人为定义地理坐标系的 x 轴指向“磁北”。既然 x 轴已经对准了磁北，那么磁场在 y 轴（东向）的分量理所当然应该是 0。
	//从而只更新yaw角
    float32_t bx = sqrtf(hx*hx + hy*hy);
    float32_t bz = mx*2.0f*(q1*q3 - q0*q2) + my*2.0f*(q2*q3 + q0*q1) + mz*(q0*q0 - q1*q1 - q2*q2 + q3*q3);

    // 2. 预测观测值 h_mag 
    // 基于当前姿态和参考场，预测传感器应该读到的值//机体系此时
    float32_t mx_p = bx*(q0*q0 + q1*q1 - q2*q2 - q3*q3) + bz*2.0f*(q1*q3 - q0*q2);
    float32_t my_p = bx*2.0f*(q1*q2 - q0*q3)             + bz*2.0f*(q0*q1 + q2*q3);
    float32_t mz_p = bx*2.0f*(q1*q3 + q0*q2)             + bz*(q0*q0 - q1*q1 - q2*q2 + q3*q3);

    // 3. 构建观测雅可比矩阵 H_mag (3x4)
    H_data[0] =  2.0f*bx*q0 - 2.0f*bz*q2;  H_data[1] = 2.0f*bx*q1 + 2.0f*bz*q3; 
    H_data[2] = -2.0f*bx*q2 - 2.0f*bz*q0;  H_data[3] = -2.0f*bx*q3 + 2.0f*bz*q1;

    H_data[4] = -2.0f*bx*q3 + 2.0f*bz*q1;  H_data[5] = 2.0f*bx*q2 + 2.0f*bz*q0; 
    H_data[6] =  2.0f*bx*q1 + 2.0f*bz*q3;  H_data[7] = -2.0f*bx*q0 + 2.0f*bz*q2;

    H_data[8] =  2.0f*bx*q2 + 2.0f*bz*q0;  H_data[9] = 2.0f*bx*q3 - 2.0f*bz*q1; 
    H_data[10] = 2.0f*bx*q0 - 2.0f*bz*q2;  H_data[11] = 2.0f*bx*q1 + 2.0f*bz*q3;

    // 4. 矩阵运算计算卡尔曼增益 K
    arm_matrix_instance_f32 tmp_PH34, tmp_HPH33, tmp_S;
    arm_mat_init_f32(&tmp_PH34, 4, 3, tmp_4x3);
    arm_mat_init_f32(&tmp_HPH33, 3, 3, S_data);
    arm_mat_init_f32(&tmp_S, 3, 3, S_data);

    arm_mat_trans_f32(&mat_H, &mat_HT);
    arm_mat_mult_f32(&imu_ekf.mat_P, &mat_HT, &tmp_PH34);  // P*HT
    arm_mat_mult_f32(&mat_H, &tmp_PH34, &tmp_HPH33);     // H*P*HT
    
    // S = H*P*HT + R_mag
    arm_mat_add_f32(&tmp_HPH33, &imu_ekf.mat_R_mag, &tmp_S);
    arm_mat_inverse_f32(&tmp_S, &mat_SI);                // inv(S)
    arm_mat_mult_f32(&tmp_PH34, &mat_SI, &mat_K);        // K = P*HT*inv(S)

    //  5. 修正状态向量 q 
    float32_t res[3] = {mx - mx_p, my - my_p, mz - mz_p}; //残差
    for(int i=0; i<4; i++) {
        imu_ekf.q[i] += K_data[i*3+0]*res[0] + K_data[i*3+1]*res[1] + K_data[i*3+2]*res[2];
    }

    // 四元数归一化
    float32_t norm = sqrtf(imu_ekf.q[0]*imu_ekf.q[0] + imu_ekf.q[1]*imu_ekf.q[1] + imu_ekf.q[2]*imu_ekf.q[2] + imu_ekf.q[3]*imu_ekf.q[3]);
    imu_ekf.q[0] /= norm; imu_ekf.q[1] /= norm; imu_ekf.q[2] /= norm; imu_ekf.q[3] /= norm;

    // 6. 更新协方差矩阵 P
    // P = (I - KH)P
    arm_matrix_instance_f32 mat_I4, mat_KH, tmp_P_new;
    float32_t I_data[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    float32_t KH_data[16];
    arm_mat_init_f32(&mat_I4, 4, 4, I_data);
    arm_mat_init_f32(&mat_KH, 4, 4, KH_data);
    arm_mat_init_f32(&tmp_P_new, 4, 4, tmp_4x4_1);

    arm_mat_mult_f32(&mat_K, &mat_H, &mat_KH);
    arm_mat_sub_f32(&mat_I4, &mat_KH, &mat_FT);           // 借用 mat_FT 存放 (I-KH)
    arm_mat_mult_f32(&mat_FT, &imu_ekf.mat_P, &tmp_P_new);
    
    // 将计算结果拷贝回 imu_ekf.P_data
    memcpy(imu_ekf.P_data, tmp_4x4_1, sizeof(imu_ekf.P_data));
}



void Get_EulerAngles(float32_t* roll, float32_t* pitch, float32_t* yaw) {
     double q0 = imu_ekf.q[0];
     double q1 = imu_ekf.q[1];
     double q2 = imu_ekf.q[2];
     double q3 = imu_ekf.q[3];

    // Roll (横滚角 - 绕 X 轴旋转)
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;

    //  Pitch (俯仰角 - 绕 Y 轴旋转)
    float32_t sin_p = 2.0f * (q0 * q2 - q3 * q1);
    if (sin_p > 1.0f)  sin_p = 1.0f;
    if (sin_p < -1.0f) sin_p = -1.0f;
    *pitch = asinf(sin_p) * 57.29578f;

    // Yaw (航向角 - 绕 Z 轴旋转)
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q3 * q3 + q2 * q2)) * 57.29578f;
}