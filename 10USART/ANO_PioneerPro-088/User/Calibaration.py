import numpy as np
import time
from pymavlink import mavutil
from scipy.optimize import least_squares

class IMUOnlyCalibrator:
    def __init__(self, port='COM6', baud=115200):
        try:
            self.master = mavutil.mavlink_connection(port, baud=baud)
            print(f"等待 {port} 心跳包...")
            self.master.wait_heartbeat()
            print("飞控已连接！")
        except Exception as e:
            print(f"连接失败: {e}")
            exit()

    def is_imu_valid(self, acc, gyro):
        """核心滤波器：专门剔除 C 程序发送的内存乱码包"""
        # 1. 剔除全零包
        if all(v == 0 for v in acc):
            return False
        # 2. 剔除极其离谱的数值（内存随机数据通常极大或极小）
        if any(abs(v) > 50 for v in acc) or any(abs(v) > 20 for v in gyro):
            return False
        return True

    def get_clean_imu(self):
        """只抓取合法的 IMU 数据包"""
        while True:
            msg = self.master.recv_match(type='HIGHRES_IMU', blocking=True, timeout=0.1)
            if msg:
                acc = [msg.xacc, msg.yacc, msg.zacc]
                gyro = [msg.xgyro, msg.ygyro, msg.zgyro]
                if self.is_imu_valid(acc, gyro):
                    return acc, gyro

    def collect_static_samples(self, label, count=300):
        print(f"\n>>> 准备采集: {label}")
        print("请调整好姿态并【保持静止】，手离开飞控...")
        
        valid_samples = []
        while len(valid_samples) < count:
            acc, gyro = self.get_clean_imu()
            
            # 滑动窗口检查稳定性，防止采集到放手瞬间的抖动
            if len(valid_samples) > 15:
                std_dev = np.std(valid_samples[-15:], axis=0)
                if np.max(std_dev) > 0.4: 
                    valid_samples = [] # 抖动过大则清空重开
                    print(f"检测到晃动，请重新静止...      ", end='\r')
                    continue
            
            valid_samples.append(acc if "加速度" in label else gyro)
            if len(valid_samples) % 50 == 0:
                print(f"进度: {len(valid_samples)}/{count} 样本已采集", end='\r')
        
        print(f"\n{label} 采集完成。")
        return np.array(valid_samples)

    def run_calibration(self):
        # 阶段 1: 陀螺仪静态标定 (计算偏置 bg')
        # 模型见图 7.2.2
        print("\n--- [第 1 阶段: 陀螺仪标定] ---")
        gyro_samples = self.collect_static_samples("陀螺仪静态", 500)
        gyro_bias = -np.mean(gyro_samples, axis=0) # 静态均值即为偏置

        # 阶段 2: 加速度计六面法标定
        # 基于公式 7.1 和 7.5 进行优化
        print("\n--- [第 2 阶段: 加速度计标定] ---")
        acc_all = []
        positions = ["正放", "倒放", "左侧立", "右侧立", "机头下", "机尾下"]
        for pos in positions:
            input(f"动作：将飞控【{pos}】并保持静止，准备好后按回车...")
            samples = self.collect_static_samples(f"加速度计-{pos}", 200)
            acc_all.append(samples)
        
        acc_raw = np.vstack(acc_all)

        # 阶段 3: LM 算法参数解算
        # 初始猜测：[d_psi, d_theta, d_phi, sx, sy, sz, bx, by, bz]
        init_p = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0]

        def accel_model(p, data):
            # 误差矩阵 Ta 结构见图 7.1.2
            T = np.array([[1, p[0], -p[1]], [-p[0], 1, p[2]], [p[1], -p[2], 1]])
            K = np.diag([p[3], p[4], p[5]])
            b = np.array([p[6], p[7], p[8]])
            return (T @ K @ (data + b).T).T

        def objective(p, data):
            # 最小化重力矢量与 g 的残差
            corrected = accel_model(p, data)
            return np.linalg.norm(corrected, axis=1) - 9.80665

        print("\n正在进行非线性最小二乘优化 (LM 算法)...")
        res = least_squares(objective, init_p, args=(acc_raw,), method='lm')

        self.print_results(res.x, gyro_bias)

    def print_results(self, pa, gb):
        print("\n" + "="*50)
        print("IMU 标定最终参数")
        print("="*50)
        print(f"1. 陀螺仪静态偏置 bg':\n   {gb}")
        print("-" * 30)
        print("2. 加速度计补偿矩阵:")
        print(f"   安装误差 Ta:\n{np.array([[1, pa[0], -pa[1]], [-pa[0], 1, pa[2]], [pa[1], -pa[2], 1]])}")
        print(f"   尺度因子 Ka: {pa[3:6]}")
        print(f"   偏置向量 ba': {pa[6:9]}")
        print("="*50)

if __name__ == "__main__":
    IMUOnlyCalibrator(port='COM6').run_calibration()