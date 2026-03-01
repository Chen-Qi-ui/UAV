import numpy as np
import time
from pymavlink import mavutil

class RobustMagCalibrator:
    def __init__(self, port='COM6', baud=115200):
        try:
            self.master = mavutil.mavlink_connection(port, baud=baud)
            print(f"等待 {port} 心跳包...")
            self.master.wait_heartbeat()
            print("连接成功！")
        except Exception as e:
            print(f"连接失败: {e}")
            exit()

    def get_raw_mag(self):
        """1. 初级过滤：剔除明显的内存乱码包 """
        msg = self.master.recv_match(type='HIGHRES_IMU', blocking=True, timeout=0.1)
        if msg:
            mag = [msg.xmag, msg.ymag, msg.zmag]
            # 过滤全零和数值夸张的乱码（例如 10^38 或 0）
            if all(v == 0 for v in mag) or any(abs(v) > 20000 for v in mag):
                return None
            return np.array(mag)
        return None

    def solve_lls_core(self, data):
        """
        核心数学实现：文档中的 6 元线性方程组求解 
        方程形式: x^2 + ay^2 + bz^2 + cx + dy + ez + f = 0
        """
        x, y, z = data[:, 0], data[:, 1], data[:, 2]
        N = len(x)

        # 预计算各项统计均值（文档公式中的 x, y, z 各次项组合）[cite: 9]
        x2, y2, z2 = x**2, y**2, z**2
        y3, y4 = y**3, y**4
        z3, z4 = z**3, z**4

        # 构造矩阵 M (6x6) 
        M = np.zeros((6, 6))
        M[0] = [np.sum(y4),      np.sum(y2*z2),  np.sum(x*y2),   np.sum(y3),     np.sum(y2*z),   np.sum(y2)]
        M[1] = [np.sum(y2*z2),   np.sum(z4),     np.sum(x*z2),   np.sum(y*z2),   np.sum(z3),     np.sum(z2)]
        M[2] = [np.sum(x*y2),    np.sum(x*z2),   np.sum(x2),     np.sum(x*y),    np.sum(x*z),    np.sum(x)]
        M[3] = [np.sum(y3),      np.sum(y*z2),   np.sum(x*y),    np.sum(y2),     np.sum(y*z),    np.sum(y)]
        M[4] = [np.sum(y2*z),    np.sum(z3),     np.sum(x*z),    np.sum(y*z),    np.sum(z2),     np.sum(z)]
        M[5] = [np.sum(y2),      np.sum(z2),     np.sum(x),      np.sum(y),      np.sum(z),      N]

        # 构造常数项 Y 
        Y = np.array([
            -np.sum(x2 * y2),
            -np.sum(x2 * z2),
            -np.sum(x2 * x),
            -np.sum(x2 * y),
            -np.sum(x2 * z),
            -np.sum(x2)
        ])

        # 解线性方程组求 a, b, c, d, e, f
        try:
            a, b, c, d, e, f = np.linalg.solve(M, Y)
        except np.linalg.LinAlgError:
            return None

        # 换算几何参数 x0, y0, z0, A, B, C [cite: 17-22]
        x0 = -c / 2.0
        y0 = -d / (2.0 * a)
        z0 = -e / (2.0 * b)
        
        # A = sqrt(x0^2 + a*y0^2 + b*z0^2 - f) [cite: 20]
        val_A = x0**2 + a*y0**2 + b*z0**2 - f
        A = np.sqrt(max(0, val_A))
        B = A / np.sqrt(abs(a)) # [cite: 21]
        C = A / np.sqrt(abs(b)) # [cite: 22]

        return np.array([x0, y0, z0, A, B, C])

    def calibrate(self):
        print("\n>>> 开始采集磁力计数据，请进行八字绕环...")
        raw_list = []
        start_t = time.time()
        while time.time() - start_t < 40:
            mag = self.get_raw_mag()
            if mag is not None:
                raw_list.append(mag)
                if len(raw_list) % 100 == 0:
                    print(f"已采集 {len(raw_list)} 个样本...", end='\r')
            time.sleep(0.01)

        data = np.array(raw_list)
        if len(data) < 100:
            print("样本量不足！")
            return

        # 2. 二级清洗：第一次拟合剔除残差大的离群点
        print("\n正在执行第一轮初步拟合...")
        res1 = self.solve_lls_core(data)
        if res1 is None: return
        
        x0, y0, z0, A, B, C = res1
        # 计算每个点到椭球面的残差
        residuals = []
        for p in data:
            # 标准方程残差判定 [cite: 2]
            err = (p[0]-x0)**2/A**2 + (p[1]-y0)**2/B**2 + (p[2]-z0)**2/C**2
            residuals.append(abs(err - 1.0))
        
        # 剔除残差最大的 10% 的数据点
        threshold = np.percentile(residuals, 90)
        clean_data = data[np.array(residuals) < threshold]
        
        print(f"已剔除噪声点: {len(data) - len(clean_data)} 个")

        # 3. 最终精确拟合
        print("正在执行第二轮抗噪拟合...")
        final_res = self.solve_lls_core(clean_data)
        if final_res is not None:
            self.print_results(final_res)

    def print_results(self, res):
        x0, y0, z0, A, B, C = res
        print("\n" + "="*45)
        print("抗噪磁力计标定结果 (LLS-Robust)")
        print("="*45)
        print(f"1. 椭球中心 (Offset): \n   x0={x0:.4f}, y0={y0:.4f}, z0={z0:.4f}")
        print(f"2. 半轴长度 (Scale): \n   A={A:.4f}, B={B:.4f}, C={C:.4f}")
        print("-" * 45)
        # 换算为飞控使用的尺度因子 K 和偏置 b
        print(f"3. 建议补偿参数:")
        print(f"   ba_prime = [{-x0:.6f}, {-y0:.6f}, {-z0:.6f}]")
        print(f"   Ka = [{1.0/A:.6f}, {1.0/B:.6f}, {1.0/C:.6f}]")
        print("="*45)

if __name__ == "__main__":
    RobustMagCalibrator(port='COM6').calibrate()