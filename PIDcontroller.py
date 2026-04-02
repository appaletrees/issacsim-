import numpy as np
from scipy.spatial.transform import Rotation as R


class PositionControllerPID:
    """
    机器人位置PID控制器（P控制，带角度约束+距离阈值）
    可在其他文件中直接 import 使用
    """

    def __init__(self, kp_linear=0.5, ki_linear=0.05, kd_linear=0.1,  # 位置PID参数
                 kp_angular=2.0, ki_angular=0.1, kd_angular=0.2,  # 角度PID参数
                 dist_tolerance=0.05,  # 位置到达容差
                 max_integral_linear=0.5, max_integral_angular=1.0):
        # 线性位移 PID 参数
        self.Kp_lin = kp_linear
        self.Ki_lin = ki_linear
        self.Kd_lin = kd_linear

        # 角度 PID 参数
        self.Kp_ang = kp_angular
        self.Ki_ang = ki_angular
        self.Kd_ang = kd_angular

        # 位置到达阈值
        self.dist_tolerance = dist_tolerance

        # 积分限幅（避免积分饱和）
        self.max_integral_lin = max_integral_linear
        self.max_integral_ang = max_integral_angular

        # --- 外部可访问的实时误差 ---
        self.current_dist_error = 0.0
        self.current_angle_error = 0.0

        # PID 内部状态变量
        self.last_dist_error = 0.0  # 上一时刻距离误差
        self.last_angle_error = 0.0  # 上一时刻角度误差
        self.integral_dist = 0.0  # 距离误差积分
        self.integral_angle = 0.0  # 角度误差积分

    def reset(self):
        """重置PID积分器和历史误差（重新导航时调用）"""
        self.last_dist_error = 0.0
        self.last_angle_error = 0.0
        self.integral_dist = 0.0
        self.integral_angle = 0.0
        self.current_dist_error = 0.0
        self.current_angle_error = 0.0

    def compute_controls(self, current_pos, current_quat, goal_pos):
        """
        计算线速度和角速度控制量
        Args:
            current_pos: [x, y, z]  当前位置
            current_quat: [qw, qx, qy, qz]  Isaac Sim 默认四元数格式
            goal_pos: [x, y]  目标点
        Returns:
            v: 线速度
            w: 角速度
        """
        # 1. 计算平面距离偏差
        dx = goal_pos[0] - current_pos[0]
        dy = goal_pos[1] - current_pos[1]
        distance = np.sqrt(dx ** 2 + dy ** 2)

        # 更新实时位置误差供外部访问
        self.current_dist_error = distance

        # 到达目标，返回零速度
        # if distance < self.dist_tolerance:
        #     self.reset()
        #     return 0.0, 0.0

        # 2. 计算目标朝向角
        target_yaw = np.arctan2(dy, dx)

        # 3. 四元数转欧拉角，获取当前偏航角
        rot = R.from_quat([current_quat[1], current_quat[2], current_quat[3], current_quat[0]])
        _, _, current_yaw = rot.as_euler('xyz')

        # 4. 角度误差归一化 [-π, π]
        angle_error = target_yaw - current_yaw
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

        # 更新实时角度误差供外部访问
        self.current_angle_error = angle_error

        # ===================== 线性速度 PID 计算 =====================
        # 误差
        error_dist = distance
        # 积分（累加）+ 限幅
        self.integral_dist += error_dist
        self.integral_dist = np.clip(self.integral_dist, -self.max_integral_lin, self.max_integral_lin)
        # 微分（当前误差 - 上一时刻误差）
        derivative_dist = error_dist - self.last_dist_error
        # 保存上一时刻误差
        self.last_dist_error = error_dist

        # PID 输出
        v_p = self.Kp_lin * error_dist
        v_i = self.Ki_lin * self.integral_dist
        v_d = self.Kd_lin * derivative_dist
        v_raw = v_p + v_i + v_d

        # 加入角度约束：只有朝向目标时才前进（和原逻辑一致）
        v = v_raw * max(0, np.cos(angle_error))

        # ===================== 角速度 PID 计算 =====================
        error_ang = angle_error
        # 积分 + 限幅
        self.integral_angle += error_ang
        self.integral_angle = np.clip(self.integral_angle, -self.max_integral_ang, self.max_integral_ang)
        # 微分
        derivative_ang = error_ang - self.last_angle_error
        self.last_angle_error = error_ang

        # PID 输出
        w_p = self.Kp_ang * error_ang
        w_i = self.Ki_ang * self.integral_angle
        w_d = self.Kd_ang * derivative_ang
        w = w_p + w_i + w_d

        return v, w